"""Bounded empirical re-observation evidence, NOT a dynamic-object classifier.

Only measured rays add evidence. Sticky occupancy, restored history and launch
priors cannot manufacture stability. A changing/noisy region remains uncertain.
"""
import math
import time
import numpy as np
from .mapping import FREE
from .planning import Brake


class LocalMapTrust:
    def __init__(self, config):
        self.c=config
        self.records={}
        self.stamp=-math.inf
        self.diagnostics={}

    def reset(self):
        self.records.clear()
        self.stamp=-math.inf

    def observe(self, grid, centre):
        stamp=grid.last_observation_stamp
        if not math.isfinite(stamp) or stamp<=self.stamp:
            return
        self.stamp=stamp
        # Small cube follows the vehicle; keys stay fixed in world coordinates.
        low=np.maximum(grid.indices(np.asarray(centre)-2.),0)
        high=np.minimum(grid.indices(np.asarray(centre)+2.)+1,grid.shape)
        if np.any(high<=low):
            self.records.clear()
            return
        ids=np.indices(tuple(high-low)).reshape(3,-1).T+low
        keys=np.rint((grid.centers(ids)-grid.res/2)/grid.res).astype(int)
        previous=self.records
        self.records={}
        for index,key in zip(ids,keys):
            key=tuple(key); index=tuple(index)
            value=int(grid.state[index])
            old=previous.get(key)
            if old is not None and old[0]!=value:
                old=None
            if grid.observed[index]:
                if old is None or stamp-old[2]>self.c['map_trust_revisit_seconds']:
                    old=(value,stamp,stamp,1)
                elif stamp-old[2]>=.2-1e-9:
                    old=(value,old[1],stamp,old[3]+1)
            if old is not None:
                self.records[key]=old

    def assess(self, grid, now, p, v, a, actual, base_age, health_ok, quality_ok, ascending=False,actual_v=None):
        started=time.perf_counter()
        cap=self.c['map_trust_ascent_max_age'] if ascending else self.c['map_trust_max_age']
        cap=max(base_age,cap)
        age=now-grid.last_observation_stamp
        speed=max(float(np.linalg.norm(v)),float(np.linalg.norm(actual_v)) if actual_v is not None else 0.)
        detail=dict(base_limit_seconds=base_age,effective_limit_seconds=base_age,
            measurement_age_seconds=float(age) if math.isfinite(age) else None,
            extended=False,reason='insufficient_reobservations',stable_fraction=0.,
            policy='empirical_all_corridor_voxels;not_motion_prediction')
        def finish(reason,limit=base_age):
            detail.update(reason=reason,effective_limit_seconds=limit,extended=limit>base_age,
                wall_ms=(time.perf_counter()-started)*1000)
            self.diagnostics=detail
            return -1e-9<=age<=limit+1e-9
        if not health_ok or not quality_ok:
            # Don't let previously good evidence survive uncertain localisation.
            self.reset()
            return finish('sensor_or_localisation_quality')
        if age<0 or not math.isfinite(age):
            return finish('invalid_map_time')
        if age<=base_age:
            return finish('fresh_base_limit')
        if speed>self.c['map_trust_max_speed']:
            return finish('speed_limit_for_extension')
        guard=self.c.get('tree_ascent_observation_guard',0.) if ascending else 0.
        reaction=max(self.c['reaction_time'],self.c.get('tree_ascent_reaction_time',.7) if ascending else 0.)
        braking=Brake(np.asarray(p)+np.asarray(v)*reaction,np.asarray(v),np.asarray(a),self.c)
        local=braking.coeff*braking.duration**np.arange(6)[:,None]
        controls=np.asarray([sum(math.comb(i,k)/math.comb(5,k)*local[k] for k in range(i+1)) for i in range(6)])
        points=np.vstack([p,actual,controls, np.asarray(p)+np.asarray(v)*cap])
        # Expand for body, quantisation, tracking error and the distance that
        # could be travelled during the longer observation age. Never shrink
        # existing inflation or mark unknown free.
        travel=speed*cap+np.linalg.norm(a)*cap*cap/2
        pad=np.array([grid.radius,grid.radius,grid.half_z])+grid.res+travel
        low=grid.indices(points.min(axis=0)-pad)
        high=grid.indices(points.max(axis=0)+pad+[0.,0.,guard])+1
        if np.any(low<0) or np.any(high>grid.shape):
            return finish('corridor_outside_window')
        ids=np.indices(tuple(high-low)).reshape(3,-1).T+low
        values=grid.state[tuple(ids.T)]
        detail.update(corridor_voxels=len(ids),unknown_voxels=int(np.count_nonzero(values==-1)),
            occupied_voxels=int(np.count_nonzero(values==1)),corridor_min=grid.centers(low).tolist(),
            corridor_max=grid.centers(high-1).tolist())
        if np.any(values!=FREE):
            return finish('corridor_unknown_or_occupied')
        keys=np.rint((grid.centers(ids)-grid.res/2)/grid.res).astype(int)
        stable=0
        for key in keys:
            evidence=self.records.get(tuple(key))
            if evidence is not None:
                value,first,last,count=evidence
                stable+=int(value==FREE and count>=3 and
                    last-first>=self.c['map_trust_stable_seconds']-1e-9 and
                    0<=now-last<=self.c['map_trust_revisit_seconds'])
        detail['stable_fraction']=stable/max(1,len(ids))
        if stable!=len(ids):
            return finish('insufficient_reobservations')
        return finish('stable_reobserved_corridor',cap)
