"""Slow continuous vertical reference with a certified stopping fallback."""
import math
import numpy as np
from .planning import Brake, brake_safe
from .mapping import vertical_guard


class ContinuousAscent:
    def __init__(self, height, config):
        self.height = float(height)
        self.c = dict(config)
        self.c['reaction_time'] = max(config['reaction_time'],config.get('tree_ascent_reaction_time',.7))
        self.speed = min(config['command_speed'],config.get('tree_ascent_speed',.12))
        # Additional observed buffer for foliage revealed during normalization.
        # Unknown stays blocked and the normal vehicle footprint is unchanged.
        self.guard = vertical_guard(config)
        if not math.isfinite(self.guard) or self.guard<0:
            raise ValueError('invalid ascent observation guard')
        if not math.isfinite(self.height) or not math.isfinite(self.speed) or self.speed<=0 or not math.isfinite(self.c['reaction_time']):
            raise ValueError('invalid continuous ascent limits')
        self.stop = None
        self.finished = False
        self.reason = ''
        self.diagnostics = {}

    def step(self, grid, p, v, a, dt):
        p,v,a = np.asarray(p),np.asarray(v),np.asarray(a)
        if self.stop is None:
            target_a = np.clip(2.*(.9*self.speed-v[2]),-self.c['acceleration'],self.c['acceleration'])
            j = np.array([0.,0.,np.clip((target_a-a[2])/dt,-self.c['jerk'],self.c['jerk'])])
            pn = p+v*dt+a*dt**2/2+j*dt**3/6
            vn = v+a*dt+j*dt**2/2
            an = a+j*dt
            detail = {}
            derivative_safe = bool(np.linalg.norm(vn)<=self.speed+1e-6 and np.linalg.norm(an)<=self.c['acceleration']+1e-6)
            segment_safe = bool(grid.line_safe(p,pn))
            stopping_safe = bool(brake_safe(grid,pn,vn,self.c,an,diagnostics=detail))
            guard_detail = {}
            guarded = pn+np.array([0.,0.,self.guard])
            guard_safe = bool(grid.line_safe(pn,guarded) and
                brake_safe(grid,guarded,vn,self.c,an,diagnostics=guard_detail))
            safe = derivative_safe and segment_safe and stopping_safe and guard_safe
            backup = Brake(pn+vn*self.c['reaction_time'],vn,an,self.c)
            # The backup's convex hull, not a few samples, bounds its maximum Z.
            local = backup.coeff*backup.duration**np.arange(6)[:,None]
            controls = [sum(math.comb(i,k)/math.comb(5,k)*local[k] for k in range(i+1)) for i in range(6)]
            height_safe = max(x[2] for x in controls) <= self.height
            self.diagnostics = dict(candidate=pn.tolist(),speed_mps=float(vn[2]),
                reaction_time=self.c['reaction_time'],height=self.height,
                derivative_safe=derivative_safe,segment_safe=segment_safe,stopping_safe=stopping_safe,
                observation_guard_m=self.guard,observation_guard_safe=guard_safe,
                observation_guard_check=guard_detail,
                braking_check=detail,height_safe=bool(height_safe),map_version=grid.version,
                map_stamp=float(grid.last_observation_stamp) if math.isfinite(grid.last_observation_stamp) else None)
            if safe and height_safe:
                return pn,vn,an
            self.reason = ('ascent_derivative_limit' if not derivative_safe else
                'observed_stopping_corridor_limit' if not segment_safe or not stopping_safe else
                'observation_guard_limit' if not guard_safe else 'desired_height_braking_limit')
            self.stop = Brake(p,v,a,self.c)
        if not self.stop.path_safe(grid):
            raise RuntimeError('ascent_backup_invalidated')
        self.stop.step(dt)
        result = self.stop.p.copy(),self.stop.v.copy(),self.stop.a.copy()
        self.finished = bool(self.stop.finished)
        self.diagnostics.update(mode='braking',stop_reason=self.reason,finished=self.finished)
        return result
