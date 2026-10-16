"""Local C2 retiming proposals; every proposal still needs full braking checks."""
import copy
import hashlib
import numpy as np
from .continuous import ContinuousTrajectory,derivative_bound
from .super_policy import reference_speed,feedback_brake_safe
from .planning import brake_safe


def retime_local(trajectory,elapsed,grid,c,factor=1.4):
    from .moving_trajectory import BoundaryTrajectory
    if not isinstance(trajectory,ContinuousTrajectory) or isinstance(trajectory,BoundaryTrajectory):
        return None  # Keep the existing fixed-moving-boundary fallback.
    n=len(trajectory.controls)
    index=min(int(np.searchsorted(trajectory.ends,elapsed,side='right')),n-1)
    durations=trajectory.durations.copy()
    points=np.vstack([*[p[0] for p in trajectory.controls],trajectory.controls[-1][-1]])
    velocities=[];accelerations=[]
    for p,t in zip(trajectory.controls,durations):
        velocities.append(5*(p[1]-p[0])/t)
        accelerations.append(20*(p[2]-2*p[1]+p[0])/t**2)
    p=trajectory.controls[-1];t=durations[-1]
    velocities.append(5*(p[-1]-p[-2])/t)
    accelerations.append(20*(p[-1]-2*p[-2]+p[-3])/t**2)
    velocities=np.asarray(velocities);accelerations=np.asarray(accelerations)
    changed=set()
    def stretch(i,scale):
        durations[i]*=scale;changed.add(i)
        for knot in (i,i+1):
            if 0<knot<n:
                velocities[knot]/=scale;accelerations[knot]/=scale**2
                changed.update(j for j in (knot-1,knot) if 0<=j<n)
    stretch(index,factor)
    vmax=reference_speed(c);amax=c['acceleration']-vmax*c.get('adaptive_rate_limit',.25)
    jmax=c['jerk']-3*amax*c.get('adaptive_rate_limit',.25)-vmax*c.get('adaptive_rate_acceleration',.4)
    caps=getattr(trajectory,'segment_speed_limits',np.full(n,vmax))
    for _ in range(16):
        controls=[p.copy() for p in trajectory.controls];violations=[]
        for i in sorted(changed):
            t=durations[i];p,q=points[i:i+2];v,w=velocities[i:i+2];a,b=accelerations[i:i+2]
            controls[i]=np.array([p,p+v*t/5,p+2*v*t/5+a*t*t/20,
                                  q-2*w*t/5+b*t*t/20,q-w*t/5,q])
            derivative=controls[i];bounds=[]
            for order in range(1,4):
                derivative=(6-order)*np.diff(derivative,axis=0)/t
                bounds.append(derivative_bound(derivative,depth=2))
            ratio=max(bounds[0]/min(vmax,caps[i]),np.sqrt(bounds[1]/amax),np.cbrt(bounds[2]/jmax))
            if ratio>1.+1e-8:violations.append((i,max(1.02,ratio*1.01)))
        if not violations:break
        for i,scale in violations:stretch(i,scale)
    else:return None
    candidate=copy.copy(trajectory);candidate.controls=controls;candidate.durations=durations
    candidate.ends=np.cumsum(durations);candidate.duration=float(candidate.ends[-1]);candidate.certificate=None
    candidate.knot_speeds=np.linalg.norm(velocities,axis=1)
    if not candidate.safe(grid):return None
    candidate.local_braking_retiming=list(getattr(trajectory,'local_braking_retiming',[]))+[
        dict(segment=index,changed_segments=sorted(changed),before_seconds=trajectory.duration,
             after_seconds=candidate.duration,initial_PVA_preserved=True)]
    candidate.timing_policy+=';local_C2_braking_retime'
    return candidate


def check_braking(trajectory,grid,c,cache):
    """Per-segment samples, including both ends; cache only in this frozen job.

    Exact control bytes and duration identify the geometry/PVA checked on the
    SAME immutable worker grid. Nothing is cached across maps or planner jobs.
    """
    checks=hits=0;begin=0.
    signature=(id(grid),grid.lo.tobytes(),grid.res,
        hashlib.sha256(grid.blocked.tobytes()).digest(),
        tuple(c.get(k) for k in ('speed','acceleration','jerk','reaction_time',
            'feedback_speed_envelope','velocity_uncertainty_mps','pid_correction_acceleration')))
    pieces=zip(trajectory.controls,trajectory.durations) if hasattr(trajectory,'controls') else [(None,trajectory.duration)]
    for index,(controls,duration) in enumerate(pieces):
        key=(signature,controls.tobytes(),float(duration)) if controls is not None else None
        if key is not None and key in cache:hits+=1;begin+=duration;continue
        for local in np.r_[np.arange(0.,duration,.05),duration]:
            elapsed=float(begin+local);p,v,a=trajectory.sample(elapsed);checks+=1;detail={}
            valid=brake_safe(grid,p,v,c,a,diagnostics=detail)
            if valid and c.get('feedback_speed_envelope'):
                valid=feedback_brake_safe(grid,p,v,a,c,detail)
            if not valid:
                return False,dict(reason='curve_braking_rejected',segment=index,
                    elapsed_seconds=elapsed,position=p.tolist(),velocity=v.tolist(),
                    acceleration=a.tolist(),check=detail),checks,hits
        if key is not None:cache.add(key)
        begin+=duration
    return True,{},checks,hits


def terminal_reserve_check(trajectory,grid,c):
    if not c.get('feedback_speed_envelope') or not hasattr(trajectory,'controls'):
        return True,{}
    end=trajectory.controls[-1][-1]
    for control in trajectory.controls[-1][-2::-1]:
        delta=end-control
        if np.linalg.norm(delta)>1e-9:
            detail={}
            safe=feedback_brake_safe(grid,end,delta/np.linalg.norm(delta)*1e-6,np.zeros(3),c,detail)
            return safe,dict(reason='terminal_robust_reserve_unavailable',position=end.tolist(),check=detail)
    return True,{}
