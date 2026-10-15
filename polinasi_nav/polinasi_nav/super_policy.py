"""SUPER-inspired engineering policies; not a port of MINCO or CIRI.

All priors retain their provenance. The final live-map/PID gates remain owners
of execution safety; timing optimization never certifies sensor freshness.
"""
import copy
import time
from functools import lru_cache
import numpy as np
from .continuous import derivative_bound,curve_safe,evaluate
from .planning import Brake,brake_safe


@lru_cache(maxsize=64)
def _feedback_cap(speed,acceleration,jerk,feedback,uncertainty):
    config=dict(speed=speed,acceleration=acceleration,jerk=jerk)
    lo,hi=0.,speed-uncertainty
    for _ in range(16):
        trial=(lo+hi)/2
        stop=Brake(np.zeros(3),[trial+uncertainty,0.,0.],[feedback,0.,0.],config)
        if stop.max_speed<=.99*speed:lo=trial
        else:hi=trial
    return lo


def reference_speed(config,speed=None):
    limit=min(config['speed'],config['command_speed'],speed or config['speed'])
    if config.get('feedback_speed_envelope'):
        limit=min(limit,_feedback_cap(config['speed'],config['acceleration'],config['jerk'],
            config['pid_correction_acceleration'],config['velocity_uncertainty_mps']))
    return float(limit)


def feedback_brake_safe(grid,p,v,a,c,diagnostics=None):
    """Bounded planning scenario, not a proof of all estimator error vectors.

    Add a declared along-motion velocity uncertainty and maximum extra PID
    acceleration. Map/state health is checked independently at execution.
    """
    v=np.asarray(v);a=np.asarray(a)
    norm=np.linalg.norm(v)
    if norm<1e-8:return brake_safe(grid,p,v,c,a,diagnostics=diagnostics)
    direction=v/norm
    predicted_v=v+direction*c['velocity_uncertainty_mps']
    predicted_a=a+direction*c['pid_correction_acceleration']
    predicted_a*=min(1.,c['acceleration']/max(np.linalg.norm(predicted_a),1e-12))
    return brake_safe(grid,p,predicted_v,c,predicted_a,diagnostics=diagnostics)


def feedback_curve_peak(controls,duration,c):
    """Finite scenario samples used by timing search; not its final certificate."""
    peak=0.
    dc=5*np.diff(controls,axis=0)/duration
    ac=4*np.diff(dc,axis=0)/duration
    for u in np.linspace(0.,1.,9):
        v=evaluate(dc,u);a=evaluate(ac,u);speed=np.linalg.norm(v)
        if speed<1e-8:continue
        direction=v/speed
        velocity=v+direction*c['velocity_uncertainty_mps']
        acceleration=a+direction*c['pid_correction_acceleration']
        acceleration*=min(1.,c['acceleration']/max(np.linalg.norm(acceleration),1e-12))
        peak=max(peak,Brake(np.zeros(3),velocity,acceleration,c).max_speed)
    return peak


def densify(points,spacing):
    result=[np.asarray(points[0],float)];knots={0:0}
    for i,(p,q) in enumerate(zip(points[:-1],points[1:]),1):
        count=max(1,int(np.ceil(np.linalg.norm(np.asarray(q)-p)/spacing)))
        result.extend(np.asarray(p)+(np.asarray(q)-p)*u for u in np.linspace(0.,1.,count+1)[1:])
        knots[i]=len(result)-1
    return np.asarray(result),knots


def route_speed_limits(grid,points,c,maximum):
    """Local pre-profile; whole-curve braking certification still follows."""
    caps=[];reasons=[]
    for p,q in zip(points[:-1],points[1:]):
        direction=(q-p)/np.linalg.norm(q-p)
        cap=0.;rejected=[]
        samples=np.linspace(p,q,max(2,int(np.ceil(np.linalg.norm(q-p)/.5))+1))
        for factor in (1.,.85,.7,.5,.3):
            speed=maximum*factor
            checks=[]
            for point in samples:
                detail={}
                if not feedback_brake_safe(grid,point,direction*speed,np.zeros(3),c,detail):
                    rejected.append(detail.get('reason'));break
            else:
                cap=speed;break
        if cap==0.:raise ValueError('no robust route braking envelope')
        caps.append(cap);reasons.append(rejected)
    return np.asarray(caps),dict(policy='sampled_route_feedback_braking_then_full_curve_check',
        segment_speed_limits_mps=caps,rejected_reasons=reasons,
        velocity_uncertainty_mps=c['velocity_uncertainty_mps'],
        whole_curve_certificate=False)


def voxel_corridor(grid,controls,expansion=3):
    """Conservative convex BOX in the already inflated voxel configuration space.

    This baseline is deliberately named voxel corridor, never CIRI. Unknown
    and virtual obstacles are included in blocked. Returns None if the initial
    hull box crosses blocked cells; callers retain recursive curve checking.
    """
    lo=grid.indices(np.min(controls,axis=0)-1e-9)
    hi=grid.indices(np.max(controls,axis=0)+1e-9)
    def safe(lower,upper):
        return (grid.inside(lower) and grid.inside(upper) and not np.any(grid.blocked[
            tuple(slice(int(a),int(b)+1) for a,b in zip(lower,upper))]))
    if not safe(lo,hi):return None
    for _ in range(expansion):
        for axis in range(3):
            for side in (-1,1):
                lower,upper=lo.copy(),hi.copy()
                if side<0:lower[axis]-=1
                else:upper[axis]+=1
                if safe(lower,upper):lo,hi=lower,upper
    return (grid.lo+lo*grid.res+1e-8,grid.lo+(hi+1)*grid.res-1e-8)


def optimise_durations(trajectory,grid,c):
    """Bounded coordinate search, fixed endpoint PVA at every knot.

    Optimizes duration within the existing quintic representation, not the
    global minimum-time problem. Every accepted change certifies derivative
    hull bounds and geometry; the caller then checks stopping for the curve.
    """
    started=time.perf_counter();before=trajectory.duration
    vmax=reference_speed(c,getattr(trajectory,'vmax',None))
    amax=c['acceleration']-vmax*c.get('adaptive_rate_limit',.25)
    jmax=c['jerk']-3*amax*c.get('adaptive_rate_limit',.25)-vmax*c.get('adaptive_rate_acceleration',.4)
    controls=[p.copy() for p in trajectory.controls];durations=trajectory.durations.copy()
    tried=accepted=box_hits=0
    first=len(getattr(trajectory,'prefix_controls',[]))
    for i in range(first,len(controls)):
        old=controls[i];duration=durations[i]
        p,q=old[0],old[-1]
        v=5*(old[1]-p)/duration;w=5*(q-old[-2])/duration
        a=20*(old[2]-2*old[1]+p)/duration**2
        b=20*(q-2*old[-2]+old[-3])/duration**2
        corridor=voxel_corridor(grid,old)
        lo,hi=.55*duration,duration
        chosen=old;best=duration
        for _ in range(7):
            t=(lo+hi)/2;tried+=1
            candidate=np.array([p,p+v*t/5,p+2*v*t/5+a*t*t/20,
                                q-2*w*t/5+b*t*t/20,q-w*t/5,q])
            derivative=candidate;bounds=[]
            for order in range(1,4):
                derivative=(6-order)*np.diff(derivative,axis=0)/t
                bounds.append(derivative_bound(derivative,depth=2))
            valid=all(value<=limit+1e-9 for value,limit in zip(bounds,(min(vmax,getattr(trajectory,'segment_speed_limits',np.full(len(controls),vmax))[i]),amax,jmax)))
            if valid and c.get('feedback_speed_envelope'):
                valid=feedback_curve_peak(candidate,t,c)<=.995*c['speed']+1e-9
            if valid:
                in_box=(corridor is not None and np.all(candidate>=corridor[0]) and np.all(candidate<=corridor[1]))
                if in_box:box_hits+=1
                valid=in_box or curve_safe(candidate,grid)
            if valid:chosen=candidate;best=t;hi=t
            else:lo=t
        if best<duration-1e-7:accepted+=1
        controls[i]=chosen;durations[i]=best
    trajectory.controls=controls;trajectory.durations=durations
    trajectory.ends=np.cumsum(durations);trajectory.duration=float(trajectory.ends[-1]);trajectory.certificate=None
    trajectory.duration_optimisation=dict(policy='bounded_fixed_PVA_quintic_coordinate_search_not_MINCO',
        before_seconds=float(before),after_seconds=trajectory.duration,attempts=tried,
        shortened_segments=accepted,voxel_corridor_hits=box_hits,wall_ms=(time.perf_counter()-started)*1000,
        final_braking_certificate_required=True)
    trajectory.timing_policy+=';bounded_duration_search'


def explicit_backup(grid,p,v,a,c,optimise=False):
    """Materialized PVA-continuous stop; no frame or setpoint publisher.

    Runtime uses the bounded straight-stop candidate. Planner/evaluation can
    search duration and terminal positions, including lateral/vertical detours.
    Final curve checks use both inflated body geometry and derivative hulls.
    """
    base=Brake(p,v,a,c)
    candidates=[base];p=np.asarray(p);v=np.asarray(v);a=np.asarray(a)
    if optimise and np.linalg.norm(v)>.02:
        direction=v/np.linalg.norm(v);side=np.cross(direction,[0.,0.,1.])
        if np.linalg.norm(side)<1e-8:side=np.array([1.,0.,0.])
        side/=np.linalg.norm(side)
        end=sum(base.duration**k*base.coeff[k] for k in range(6))
        for scale in (.7,.85,1.,1.2):
            t=base.duration*scale
            for offset in (np.zeros(3),side*.3,-side*.3,np.array([0.,0.,.2]),np.array([0.,0.,-.2])):
                q=p+(end-p)*scale+offset
                trial=copy.copy(base);coeff=np.zeros((6,3));coeff[:3]=[p,v,a/2]
                matrix=[[t**3,t**4,t**5],[3*t*t,4*t**3,5*t**4],[6*t,12*t*t,20*t**3]]
                coeff[3:]=np.linalg.solve(matrix,[q-p-v*t-a*t*t/2,-v-a*t,-a])
                trial.coeff=coeff;trial.duration=t;candidates.append(trial)
    best=None
    for candidate in candidates:
        t=candidate.duration
        control=np.zeros((6,3))
        import math
        for i in range(6):
            for k in range(i+1):control[i]+=math.comb(i,k)/math.comb(5,k)*candidate.coeff[k]*t**k
        derivative=control;bounds=[]
        for order in range(1,4):
            derivative=(6-order)*np.diff(derivative,axis=0)/t
            bounds.append(derivative_bound(derivative,depth=2))
        if any(b>limit+1e-8 for b,limit in zip(bounds,(c['speed'],c['acceleration'],c['jerk']))):continue
        candidate.max_speed=bounds[0]
        detail={}
        if not candidate.path_safe(grid,False,detail):continue
        if best is None or t<best.duration:best=candidate
    if best is None:return None,dict(valid=False,reason='no_bounded_collision_free_stop')
    end=sum(best.duration**k*best.coeff[k] for k in range(6))
    return best,dict(valid=True,policy='explicit_PVA_stop_with_live_voxel_check',
        duration_seconds=float(best.duration),end_position=end.tolist(),
        max_speed_bound_mps=best.max_speed,map_version=int(grid.version),
        provenance='measured_plus_explicit_FOV_prior' if c.get('fixed_fov_assumptions') else 'measured_free',
        conditional_on_FOV_assumptions=bool(c.get('fixed_fov_assumptions')))


def select_backup_switch(trajectory,elapsed,lead,grid,c):
    """Latest feasible of three bounded future PVA boundaries.

    A small receding search, not SUPER's joint differentiable optimization.
    The old stopping trajectory remains committed until an adoption succeeds.
    """
    best=None;rejections=[]
    for extra in (0.,.05,.1):
        t=elapsed+lead+extra
        if t>=trajectory.duration-.3:continue
        p,v,a=trajectory.sample(t)
        if np.linalg.norm(v)<.04:continue
        detail={}
        if not feedback_brake_safe(grid,p,v,a,c,detail):
            rejections.append(detail.get('reason'));continue
        stop,status=explicit_backup(grid,p,v,a,c)
        if stop is None:
            rejections.append(status['reason']);continue
        score=t-.1*stop.duration
        if best is None or score>best[0]:best=(score,t,status)
    if best is None:return None,dict(reason='no_future_stop_certificate',rejections=rejections)
    return best[1],dict(policy='bounded_latest_feasible_PVA_switch_not_joint_MINCO',
        scheduled_elapsed=best[1],stop=best[2],rejections=rejections)


def backup_grid(controller):
    if (controller.c.get('explicit_backup_enabled') and controller.c.get('scan_adaptive_height')
            and getattr(controller,'tree_phase',None) in ('SCAN','ASCEND')):
        from .canopy_scan import canopy_guard_grid
        return canopy_guard_grid(controller.grid,controller.c.get('tree_ascent_observation_guard',.2))
    return controller.grid
