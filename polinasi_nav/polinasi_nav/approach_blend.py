"""Observed-free C2 approach-to-ascent curve; never rounds through unknown."""
import math
import numpy as np
from .continuous import ContinuousTrajectory, curve_safe, evaluate
from .mapping import OCCUPIED
from .planning import brake_safe


class ApproachAscentTrajectory(ContinuousTrajectory):
    def __init__(self, start, end, config, terminal_speed):
        self.c=dict(config)
        self.c['reaction_time']=max(config['reaction_time'],config.get('tree_ascent_reaction_time',.7))
        p,q=np.asarray(start,float),np.asarray(end,float)
        rise=float(q[2]-p[2])
        if rise<=0 or not 0<terminal_speed<=config.get('tree_ascent_speed',.12):
            raise ValueError('invalid rising blend boundary')
        # Initial velocity/acceleration zero, initial geometric tangent level;
        # final tangent vertical with nonzero velocity and zero acceleration.
        h=np.r_[q[:2],p[2]]
        self.controls=[np.array([p,p,p,h,(h+q)/2,q])]
        self.durations=np.array([2.5*rise/terminal_speed])
        self.certificate=None
        self.geometry_checks=self.geometry_cache_hits=0
        self.timing_policy='C2_horizontal_to_vertical_nonzero_terminal_velocity'
        self.knot_speeds=np.array([0.,terminal_speed])
        vmax=min(config['command_speed'],config['speed'])
        alpha,beta=config.get('adaptive_rate_limit',.25),config.get('adaptive_rate_acceleration',.4)
        amax=config['acceleration']-vmax*alpha
        jmax=config['jerk']-3*amax*alpha-vmax*beta
        if min(amax,jmax)<=0:raise ValueError('no derivative headroom')
        bv,ba,bj=self.bounds()
        self.slow(max(1.,bv/vmax,math.sqrt(ba/amax),np.cbrt(bj/jmax)))
        self.knot_speeds=np.array([0.,self.sample(self.duration)[1][2]])
        self.guard=float(config.get('tree_ascent_observation_guard',0.))
        if not math.isfinite(self.guard) or self.guard<0:
            raise ValueError('invalid observation guard')
        self.approach_ascent_blend=True
        self.points=np.array([evaluate(self.controls[0],u) for u in np.linspace(0,1,41)])

    def derivatives(self,elapsed):
        values=super().derivatives(min(elapsed,self.duration))
        if elapsed>self.duration:
            # A discrete tick may straddle the handoff. Continue vertically,
            # rather than freeze position while publishing nonzero velocity.
            values[0]=values[0]+values[1]*(elapsed-self.duration)
            values[3]=np.zeros(3)
        return values

    def safe(self,grid):
        return bool(super().safe(grid) and
            curve_safe(self.controls[0]+[0.,0.,self.guard],grid))

    def safe_from(self,grid,elapsed):
        # This short transition remains inside the rolling window. Keeping its
        # complete guarded envelope is deliberately conservative.
        return self.safe(grid)

    def candidate_safe(self,grid,p,v,a,diagnostics=None):
        guarded=np.asarray(p)+[0.,0.,self.guard]
        detail={}
        safe=(grid.line_safe(p,guarded) and brake_safe(grid,p,v,self.c,a)
              and brake_safe(grid,guarded,v,self.c,a,diagnostics=detail))
        if diagnostics is not None:diagnostics.update(safe=bool(safe),guarded_braking=detail)
        return bool(safe)


def build_approach_blend(grid,start,goal,tree,config):
    start,goal=np.asarray(start,float),np.asarray(goal,float)
    detail=dict(policy='observed_under_canopy_C2_blend',tree_id=tree.get('id'),accepted=False)
    horizontal=float(np.linalg.norm(start[:2]-goal[:2]))
    if not .35<=horizontal<=config.get('approach_blend_distance',1.8) or abs(start[2]-goal[2])>.15:
        return None,dict(detail,reason='not_final_level_approach_leg')
    outward=goal[:2]-np.asarray(tree['xy'])
    if np.dot(start[:2]-goal[:2],outward)<-1e-8:
        return None,dict(detail,reason='would_cut_inside_standoff_tangent')
    occupied=np.argwhere(grid.state==OCCUPIED)
    centres=grid.centers(occupied)
    near=(np.linalg.norm(centres[:,:2]-goal[:2],axis=1)<=grid.radius+grid.res)
    overhead=near & (centres[:,2]-grid.res/2>start[2]+.3)
    if not np.any(overhead):return None,dict(detail,reason='no_observed_overhead_surface')
    ceiling=float(np.min(centres[overhead,2])-grid.res/2)
    margin=(config['drone_dimensions'][2]/2+config['clearance']+config['tracking_margin']
            +config.get('tree_ascent_observation_guard',0.))
    cap=min(ceiling-margin,float(tree['observed_stem_top'])-.4)
    detail.update(observed_ceiling_z_m=ceiling,maximum_end_z_m=cap,
                  start=start.tolist(),original_approach_goal=goal.tolist())
    for rise in (config.get('approach_blend_rise',.6),.4,.2):
        if start[2]+rise>cap or rise<=0:continue
        end=np.r_[goal[:2],start[2]+rise]
        try:
            trajectory=ApproachAscentTrajectory(start,end,config,
                min(config.get('approach_blend_terminal_speed',.08),config.get('tree_ascent_speed',.12)))
        except ValueError:continue
        if not trajectory.safe(grid):
            detail['reason']='curve_or_observation_envelope_not_free'
            continue
        if not all(trajectory.candidate_safe(grid,*trajectory.sample(t))
                   for t in np.arange(0.,trajectory.duration+.05,.05)):
            detail['reason']='no_guarded_braking_corridor'
            continue
        detail.update(accepted=True,reason='certified',end=end.tolist(),
                      duration_seconds=trajectory.duration,
                      terminal_velocity=trajectory.sample(trajectory.duration)[1].tolist(),
                      derivative_bounds=trajectory.bounds().tolist())
        trajectory.blend_diagnostics=detail
        return trajectory,detail
    return None,dict(detail,reason=detail.get('reason','insufficient_observed_under_canopy_height'))
