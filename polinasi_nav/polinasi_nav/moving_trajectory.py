"""C2 reference replacement preserving an explicitly scheduled initial PVA.

This is a bounded engineering construction, not time-optimal trajectory planning.
Retiming rebuilds the boundary controls; it never scales away the initial state.
"""
import numpy as np
from .continuous import ContinuousTrajectory, derivative_bound


class BoundaryTrajectory(ContinuousTrajectory):
    def __init__(self, points, config, velocity, acceleration, tangent_scale=1., speed=None):
        base=ContinuousTrajectory(points,config,tangent_scale,speed)
        self.__dict__.update(base.__dict__)
        self.initial_velocity=np.asarray(velocity,float).copy()
        self.initial_acceleration=np.asarray(acceleration,float).copy()
        if (self.initial_velocity.shape!=(3,) or self.initial_acceleration.shape!=(3,)
                or not np.all(np.isfinite(np.r_[self.initial_velocity,self.initial_acceleration]))):
            raise ValueError('invalid moving boundary')
        self.vmax=min(config['speed'],config['command_speed'],speed or config['speed'])
        alpha=config.get('adaptive_rate_limit',.25);beta=config.get('adaptive_rate_acceleration',.4)
        self.amax=config['acceleration']-self.vmax*alpha
        self.jmax=config['jerk']-3*self.amax*alpha-self.vmax*beta
        if (np.linalg.norm(self.initial_velocity)>self.vmax+1e-8
                or np.linalg.norm(self.initial_acceleration)>self.amax+1e-8):
            raise ValueError('moving boundary exceeds derivative reserve')
        # Remove initial acceleration with a short, bounded constant-jerk
        # bridge. A single long quintic can overspeed near the velocity limit;
        # increasing its duration with fixed positive initial acceleration is
        # not a monotonic remedy.
        self.prefix_controls=[];self.prefix_durations=np.array([])
        self.core_points=self.points.copy()
        core_velocity=self.initial_velocity.copy()
        if np.linalg.norm(self.initial_acceleration)>1e-9:
            t=max(.05,np.linalg.norm(self.initial_acceleration)/self.jmax*1.02)
            p=self.points[0];v=self.initial_velocity;a=self.initial_acceleration
            q=p+v*t+a*t*t/3;w=v+a*t/2
            if np.linalg.norm(w)>self.vmax+1e-9:raise ValueError('no acceleration bridge within speed reserve')
            bridge=np.array([p,p+v*t/5,p+2*v*t/5+a*t*t/20,q-2*w*t/5,q-w*t/5,q])
            self.prefix_controls=[bridge];self.prefix_durations=np.array([t])
            self.core_points=np.vstack([q,self.points[1:]])
            self.points=np.vstack([p,self.core_points])
            core_velocity=w
            base=ContinuousTrajectory(self.core_points,config,tangent_scale,speed)
        self.core_velocity=core_velocity
        velocities=np.vstack([core_velocity,*[base.sample(t)[1] for t in base.ends]])
        durations=base.durations.copy()
        # The first core leg starts in motion. A stop-to-stop seed wastes
        # that velocity; start shorter, then certify/lengthen with the SAME
        # derivative bounds. This is a feasible timing seed, not an optimum.
        if np.linalg.norm(core_velocity)>.04:durations[0]*=.75
        self._retime(durations,velocities)
        self.timing_policy='moving_boundary_C2_quintic_bounds'

    def _retime(self,durations,velocities):
        velocities=velocities.copy()
        for _ in range(64):
            controls=[];factors=[]
            for i,t in enumerate(durations):
                p,q=self.core_points[i:i+2];v,w=velocities[i:i+2]
                a=np.zeros(3)
                control=np.array([p,p+v*t/5,p+2*v*t/5+a*t*t/20,
                                  q-2*w*t/5,q-w*t/5,q])
                derivative=control;bound=[]
                for order in range(1,4):
                    derivative=(6-order)*np.diff(derivative,axis=0)/t
                    bound.append(derivative_bound(derivative,depth=2))
                factors.append(max(bound[0]/self.vmax,np.sqrt(bound[1]/self.amax),np.cbrt(bound[2]/self.jmax)))
                controls.append(control)
            if max(factors)<=1.+1e-9:
                self.controls=self.prefix_controls+controls;self.durations=np.r_[self.prefix_durations,durations]
                self.ends=np.cumsum(self.durations);self.duration=float(self.ends[-1])
                self.certificate=None
                return
            factor=max(1.02,min(1.25,max(factors)*1.01))
            durations=durations*factor
            velocities[1:]/=factor
        raise ValueError('moving boundary retiming did not converge')

    def slow(self,factor):
        if factor==1.:
            return ContinuousTrajectory.slow(self,factor)
        offset=len(self.prefix_controls)
        velocities=np.vstack([self.core_velocity,*[self.sample(t)[1]/factor for t in self.ends[offset:]]])
        self._retime(self.durations[offset:]*factor,velocities)
