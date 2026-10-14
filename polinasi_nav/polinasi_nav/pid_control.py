"""Model-free position PID acceleration feedback over certified PVA references.

ArduPilot still owns attitude/rate stabilization. No optimizer, model fitting,
worker process or additional MAVROS publisher is created here.
"""
import time
import numpy as np
from .planning import brake_safe


def limit_norm(value, maximum):
    value = np.asarray(value, dtype=float)
    return value*min(1., maximum/max(float(np.linalg.norm(value)), 1e-12))


class PositionPID:
    def __init__(self, config):
        self.c = config
        self.kp, self.ki, self.kd = (np.asarray(config[name], dtype=float)
                                    for name in ('pid_kp', 'pid_ki', 'pid_kd'))
        self.integral = np.zeros(3)
        self.previous_acceleration = np.zeros(3)
        self.state = None
        self.last = {}
        self.wall_max_ms = 0.

    def apply(self, dt, controller, position, velocity):
        started = time.perf_counter()
        try:
            return self._apply(dt, controller, position, velocity)
        finally:
            self.wall_max_ms = max(self.wall_max_ms, (time.perf_counter()-started)*1000.)

    def _apply(self, dt, controller, position, velocity):
        if controller.state != self.state:
            self.integral[:] = 0.
            self.state = controller.state
        if controller.failure or controller.state not in ('SURVEY', 'RETURN', 'EXPLORE', 'DESCEND'):
            self.integral[:] = 0.
            self.previous_acceleration = controller.command_a.copy()
            self.last = dict(active=False, reason='inactive_or_braking')
            return controller.command.copy(), controller.command_v.copy()
        values = np.r_[position, velocity, controller.command, controller.command_v, controller.command_a, dt]
        if not np.all(np.isfinite(values)) or not 0 < dt <= .1:
            self.last=dict(active=False,reason='pid_invalid_measurement_or_dt',
                dt=float(dt) if np.isfinite(dt) else None,
                measurements_finite=bool(np.all(np.isfinite(values))))
            controller.fail('pid_invalid_measurement_or_dt')
            return controller._braking(dt if np.isfinite(dt) and 0 < dt <= .1 else .05)
        error = controller.command-np.asarray(position)
        derivative = controller.command_v-np.asarray(velocity)
        previous_integral = self.integral.copy()
        candidate_integral = np.clip(self.integral+error*dt,
                                    -self.c['pid_integral_limit'], self.c['pid_integral_limit'])
        feedback = self.kp*error+self.ki*candidate_integral+self.kd*derivative
        correction = limit_norm(feedback, self.c['pid_correction_acceleration'])
        requested = controller.command_a+correction
        bounded = limit_norm(requested, self.c['acceleration'])
        output = self.previous_acceleration+limit_norm(bounded-self.previous_acceleration, self.c['jerk']*dt)
        saturated = bool(np.linalg.norm(feedback-correction) > 1e-9
                         or np.linalg.norm(requested-output) > 1e-9)
        # Conditional integration: do not accumulate errors while authority is
        # limited by correction, acceleration, or jerk. Reset on state changes.
        if not saturated:
            self.integral = candidate_integral
        self.last = dict(active=True, reason='tracking', position_error_m=error.tolist(),
            actual_position=np.asarray(position).tolist(), actual_velocity=np.asarray(velocity).tolist(),
            reference_position=controller.command.tolist(), reference_velocity=controller.command_v.tolist(),
            nominal_acceleration=controller.command_a.tolist(),
            velocity_error_mps=derivative.tolist(), integral=self.integral.tolist(),
            correction_acceleration=correction.tolist(), output_acceleration=output.tolist(),
            saturated=saturated, commanded_jerk_mps3=float(np.linalg.norm(output-self.previous_acceleration)/dt))
        checks = []
        safe = False
        # Reduce only additional feedback if it invalidates a safe nominal
        # reference. Every candidate, including zero correction, is certified.
        for scale in (1., .5, 0.):
            requested = controller.command_a+correction*scale
            bounded = limit_norm(requested, self.c['acceleration'])
            output = self.previous_acceleration+limit_norm(bounded-self.previous_acceleration, self.c['jerk']*dt)
            actual_check, reference_check = {}, {}
            braking_config=getattr(controller,'final_braking_config',lambda:self.c)()
            actual_safe = brake_safe(controller.grid, position, velocity, braking_config, output,
                                     actual=True, diagnostics=actual_check)
            reference_safe = brake_safe(controller.grid, controller.command, controller.command_v,
                                        braking_config, output, diagnostics=reference_check)
            guard_safe=getattr(controller,'final_reference_guard',lambda a:True)(output)
            safe = bool(np.linalg.norm(output) <= self.c['acceleration']+1e-8
                        and actual_safe and reference_safe and guard_safe)
            checks.append(dict(correction_scale=scale, actual=actual_check, reference=reference_check,
                               observation_guard_safe=bool(guard_safe),
                               observation_guard_policy=getattr(controller,'final_reference_guard_diagnostics',{}).copy()))
            if safe:
                break
        self.last.update(safety_checks=checks, correction_scale=scale,
                         output_acceleration=output.tolist(),
                         commanded_jerk_mps3=float(np.linalg.norm(output-self.previous_acceleration)/dt))
        if scale < 1.:
            # Reduced authority must not wind up the integrator.
            self.integral = previous_integral
            self.last.update(integral=self.integral.tolist(), saturated=True)
        if not safe:
            # Brake from the last actually published acceleration, not the
            # nominal acceleration or an unchecked newly proposed correction.
            controller.command_a = self.previous_acceleration.copy()
            controller.fail('pid_final_safety')
            self.last.update(active=False, reason='pid_final_safety')
            return controller._braking(dt)
        controller.command_a = output
        self.previous_acceleration = output.copy()
        return controller.command.copy(), controller.command_v.copy()

    def status(self):
        return dict(active_controller='PID_PVA_feedback', pid=self.last,
                    pid_wall_max_ms=self.wall_max_ms, pid_gains=dict(
                        kp=self.kp.tolist(), ki=self.ki.tolist(), kd=self.kd.tolist()),
                    predictive_active=False, model_controls_flight=False)
