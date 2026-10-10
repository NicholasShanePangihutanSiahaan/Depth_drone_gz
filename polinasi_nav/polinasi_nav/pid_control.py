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
            controller.fail('pid_invalid_measurement_or_dt')
            return controller._braking(dt if np.isfinite(dt) and 0 < dt <= .1 else .05)
        error = controller.command-np.asarray(position)
        derivative = controller.command_v-np.asarray(velocity)
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
            velocity_error_mps=derivative.tolist(), integral=self.integral.tolist(),
            correction_acceleration=correction.tolist(), output_acceleration=output.tolist(),
            saturated=saturated, commanded_jerk_mps3=float(np.linalg.norm(output-self.previous_acceleration)/dt))
        safe = (np.linalg.norm(output) <= self.c['acceleration']+1e-8
                and brake_safe(controller.grid, position, velocity, self.c, output, actual=True)
                and brake_safe(controller.grid, controller.command, controller.command_v, self.c, output))
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
