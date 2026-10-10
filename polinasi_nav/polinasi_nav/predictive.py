"""Reduced-order constrained PVA MPC. Not rotor-model NMPC or L1-NMPC.

The worker owns no ROS publisher. Only acceleration feedforward is optimized;
the certified nominal position/velocity reference and flight manager remain.
"""
import multiprocessing
import signal
import time
from collections import deque
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.optimize import minimize
from .adaptive_model import GuardedRLS, load_validated_seed, stable
from .planning import brake_safe


def initialise_worker():
    # Parent handles Ctrl+C and closes the pool. Group TERM still stops a hung
    # child during launcher cleanup; ignore only duplicate child SIGINT.
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def warm_worker():
    return True


def predict(job, controls):
    p, v = np.array(job['position']), np.array(job['velocity'])
    theta, delays = np.asarray(job['theta']), np.asarray(job['delays'])
    h = job['step']
    n = len(controls)
    times = np.arange(n)*h
    history = np.asarray(job['history'])
    reference = np.asarray(job['reference'])
    all_times = np.r_[history[:, 0]-job['stamp'], times]
    all_p = np.vstack((history[:, 1:4], reference[:, :3]))
    all_v = np.vstack((history[:, 4:7], reference[:, 3:6]))
    all_a = np.vstack((history[:, 7:10], controls))
    states, accelerations = [], []
    for k in range(n):
        for sub in range(2):
            t = (k+sub/2)*h-delays
            pc = np.array([np.interp(t[a], all_times, all_p[:, a]) for a in range(3)])
            vc = np.array([np.interp(t[a], all_times, all_v[:, a]) for a in range(3)])
            # Zero-order-held acceleration controls, matching ROS command history.
            ids = np.clip(np.searchsorted(all_times, t, side='right')-1, 0, len(all_times)-1)
            ac = all_a[ids, np.arange(3)]
            acceleration = theta[:, 0]*(pc-p)+theta[:, 1]*(vc-v)+theta[:, 2]*ac+theta[:, 3]
            p, v = p+h/2*v, v+h/2*acceleration
        states.append(np.r_[p, v])
        accelerations.append(acceleration)
    return np.asarray(states), np.asarray(accelerations)


def solve(job):
    started = time.perf_counter()
    reference = np.asarray(job['reference'])
    n = len(reference)
    previous = np.asarray(job['previous_acceleration'])
    h, first_dt = job['step'], job['first_dt']
    nominal = reference[:, 6:9]
    # Dynamics are affine in the PVA acceleration sequence. Condense once;
    # numerical optimizer iterations need only tiny matrix products.
    zero_states, zero_acceleration = predict(job, np.zeros((n, 3)))
    basis_states, basis_acceleration = [], []
    for i in range(n*3):
        basis = np.zeros(n*3)
        basis[i] = 1.
        states, acceleration = predict(job, basis.reshape(n, 3))
        basis_states.append((states-zero_states).ravel())
        basis_acceleration.append((acceleration-zero_acceleration).ravel())
    state_matrix = np.asarray(basis_states).T
    acceleration_matrix = np.asarray(basis_acceleration).T
    def rollout(flat):
        return ((zero_states.ravel()+state_matrix@flat).reshape(n, 6),
                (zero_acceleration.ravel()+acceleration_matrix@flat).reshape(n, 3))
    def objective(flat):
        u = flat.reshape(n, 3)
        states, _ = rollout(flat)
        # Terminal reference is held beyond this short certified lookahead.
        desired = np.vstack((reference[1:, :6], reference[-1:, :6]))
        error = states-desired
        changes = np.diff(np.vstack((previous, u)), axis=0)
        return float(150*np.sum(error[:, :3]**2)+10*np.sum(error[:, 3:]**2)
                     +.3*np.sum((u-nominal)**2)+.2*np.sum(changes**2))
    def constraints(flat):
        u = flat.reshape(n, 3)
        states, acceleration = rollout(flat)
        changes = np.diff(np.vstack((previous, u)), axis=0)
        limits = np.r_[first_dt, np.full(n-1, h)]*job['jerk']
        desired = np.vstack((reference[1:, :3], reference[-1:, :3]))
        return np.r_[job['acceleration']**2-np.sum(u*u, axis=1),
            limits**2-np.sum(changes*changes, axis=1),
            job['speed']**2-np.sum(states[:, 3:]**2, axis=1),
            job['acceleration']**2-np.sum(acceleration*acceleration, axis=1),
            job['tube']**2-np.sum((states[:, :3]-desired)**2, axis=1)]
    result = minimize(objective, np.tile(previous, (n, 1)).ravel(), method='SLSQP',
        bounds=[(-job['acceleration'], job['acceleration'])]*(n*3),
        constraints=[dict(type='ineq', fun=constraints)],
        options=dict(maxiter=35, ftol=1e-7))
    u = result.x.reshape(n, 3)
    states, _ = predict(job, u)
    valid = bool(result.success and np.all(np.isfinite(u)) and np.min(constraints(result.x)) >= -1e-7)
    finished = time.perf_counter()
    return dict(stamp=job['stamp'], controls=u, states=states,
        ok=valid, reason=str(result.message), wall_ms=(time.perf_counter()-started)*1000,
        model_version=job['model_version'], generation=job.get('generation', 0),
        queue_wall_ms=max(0., (started-job.get('submitted_wall', started))*1000),
        finished_wall=finished)


class PredictiveControl:
    def __init__(self, config):
        self.c = config
        if not config.get('predictive_simulation_only'):
            raise ValueError('predictive control is simulation only')
        theta, self.delays, source = load_validated_seed(config.get('adaptive_model_file'), config)
        if source == 'nominal':
            raise ValueError('active MPC requires a validated identified model')
        self.base = np.asarray(theta)
        self.estimators = [GuardedRLS(initial=t, window=120) for t in theta]
        self.theta = self.base.copy()
        self.history = deque(maxlen=100)
        self.previous_measurement = None
        self.previous_acceleration = np.zeros(3)
        self.worker = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'),
            initializer=initialise_worker)
        self.warmup = self.worker.submit(warm_worker)
        self.future = self.plan = None
        self.activated = False
        self.started = None
        self.model_version = self.solves = self.applied = self.updates = 0
        self.wall_ms = 0.
        self.commanded_jerk = 0.
        self.reason = 'warmup'
        self.generation = 0
        self.future_generation = 0
        self.state = None
        self.recoveries = 0
        self.recovery_started = None
        self.awaiting_fresh = False
        self.queue_ms = self.receive_ms = self.solution_age = self.measurement_age = 0.

    def ready(self):
        if not self.warmup.done():
            return False
        try:
            return bool(self.warmup.result())
        except Exception:
            self.reason = 'mpc_worker_start_failure'
            return False

    def sync_state(self, state):
        if state == self.state:
            return
        self.state = state
        self.generation += 1
        self.plan = None
        self.started = None
        # A running task cannot be cancelled. Keep its handle and discard its
        # result by generation, rather than queue more work behind it.
        if self.future is not None and self.future.cancel():
            self.future = None

    def recover(self, now, dt, controller):
        self.plan = None
        if controller.state not in ('SURVEY', 'RETURN') or self.recoveries >= 3:
            self.reason = 'mpc_stale_solution'
            controller.fail(self.reason)
        else:
            self.recoveries += 1
            self.recovery_started = now
            self.awaiting_fresh = True
            self.reason = 'mpc_recovering'
            # Reuse the existing jerk-limited, map-checked Brake. Do not follow
            # nominal moving targets while waiting for a fresh MPC solution.
            controller.fail(self.reason)
            pending = getattr(controller, 'pending_plan', None)
            if pending is not None:
                pending.cancel()
                controller.pending_plan = None
        return controller._braking(dt)

    def poll_recovery(self, now, controller, position=None, velocity=None):
        if controller.failure != 'mpc_recovering':
            return
        if position is not None:
            reason = controller.health.reason(now, self.c, controller.sensor_mode)
            if reason or not brake_safe(controller.grid, position, velocity, self.c,
                                        controller.command_a, actual=True):
                controller.failure = self.reason = reason or 'insufficient_stopping_clearance'
                return
            if np.linalg.norm(position-controller.command) > self.c['tracking_limit']:
                controller.failure = self.reason = 'tracking_error'
                return
        if now-self.recovery_started > 4.:
            controller.failure = self.reason = 'mpc_recovery_timeout'
            return
        if controller.state == 'HOLD_ABORT' and (velocity is None or np.linalg.norm(velocity) < .08):
            controller.failure = ''
            controller.state = controller.resume_state
            controller.goal = controller.trajectory = None
            controller.elapsed = 0.
            self.reason = 'mpc_replanning'

    def observe(self, now, position, velocity):
        old = self.previous_measurement
        self.previous_measurement = (now, np.array(position), np.array(velocity))
        if not self.c.get('predictive_adaptation', False) or old is None:
            return
        dt = now-old[0]
        if not .015 <= dt <= .08:
            return
        history = list(self.history)
        changed = False
        for axis, estimator in enumerate(self.estimators):
            candidates = [r for r in history if r[0] <= old[0]-self.delays[axis]]
            if not candidates or old[0]-self.delays[axis]-candidates[-1][0] > .12:
                continue
            row = candidates[-1]
            phi = [row[1+axis]-old[1][axis], row[4+axis]-old[2][axis], row[7+axis], 1.]
            if estimator.update(phi, (velocity[axis]-old[2][axis])/dt, dt):
                # Limit promotion around independently validated coefficients.
                limits = np.r_[np.maximum(.2*np.abs(self.base[axis, :3]), .02), .01]
                candidate = np.clip(estimator.theta, self.base[axis]-limits, self.base[axis]+limits)
                if stable(candidate):
                    changed |= bool(np.max(np.abs(candidate-self.theta[axis])) > 1e-8)
                    self.theta[axis] = candidate
                    self.updates += 1
        if changed:
            self.model_version += 1

    def record(self, now, p, v, a):
        self.history.append(np.r_[now, p, v, a])
        self.previous_acceleration = np.array(a)

    def apply(self, now, dt, controller, position, velocity, reference, alignment_error):
        if alignment_error > .01:
            self.reason = 'mpc_frame_alignment'
            controller.fail(self.reason)
            return controller._braking(dt)
        self.started = now if self.started is None else self.started
        self.observe(controller.health.pose_stamp, position, velocity)
        if self.future is not None and self.future.done():
            try:
                candidate = self.future.result()
                self.wall_ms = max(self.wall_ms, candidate['wall_ms'])
                self.solves += 1
                current_generation = candidate.get('generation', self.generation) == self.generation
                if current_generation and not candidate['ok']:
                    raise ValueError(candidate['reason'])
                self.queue_ms = max(self.queue_ms, candidate.get('queue_wall_ms', 0.))
                self.receive_ms = max(self.receive_ms,
                    max(0., (time.perf_counter()-candidate.get('finished_wall', time.perf_counter()))*1000))
                if current_generation:
                    self.plan = candidate
            except Exception as exc:
                if self.future_generation == self.generation:
                    self.reason = 'mpc_solver_failure:'+str(exc)
                    controller.fail('mpc_solver_failure')
            self.future = None
        if controller.failure:
            return controller._braking(dt)
        if not self.history:
            self.reason = 'warmup_nominal'
            return controller.command.copy(), controller.command_v.copy()
        job = dict(stamp=now, position=np.array(position), velocity=np.array(velocity),
                theta=self.theta.copy(), delays=self.delays, history=list(self.history),
                reference=reference, step=.1, first_dt=dt,
                previous_acceleration=self.previous_acceleration.copy(),
                acceleration=self.c['acceleration'], jerk=self.c['jerk'], speed=self.c['speed'],
                tube=min(.06, self.c['tracking_margin']/2), model_version=self.model_version,
                generation=self.generation, submitted_wall=time.perf_counter())
        self.measurement_age = now-controller.health.pose_stamp
        if self.future is None:
            self.future_generation = self.generation
            self.future = self.worker.submit(solve, job)
        if self.plan is None:
            self.reason = 'warmup_nominal'
            if now-self.started > 2.:
                return self.recover(now, dt, controller)
            return controller.command.copy(), controller.command_v.copy()
        age = now-self.plan['stamp']
        self.solution_age = age
        if not 0 <= age <= .25 or self.model_version-self.plan['model_version'] > 10:
            return self.recover(now, dt, controller)
        index = min(int(age/.1), len(self.plan['controls'])-1)
        requested = self.plan['controls'][index]
        delta = requested-self.previous_acceleration
        final_a = self.previous_acceleration+delta*min(1., self.c['jerk']*dt/max(np.linalg.norm(delta), 1e-12))
        self.commanded_jerk = float(np.linalg.norm(final_a-self.previous_acceleration)/dt)
        # Recheck the response of the ACTUAL applied acceleration, not just the
        # optimizer output. A short rollout uses the latest measured state/map.
        job.update(previous_acceleration=final_a)
        controls = self.plan['controls'][index:].copy()
        controls = np.vstack((controls, np.tile(controls[-1], (index, 1))))
        # State is current; restart the horizon with current reference/delays.
        controls[0] = final_a
        states, acceleration = predict(job, controls)
        previous = np.asarray(position)
        valid = np.linalg.norm(final_a) <= self.c['acceleration']+1e-8
        desired = np.vstack((reference[1:, :3], reference[-1:, :3]))
        valid &= np.max(np.linalg.norm(states[:, :3]-desired, axis=1)) <= job['tube']+1e-6
        for state, a in zip(states, acceleration):
            valid &= (np.linalg.norm(state[3:]) <= self.c['speed']+1e-6
                      and np.linalg.norm(a) <= self.c['acceleration']+1e-6
                      and controller.grid.line_safe(previous, state[:3], actual=True))
            previous = state[:3]
        valid &= brake_safe(controller.grid, position, velocity, self.c, acceleration[0], actual=True)
        valid &= brake_safe(controller.grid, controller.command, controller.command_v, self.c, final_a)
        if not valid:
            self.reason = 'mpc_final_safety'
            controller.fail(self.reason)
            return controller._braking(dt)
        controller.command_a = final_a
        self.awaiting_fresh = False
        self.activated, self.reason = True, 'active'
        self.applied += 1
        return controller.command.copy(), controller.command_v.copy()

    def status(self):
        return dict(predictive_controller='PVA_response_MPC_not_full_NMPC',
            predictive_active=self.activated, predictive_reason=self.reason,
            predictive_solves=self.solves, predictive_applied=self.applied,
            predictive_worker_wall_max_ms=self.wall_ms,
            predictive_worker_ready=self.ready(), predictive_recoveries=self.recoveries,
            predictive_queue_wall_max_ms=self.queue_ms,
            predictive_receive_wall_max_ms=self.receive_ms,
            predictive_solution_age_seconds=self.solution_age,
            predictive_measurement_age_seconds=self.measurement_age,
            predictive_commanded_jerk_mps3=self.commanded_jerk,
            predictive_model_updates=self.updates, predictive_model_version=self.model_version,
            predictive_parameters=self.theta.tolist(), model_controls_flight=self.activated)

    def close(self):
        self.worker.shutdown(wait=True, cancel_futures=True)
