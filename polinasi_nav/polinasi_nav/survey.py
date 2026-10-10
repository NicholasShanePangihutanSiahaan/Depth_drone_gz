"""Separate waypoint mapping mission, reusing certified navigation/flight gates."""
import copy
import math
import multiprocessing
import time
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from .control import MissionController
from .continuous import ContinuousTrajectory, AdaptivePhase
from .planning import brake_safe


def prepare_survey_snapshot(clone, now, position, range_value, range_stamp):
    started = time.perf_counter()
    ready = MissionController._prepare_trajectory(clone, now, position, range_value, range_stamp)
    clone.planner_wall_max_ms = max(clone.planner_wall_max_ms, (time.perf_counter()-started)*1000.)
    clone.planner_stages_ms['worker_total'] = (time.perf_counter()-started)*1000.
    clone.planner_finished_ns = time.monotonic_ns()
    return clone, ready


class SurveyController(MissionController):
    def __init__(self, config, grid, sensor_mode=False, asynchronous=True):
        config = copy.deepcopy(config)
        points = np.asarray(config.get('survey_waypoints', []), dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.all(np.isfinite(points)):
            raise ValueError('survey_waypoints must be a nonempty list of finite XYZ points')
        lo = np.asarray(config['bounds_min']) if config.get('navigation_window_size') else grid.lo
        hi = np.asarray(config['bounds_max']) if config.get('navigation_window_size') else grid.lo+np.asarray(grid.shape)*grid.res
        if not np.all((points >= lo) & (points < hi)):
            raise ValueError('survey waypoint outside navigation map bounds')
        dwell = config.get('survey_dwell', 1.)
        if not isinstance(dwell, (float, int)) or not math.isfinite(dwell) or dwell <= 0:
            raise ValueError('survey_dwell must be finite and positive')
        config.update(mock_flower_events=False, terrain_following=False,
                      inspection_mode='viewpoints', inspection_dwell=dwell)
        super().__init__(config, grid, sensor_mode)
        self.airborne_state = 'SURVEY'
        self.exploration_states = ('SURVEY', 'RETURN')
        self.waypoints, self.waypoint_index = points, 0
        self.nominal = []  # Never label the survey route as an inspection orbit.
        self.wait_for_scan = None
        self.survey_now = 0.
        self.survey_started = None
        self.executor = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn')) if asynchronous else None
        self.pending_plan = None
        self.planner_wall_max_ms = 0.
        self.batch_end_index = None
        self.batch_knots = {}
        self.pass_arrival = None
        self.phase = AdaptivePhase(config)
        self.adaptive_reason = ''
        self.speed_diagnostics = {}
        self.command_jerk = np.zeros(3)
        self.continuous_passes = 0
        self.prefetched_plan = None
        self.prefetch_key = None
        self.prefetch_submitted = self.prefetch_used = self.prefetch_rejected = 0
        self.prefetch_reason = ''

    def _planning_clone(self):
        clone = object.__new__(SurveyController)
        excluded = ('executor', 'pending_plan', 'prefetched_plan', 'prefetch_key')
        for key, value in self.__dict__.items():
            if key not in excluded:
                setattr(clone, key, copy.deepcopy(value))
        clone.executor = clone.pending_plan = clone.prefetched_plan = clone.prefetch_key = None
        clone.planner_submitted_ns = time.monotonic_ns()
        return clone

    def _prefetch_next(self, now):
        if (not self.c.get('survey_prefetch', False) or self.executor is None or self.failure
                or self.state != 'SURVEY' or self.trajectory is None or self.pending_plan is not None
                or self.batch_end_index is None or self.batch_end_index+1 >= len(self.waypoints)):
            return
        if self.prefetched_plan is not None:
            return
        remaining = self.trajectory.duration-self.elapsed
        if remaining > self.c.get('survey_prefetch_seconds', 3.):
            return
        clone = self._planning_clone()
        clone.command = self.goal.copy()
        clone.command_v = clone.command_a = np.zeros(3)
        clone.waypoint_index = self.batch_end_index+1
        clone.goal = clone.trajectory = None
        clone.wait_for_scan = None
        clone.dwell = clone.elapsed = 0.
        self.prefetch_key = (clone.waypoint_index, clone.command.copy())
        self.prefetched_plan = self.executor.submit(prepare_survey_snapshot, clone, now,
                                                   clone.command.copy(), None, None)
        self.prefetch_submitted += 1
        self.prefetch_reason = 'planning_next_stop_to_stop_chunk'

    def flower_event(self, confirmed):
        pass  # Surveying never activates the sprayer/buzzer.

    def _choose_goal(self, current):
        if self.state == 'RETURN':
            self.goal = self._return_goal(current)
            return self.grid.safe(self.goal)
        if self.state != 'SURVEY':
            return super()._choose_goal(current)
        self.goal = self.waypoints[self.waypoint_index].copy()
        delta = self.goal-np.asarray(current)
        self.goal_yaw = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta[:2]) > .05 else self.yaw
        self.desired_pitch = 0.
        return self.grid.safe(self.goal)

    def _return_goal(self, current):
        home = np.array([*self.c['home'][:2], self.c['takeoff_altitude']])
        delta = home-np.asarray(current)
        distance = np.linalg.norm(delta)
        # Home may lie outside the rolling local map. Replan bounded legs;
        # this does not certify the unseen remainder or bypass collision checks.
        step = 2.
        return np.asarray(current)+delta*min(1., step/max(distance, 1e-9))

    def _mission_hint(self, current):
        if self.state == 'SURVEY':
            return self.waypoints[self.waypoint_index].copy()
        if self.state == 'RETURN':
            return self._return_goal(current)
        return super()._mission_hint(current)

    def _advance(self):
        if self.state == 'RETURN':
            home = np.array([*self.c['home'][:2], self.c['takeoff_altitude']])
            intermediate = self.goal is not None and np.linalg.norm(self.goal-home) > 1e-6
            super()._advance()
            if intermediate:
                self.state = 'RETURN'
            return
        if self.state == 'TAKEOFF':
            super()._advance()
            self.state = 'SURVEY'
        elif self.state == 'SURVEY':
            # Do not count a survey point until a newly integrated scan arrives
            # AFTER settling there. Packet traffic alone is insufficient.
            self.wait_for_scan = self.survey_now
        else:
            super()._advance()

    def tick(self, now, dt, position, velocity, range_value=None, range_stamp=None):
        self.survey_now = now
        if self.state in ('SURVEY', 'RETURN'):
            self.survey_started = now if self.survey_started is None else self.survey_started
            if now-self.survey_started > self.c.get('survey_max_duration', float('inf')):
                self.fail('survey_duration_limit')
        result = super().tick(now, dt, position, velocity, range_value, range_stamp)
        self._prefetch_next(now)
        return result

    def _prepare_trajectory(self, now, position, range_value=None, range_stamp=None):
        if self.wait_for_scan is not None:
            if self.grid.last_observation_stamp > self.wait_for_scan:
                self.wait_for_scan = None
                super()._advance()  # Reset per-leg budgets/dwell without tree logic.
                self.waypoint_index += 1
                self.batch_end_index, self.pass_arrival = None, None
                if self.waypoint_index >= len(self.waypoints):
                    self.state = 'RETURN'
            return False
        if self.executor is None:
            return super()._prepare_trajectory(now, position, range_value, range_stamp)
        if self.pending_plan is None and self.prefetched_plan is not None:
            index, start = self.prefetch_key
            if self.state == 'SURVEY' and index == self.waypoint_index and np.linalg.norm(start-self.command) < 1e-6:
                if not self.prefetched_plan.done():
                    return False  # Hold at rest; never switch to an unfinished path.
                try:
                    candidate, ready = self.prefetched_plan.result()
                    usable = (ready and not candidate.failure and candidate.state == 'SURVEY'
                              and candidate.trajectory.safe(self.grid))
                except Exception:
                    usable = False
                if usable:
                    self.pending_plan = self.prefetched_plan
                    self.prefetch_used += 1
                    self.prefetch_reason = 'accepted_after_live_map_check_at_rest'
                else:
                    self.prefetch_rejected += 1
                    self.prefetch_reason = 'discarded_not_safe_on_live_map'
                self.prefetched_plan, self.prefetch_key = None, None
            else:
                self.prefetched_plan.cancel()
                # Do not enqueue behind running work. Keep its handle until done.
                if not self.prefetched_plan.done():
                    return False
                self.prefetched_plan, self.prefetch_key = None, None
                self.prefetch_rejected += 1
                self.prefetch_reason = 'discarded_state_or_start_changed'
        if self.pending_plan is None:
            # Plan only at rest, on a detached map/controller snapshot. Holding
            # an unchanged setpoint while planning cannot advance an old route.
            clone = self._planning_clone()
            self.pending_plan = self.executor.submit(prepare_survey_snapshot, clone, now,
                np.asarray(position).copy(), range_value, range_stamp)
            return False
        if not self.pending_plan.done():
            return False
        try:
            clone, ready = self.pending_plan.result()
        except Exception:
            self.pending_plan = None
            self.fail('survey_planner_error')
            return False
        self.pending_plan = None
        self.planner_stages_ms = clone.planner_stages_ms
        self.planner_job_id = clone.planner_submitted_ns
        self.planner_stages_ms['submit_to_result'] = (time.monotonic_ns()-clone.planner_submitted_ns)/1e6
        self.planner_wall_max_ms = clone.planner_wall_max_ms
        if clone.failure:
            self.fail(clone.failure)
            return False
        # Reject a route invalidated during planning. Actual command/braking
        # checks also run on the current live grid before every setpoint.
        if ready and not clone.trajectory.safe(self.grid):
            return False
        # Never replace live health, map, command derivatives, or worker handles.
        for key in ('trajectory', 'goal', 'goal_yaw', 'desired_pitch', 'visible_view',
                    'state', 'elapsed', 'blocked_since', 'paths', 'replans', 'dwell',
                    'confirmations', 'buzzer_remaining',
                    'exploration_resume', 'exploration_phase', 'exploration_started',
                    'exploration_arrival_cloud', 'exploration_visited', 'exploration_points',
                    'exploration_views', 'exploration_gain', 'last_exploration_search'):
            setattr(self, key, getattr(clone, key))
        for key in ('batch_end_index', 'batch_knots', 'phase', 'adaptive_reason', 'planner_wall_max_ms'):
            setattr(self, key, getattr(clone, key))
        for key in ('route_wait_started', 'route_wait_stamp', 'route_wait_scans',
                    'route_blockage', 'route_planner_reason'):
            setattr(self, key, getattr(clone, key))
        return ready

    def _plan_route(self, start, goal):
        route = super()._plan_route(start, goal)
        if not self.c.get('continuous_survey', False) or self.state != 'SURVEY' or route is None:
            return route
        self.batch_end_index = self.waypoint_index
        self.batch_knots = {self.waypoint_index: len(route)-1}
        for index in range(self.waypoint_index+1,
                           min(len(self.waypoints), self.waypoint_index+self.c.get('survey_batch_points', 4))):
            candidate = self.waypoints[index]
            if not self.grid.safe(candidate):
                break  # Only measured safe lookahead, never a whole-map truth fixture.
            extension = super()._plan_route(route[-1], candidate)
            if extension is None:
                break
            route = route+extension[1:]
            self.batch_end_index, self.goal = index, candidate.copy()
            self.batch_knots[index] = len(route)-1
        return route

    def _trajectory_from_points(self, points, speed=None):
        self.phase = AdaptivePhase(self.c)
        if (not self.c.get('continuous_survey', False)
                or np.max(np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1), initial=0.) < 1e-8):
            return super()._trajectory_from_points(points, speed)
        for scale in (1., .5, .25, 0.):
            try:
                trajectory = ContinuousTrajectory(points, self.c, scale, speed)
            except ValueError:
                continue  # A bounded failed retiming attempt never escapes into flight.
            if trajectory.safe(self.grid):
                return trajectory
        # No rounding is accepted without a complete curve certificate.
        return super()._trajectory_from_points(points, speed)

    def _sample_next(self, dt, position, velocity):
        if not isinstance(self.trajectory, ContinuousTrajectory):
            return super()._sample_next(dt, position, velocity)
        _, nominal_v, nominal_a, _ = self.trajectory.derivatives(self.elapsed)
        error = float(np.linalg.norm(position-self.command))
        soft, hard = .25*self.c['tracking_limit'], .8*self.c['tracking_limit']
        tracking = np.clip(1-(error-soft)/max(hard-soft, 1e-8), self.phase.minimum, 1.)
        # Occupancy clearance is not a claim about true physical clearance.
        occupied = self.grid.clearance(position)
        index = self.grid.indices(position)
        free = float(self.grid.free_distance[tuple(index)]) if hasattr(self.grid, 'free_distance') and self.grid.inside(index) else occupied
        clearance = min(occupied, free)
        low, high = self.c.get('adaptive_clearance_slow', .05), self.c.get('adaptive_clearance_full', .45)
        space = np.clip((clearance-low)/max(high-low, 1e-8), self.phase.minimum, 1.)
        legacy_space = float(space)
        if self.c.get('adaptive_directional_clearance', False):
            # Check stopping along motion, not just distance to a side boundary.
            # Full body/margin inflation and jerk-limited Brake remain intact.
            space = 0.
            for factor in (1., .75, .5, self.phase.minimum):
                if brake_safe(self.grid, position, nominal_v*factor, self.c,
                              nominal_a*factor**2):
                    space = factor
                    break
            if space == 0.:
                self.fail('no_directional_stopping_corridor')
                return self.command.copy(), self.command_v.copy(), self.command_a.copy(), self.elapsed
        curvature_load = (np.linalg.norm(np.cross(nominal_v, nominal_a))/max(np.linalg.norm(nominal_v), 1e-8))
        turn = min(1., math.sqrt(self.c.get('adaptive_turn_acceleration', .25)/max(curvature_load, 1e-8)))
        target = min(float(tracking), float(space), turn)
        self.adaptive_reason = min(('tracking', 'clearance', 'turn'),
                                   key=lambda reason: dict(tracking=tracking, clearance=space, turn=turn)[reason]) if target < .98 else 'open_space'
        if self.adaptive_reason == 'clearance' and self.c.get('adaptive_directional_clearance', False):
            self.adaptive_reason = 'directional_braking'
        self.speed_diagnostics = dict(occupied_clearance_m=float(occupied),
            inflated_free_clearance_m=float(free), legacy_space_scale=legacy_space,
            braking_scale=float(space), tracking_scale=float(tracking), turn_scale=float(turn),
            target_scale=float(target), nominal_speed_mps=float(np.linalg.norm(nominal_v)),
            policy='directional_braking' if self.c.get('adaptive_directional_clearance', False) else 'nearest_boundary')
        advance = self.phase.step(dt, target)
        next_elapsed = min(self.elapsed+advance, self.trajectory.duration)
        p, v, a, j = self.trajectory.derivatives(next_elapsed)
        r, rd, rdd = self.phase.rate, self.phase.rate_dot, self.phase.rate_ddot
        commanded_v = v*r
        commanded_a = a*r*r+v*rd
        self.command_jerk = j*r**3+3*a*r*rd+v*rdd
        if (np.linalg.norm(commanded_v) > self.c['command_speed']+1e-6
                or np.linalg.norm(commanded_a) > self.c['acceleration']+1e-6
                or np.linalg.norm(self.command_jerk) > self.c['jerk']+1e-6):
            self.fail('adaptive_derivative_limit')
        return p, commanded_v, commanded_a, next_elapsed

    def _on_motion(self, now, position):
        if (not self.c.get('continuous_survey', False) or self.state != 'SURVEY'
                or self.batch_end_index is None or self.waypoint_index >= self.batch_end_index):
            return
        goal = self.waypoints[self.waypoint_index]
        if np.linalg.norm(position-goal) <= self.c.get('survey_pass_radius', .2):
            if self.pass_arrival is None:
                self.pass_arrival = now
            if self.grid.last_observation_stamp > self.pass_arrival:
                self.waypoint_index += 1
                self.continuous_passes += 1
                self.pass_arrival = None
                return
        knot = self.batch_knots.get(self.waypoint_index)
        if knot is not None and knot > 0 and self.elapsed > self.trajectory.ends[knot-1]+1.:
            self.fail('survey_waypoint_or_scan_missed')

    def close(self):
        if self.executor:
            self.executor.shutdown(wait=True, cancel_futures=True)
