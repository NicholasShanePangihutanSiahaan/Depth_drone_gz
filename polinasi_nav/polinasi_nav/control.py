"""Shared mission, replanning and final trajectory safety checks."""
import math
import time
import numpy as np
from .inspection import orbit, view_candidates, view_quality
from .exploration import choose_view
from .planning import Brake, Health, OrbitTrajectory, Trajectory, brake_safe, plan


class MissionController:
    def __init__(self, config, grid, sensor_mode=False):
        self.c, self.grid, self.sensor_mode = config, grid, sensor_mode
        self.health = Health()
        self.trajectory = None
        self.elapsed = 0.
        self.goal = None
        self.goal_yaw = 0.
        self.yaw = 0.
        self.pitch = 0.
        self.desired_pitch = 0.
        self.camera_feedback_ok = True  # standalone simulated mount is ideal
        self.orbit_running = False
        self.command = np.asarray(config['home'], float).copy()
        self.command_v = np.zeros(3)
        self.command_a = np.zeros(3)
        self.brake = None
        self.state = 'TAKEOFF'
        self.airborne_state = 'INSPECT'
        self.exploration_states = ('INSPECT', 'RETURN')
        self.failure = ''
        self.tree_index = 0
        self.view_index = 0
        self.dwell = 0.
        self.confirmations = 0
        self.buzzer_remaining = 0.
        self.buzzer_events = 0
        self.tree_buzzed = False
        self.blocked_since = None
        self.replans = 0
        self.paths = []
        self.nominal = []
        self.visible_view = False
        self.land_requested = False
        self.resume_state = None
        self.failure_history = []
        self.checked_map_version = -1
        self.exploration_resume = None
        self.exploration_phase = ''
        self.exploration_started = None
        self.exploration_arrival_cloud = None
        self.exploration_visited = []
        self.exploration_points = []
        self.exploration_views = 0
        self.exploration_gain = 0
        self.last_exploration_search = -math.inf
        self.route_wait_started = None
        self.route_wait_stamp = -math.inf
        self.route_wait_scans = 0
        self.route_blockage = {}
        self.route_planner_reason = ''

    def flower_event(self, confirmed):
        # An event only counts while settled at an inspection view with visibility.
        if self.state == 'INSPECT' and self.dwell > 0 and self.visible_view and confirmed:
            self.confirmations += 1

    def fail(self, reason):
        if not self.failure:
            self.buzzer_remaining = 0.
            if self.state == 'EXPLORE':
                self.exploration_phase = 'BRAKE'
            self.resume_state = self.state
            self.failure_history.append(reason)
            self.failure, self.state = reason, 'BRAKE'
            self.brake = Brake(self.command, self.command_v, self.command_a, self.c)
            self.trajectory = None

    def _braking(self, dt):
        if self.brake is not None:
            if not self.brake.path_safe(self.grid, actual=True):
                self.failure = 'emergency_unavoidable:'+self.failure
                self.brake = None
                self.command_v, self.command_a = np.zeros(3), np.zeros(3)
                return self.command.copy(), np.zeros(3)
            candidate = self.brake.step(dt)
            if not self.grid.line_safe(self.command, candidate):
                self.failure = 'emergency_unavoidable:'+self.failure
                self.brake = None
                self.command_v, self.command_a = np.zeros(3), np.zeros(3)
            else:
                self.command = candidate
                self.command_v, self.command_a = self.brake.v.copy(), self.brake.a.copy()
                if self.brake.finished:
                    self.state = 'HOLD_ABORT'
                    if self.exploration_phase == 'BRAKE':
                        self.exploration_phase = 'HOLD'
                    self.brake = None
                    if self.failure in ('route_changed', 'terrain_replan_required'):
                        self.failure = ''
                        self.state = self.resume_state
                        if self.state == 'EXPLORE':
                            self._leave_exploration()
                        self.goal = None
        return self.command.copy(), self.command_v.copy()

    def _choose_goal(self, current):
        if self.state == 'TAKEOFF':
            self.goal = np.array([*self.c['home'][:2], self.c['takeoff_altitude']])
            return True
        if self.state == 'RETURN':
            self.goal = np.array([*self.c['home'][:2], self.c['takeoff_altitude']])
            return True
        if self.state == 'DESCEND':
            self.goal = np.asarray(self.c['home'], float)
            return True
        if self.state == 'INSPECT':
            tree = self.c['trees'][self.tree_index]
            nominal = orbit(tree, self.c)
            self.nominal = nominal
            if self.c['inspection_mode'] == 'orbit':
                self.goal = nominal[self.view_index]
                self.goal_yaw = math.atan2(tree[1]-self.goal[1], tree[0]-self.goal[0])
                self.desired_pitch, self.visible_view = view_quality(self.goal, tree, self.goal_yaw, self.grid, self.c)
                return self.grid.safe(self.goal)
            angle = math.pi+2*math.pi*self.view_index/self.c['viewpoint_count']
            # A high score alone is insufficient: candidate must be reachable.
            for _, p, yaw, pitch, visible in view_candidates(tree, angle, current, self.grid, self.c):
                candidate = plan(self.grid, current, p)
                if candidate is not None:
                    self.goal, self.goal_yaw, self.desired_pitch, self.visible_view = p, yaw, pitch, visible
                    return True
        return False

    def _mission_hint(self, current):
        if self.goal is not None:
            return self.goal.copy()
        if self.state == 'INSPECT':
            return orbit(self.c['trees'][self.tree_index], self.c)[self.view_index]
        return np.array([*self.c['home'][:2], self.c['takeoff_altitude']])

    def _begin_exploration(self, now, current, relevant_unknown=None):
        # Airborne only: no exploratory arming/takeoff/landing exceptions.
        if not self.c['exploration_enabled'] or self.state not in self.exploration_states:
            return None
        if now-self.last_exploration_search < self.c['exploration_retry_seconds']:
            return None
        self.last_exploration_search = now
        if (len(self.exploration_visited) >= self.c['exploration_max_views'] or
                (self.exploration_started is not None and
                 now-self.exploration_started > self.c['exploration_timeout'])):
            self.fail('exploration_exhausted')
            return None
        view = choose_view(self.grid, current, self._mission_hint(current), self.exploration_visited,
                           self.yaw, relevant_unknown=relevant_unknown)
        if view is None:
            return None
        self.exploration_started = now if self.exploration_started is None else self.exploration_started
        self.exploration_resume, self.state = self.state, 'EXPLORE'
        self.exploration_phase = 'MOVE'
        self.exploration_arrival_cloud = None
        self.exploration_gain = view.gain
        self.route_blockage.update(selected_viewpoint=view.point.tolist(), predicted_relevant_gain=view.gain)
        self.exploration_visited.append(view.point.copy())
        self.exploration_points.append(view.point.copy())
        self.exploration_views += 1
        self.goal = view.point.copy()
        self.dwell, self.confirmations, self.buzzer_remaining = 0., 0, 0.
        self.visible_view = False
        return view.path

    def _guarded_exploration(self, now, current):
        blockers = self.grid.corridor_blockers(current, self._mission_hint(current))
        unknown, occupied = blockers['unknown'], blockers['occupied']
        self.route_blockage = dict(reason='waiting_for_map', unknown_count=len(unknown),
            occupied_count=len(occupied), outside_window=blockers['outside'],
            planner_reason=self.route_planner_reason,
            unknown_samples=self.grid.centers(unknown[:16]).tolist(),
            occupied_samples=self.grid.centers(occupied[:16]).tolist(),
            frame='map', observation_stamp=float(self.grid.last_observation_stamp))
        if self.route_wait_started is None:
            self.route_wait_started = now
            self.route_wait_stamp = self.grid.last_observation_stamp
            self.route_wait_scans = 0
        elif self.grid.last_observation_stamp > self.route_wait_stamp:
            self.route_wait_scans += 1
            self.route_wait_stamp = self.grid.last_observation_stamp
        self.route_blockage['new_scan_count'] = self.route_wait_scans
        if (now-self.route_wait_started < self.c.get('exploration_map_wait_seconds', .8)
                or self.route_wait_scans < self.c.get('exploration_map_wait_scans', 2)):
            return None
        if blockers['outside']:
            self.route_blockage['reason'] = 'goal_outside_local_window'
            return None
        if self.route_planner_reason == 'search_limit':
            self.route_blockage['reason'] = 'planner_search_limit'
            return None
        if not len(unknown):
            self.route_blockage['reason'] = 'occupied_blocked' if len(occupied) else 'planner_no_route'
            return None
        self.route_blockage['reason'] = 'route_unknown'
        points = self._begin_exploration(now, current, relevant_unknown=unknown)
        if points is None:
            self.route_blockage['reason'] = 'no_reachable_informative_view'
        return points

    def _leave_exploration(self):
        self.state = self.exploration_resume
        self.exploration_phase = ''
        self.trajectory, self.goal = None, None
        self.exploration_arrival_cloud = None
        self.dwell, self.confirmations = 0., 0
        self.blocked_since = None

    def _advance(self):
        # Budgets/visited positions apply to one inspection or return leg.
        self.exploration_started = None
        self.exploration_visited = []
        self.last_exploration_search = -math.inf
        self.goal = None
        self.confirmations = 0
        self.dwell = 0.
        if self.state == 'TAKEOFF':
            self.state = 'INSPECT'
        elif self.state == 'INSPECT':
            if self.orbit_running:
                self.view_index = self.c['viewpoint_count']
                self.orbit_running = False
            self.view_index += 1
            count = self.c['viewpoint_count'] + (self.c['inspection_mode'] == 'orbit')
            if self.view_index >= count:
                self.tree_index += 1
                self.view_index, self.tree_buzzed = 0, False
                if self.tree_index >= len(self.c['trees']):
                    self.state = 'RETURN'
        elif self.state == 'RETURN':
            self.state = 'DESCEND'
        elif self.state == 'DESCEND':
            self.state, self.land_requested = 'LAND', True

    def tick(self, now, dt, position, velocity, range_value=None, range_stamp=None):
        position, velocity = np.asarray(position), np.asarray(velocity)
        dt = float(np.clip(dt, 0.001, 0.1))
        self.pitch += float(np.clip(self.desired_pitch-self.pitch, -0.5*dt, 0.5*dt))
        self.buzzer_remaining = max(0., self.buzzer_remaining-dt)
        reason = self.health.reason(now, self.c, self.sensor_mode)
        if reason:
            self.fail(reason)
        if self.failure:
            return self._braking(dt)
        if self.state in ('LAND', 'COMPLETE'):
            return self.command.copy(), np.zeros(3)
        if (self.exploration_started is not None
                and now-self.exploration_started > self.c['exploration_timeout']):
            self.fail('exploration_timeout')
            return self._braking(dt)
        if np.linalg.norm(velocity) > self.c['speed']+0.05:
            self.fail('measured_overspeed')
            return self._braking(dt)
        if not self.grid.safe(position, actual=True) or not brake_safe(self.grid, position, velocity, self.c, self.command_a, actual=True):
            self.fail('insufficient_stopping_clearance')
            return self._braking(dt)
        if np.linalg.norm(position-self.command) > self.c['tracking_limit']:
            self.fail('tracking_error')
            return self._braking(dt)
        if getattr(self, 'mpc_waiting_for_fresh', False):
            # Safety gates above still run; nominal trajectory/planner time is
            # frozen only after the MPC recovery brake has completed.
            return self.command.copy(), self.command_v.copy()
        if self.trajectory is not None and self.checked_map_version != self.grid.version and not self.trajectory.safe(self.grid):
            # Brake FIRST; replanning is allowed only after the vehicle settles.
            self.fail('route_changed')
            return self._braking(dt)
        self.checked_map_version = self.grid.version
        if self.trajectory is None:
            if np.linalg.norm(velocity) > 0.08:
                return self.command.copy(), np.zeros(3)
            if not self._prepare_trajectory(now, position, range_value, range_stamp):
                if self.failure:
                    return self._braking(dt)
                return self.command.copy(), np.zeros(3)
        candidate, v, a, next_elapsed = self._sample_next(dt, position, velocity)
        if self.failure:
            return self._braking(dt)
        # Optional range-based terrain correction is BEFORE the final map gate.
        if self.c['terrain_following'] and self.state == 'INSPECT':
            if range_stamp is None or range_value is None or not 0 <= now-range_stamp < self.c['range_timeout']:
                self.fail('stale_rangefinder')
                return self._braking(dt)
            correction = np.clip(self.c['terrain_agl']-range_value, -self.c['terrain_max_correction'], self.c['terrain_max_correction'])
            # Disallow unchecked changes to a timed trajectory. Replan at rest.
            if abs(correction) > 0.05:
                if np.linalg.norm(self.command_v) > 0.02:
                    self.fail('terrain_replan_required')
                    return self._braking(dt)
                self.goal = self.goal.copy()
                self.goal[2] += correction
                self.trajectory = None
                return self.command.copy(), np.zeros(3)
        if (not self.grid.line_safe(self.command, candidate)
                or not brake_safe(self.grid, candidate, v, self.c, a)):
            self.fail('command_or_braking_collision')
            return self._braking(dt)
        self.command, self.command_v, self.command_a = candidate, v, a
        self.elapsed = next_elapsed
        # Yaw points at the tree along detours too. Bound mount/body rotation.
        yaw_target = self.goal_yaw
        if self.state == 'INSPECT':
            tree = self.c['trees'][self.tree_index]
            yaw_target = math.atan2(tree[1]-candidate[1], tree[0]-candidate[0])
        difference = math.atan2(math.sin(yaw_target-self.yaw), math.cos(yaw_target-self.yaw))
        self.yaw += float(np.clip(difference, -self.c['yaw_rate']*dt, self.c['yaw_rate']*dt))
        self._on_motion(now, position)
        if self.failure:
            return self._braking(dt)
        if self.elapsed >= self.trajectory.duration and np.linalg.norm(position-self.goal) < 0.12 and np.linalg.norm(velocity) < 0.08:
            self.dwell += dt
            if self.state == 'EXPLORE':
                self.exploration_phase = 'OBSERVE'
                if self.exploration_arrival_cloud is None:
                    self.exploration_arrival_cloud = now
                # A healthy old cloud is not proof of a new observation here.
                if (self.dwell >= self.c['exploration_observe_seconds']
                        and self.grid.last_observation_stamp > self.exploration_arrival_cloud):
                    self._leave_exploration()
                return self.command.copy(), self.command_v.copy()
            yaw_settled = abs(difference) < 0.1
            pitch_settled = abs(self.pitch-self.desired_pitch) < 0.1 and self.camera_feedback_ok
            if self.c['mock_flower_events'] and yaw_settled and pitch_settled:
                self.flower_event(True)
            if self.confirmations >= self.c['mock_confirmations'] and not self.tree_buzzed:
                self.buzzer_remaining = self.c['buzzer_duration']
                self.buzzer_events += 1
                self.tree_buzzed = True
            dwell = self.c['inspection_dwell'] if self.c['inspection_mode'] == 'viewpoints' else max(0.2, self.c['mock_confirmations']*dt)
            if self.dwell >= dwell and self.buzzer_remaining <= 0:
                self.trajectory = None
                self._advance()
        return self.command.copy(), self.command_v.copy()

    def _prepare_trajectory(self, now, position, range_value=None, range_stamp=None):
        """Shared route creation/certification; mapping surveys run it off-thread."""
        self.planner_stages_ms = {}
        stage_started = time.perf_counter()
        if self.goal is None and not self._choose_goal(position):
            self.route_planner_reason = 'goal_not_safe'
            points = None
        else:
            if self.c['terrain_following'] and self.state == 'INSPECT':
                if range_stamp is None or range_value is None or not 0 <= now-range_stamp < self.c['range_timeout']:
                    self.fail('stale_rangefinder')
                    return False
                nominal_z = self.goal[2]
                terrain_z = position[2]-range_value+self.c['terrain_agl']
                self.goal = self.goal.copy()
                self.goal[2] = float(np.clip(terrain_z, nominal_z-self.c['terrain_max_correction'], nominal_z+self.c['terrain_max_correction']))
            points = self._plan_route(self.command, self.goal)
        self.planner_stages_ms['goal_and_route_search'] = (time.perf_counter()-stage_started)*1000.
        if points is None:
            stage_started = time.perf_counter()
            self.blocked_since = now if self.blocked_since is None else self.blocked_since
            if self.c.get('exploration_route_guard', False):
                points = self._guarded_exploration(now, self.command)
            else:
                points = self._begin_exploration(now, self.command)
            self.planner_stages_ms['exploration_search'] = (time.perf_counter()-stage_started)*1000.
            if self.failure:
                return False
        if points is None:
            self.blocked_since = now if self.blocked_since is None else self.blocked_since
            if now-self.blocked_since > self.c['blocked_timeout']:
                self.fail('blocked_route')
            return False
        self.blocked_since = None
        self.route_wait_started = None
        self.route_wait_scans = 0
        if self.state != 'EXPLORE':
            self.exploration_started = None
            self.route_blockage = dict(reason='route_found', planner_reason=self.route_planner_reason)
        speed = self.c['exploration_speed'] if self.state == 'EXPLORE' else (self.c['orbit_speed'] if self.state == 'INSPECT' else None)
        stage_started = time.perf_counter()
        self.trajectory = self._trajectory_from_points(points, speed)
        self.planner_stages_ms['trajectory_construction'] = (time.perf_counter()-stage_started)*1000.
        stage_started = time.perf_counter()
        if not self.trajectory.safe(self.grid):
            self.planner_stages_ms['smoothed_collision_check'] = (time.perf_counter()-stage_started)*1000.
            self.trajectory = None
            self.fail('unsafe_smoothed_trajectory')
            return False
        if self.state == 'INSPECT' and self.c['inspection_mode'] == 'orbit' and self.view_index == 1:
            circuit = OrbitTrajectory(self.c['trees'][self.tree_index], self.c)
            if circuit.safe(self.grid) and np.linalg.norm(self.command-circuit.sample(0.)[0]) < 0.12:
                self.trajectory = circuit
                self.goal = circuit.sample(circuit.duration)[0]
                self.orbit_running = True
        self.planner_stages_ms['smoothed_collision_check'] = (time.perf_counter()-stage_started)*1000.
        # Certify the complete commanded trajectory, including safe stopping.
        stage_started = time.perf_counter()
        checks = 0
        for _ in range(12):
            certified = True
            for elapsed in np.arange(0., self.trajectory.duration+0.05, 0.05):
                checks += 1
                p, v, a = self.trajectory.sample(elapsed)
                if not brake_safe(self.grid, p, v, self.c, a):
                    certified = False
                    break
            if certified:
                break
            self.trajectory.slow(1.4)
        self.planner_stages_ms['braking_certification'] = (time.perf_counter()-stage_started)*1000.
        self.planner_stages_ms['braking_sample_checks'] = checks
        self.planner_stages_ms['braking_attempts'] = _+1
        if not certified:
            self.trajectory = None
            self.fail('no_safe_braking_trajectory')
            return False
        self.paths.append(self.trajectory.points.copy())
        self.replans += 1
        self.elapsed = 0.
        return True

    def _plan_route(self, start, goal):
        diagnostics = {}
        result = plan(self.grid, start, goal, self.c.get('planner_max_expansions', 50000), diagnostics)
        self.route_planner_reason = diagnostics.get('reason', '')
        return result

    def _trajectory_from_points(self, points, speed=None):
        return Trajectory(points, self.c, speed)

    def _sample_next(self, dt, position, velocity):
        return (*self.trajectory.sample(self.elapsed+dt), self.elapsed+dt)

    def _on_motion(self, now, position):
        pass
