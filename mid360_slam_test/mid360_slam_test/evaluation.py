"""Operator-led stage machine. No reference is inferred from SLAM itself."""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import inverse, angle, unpack

STAGES = [
    dict(name='stationary', kind='stationary', duration_s=30.),
    dict(name='forward', kind='translation', reference_xyz_m=[1., 0., 0.]),
    dict(name='backward', kind='translation', reference_xyz_m=[-1., 0., 0.]),
    dict(name='left', kind='translation', reference_xyz_m=[0., 1., 0.]),
    dict(name='right', kind='translation', reference_xyz_m=[0., -1., 0.]),
    dict(name='up', kind='translation', reference_xyz_m=[0., 0., .5]),
    dict(name='down', kind='translation', reference_xyz_m=[0., 0., -.5]),
    dict(name='rotation', kind='rotation', reference_yaw_deg=360.),
    dict(name='loop_closure', kind='loop', reference_xyz_m=[0., 0., 0.], reference_yaw_deg=0.),
]


def finite_vector(value, size):
    a = np.asarray(value, float)
    if a.shape != (size,) or not np.all(np.isfinite(a)):
        raise ValueError('finite vector required')
    return a


class Evaluation:
    def __init__(self, config=None):
        defaults = dict(status_timeout_s=1., settle_window_s=1., settle_position_span_m=.03,
            settle_angle_span_deg=2., minimum_settle_samples=5, loop_wait_s=20.,
            max_position_error_m=-1., max_angle_error_deg=-1., max_stationary_drift_m=-1.,
            max_stationary_orientation_deg=-1., reference_uncertainty_m=-1.,
            reference_uncertainty_deg=-1.)
        self.c = dict(defaults, **(config or {}))
        self.index, self.phase = 0, 'READY'
        self.specs = [dict(s) for s in STAGES]
        self.results = []
        self.latest, self.receipt = None, None
        self.recent = deque(maxlen=400)
        self.active = None
        self.seen_commands = {}
        self.closed = False

    @property
    def spec(self):
        return self.specs[min(self.index, len(self.specs)-1)]

    def receive(self, status, now):
        previous_receipt = self.receipt
        previous_status = self.latest
        self.latest, self.receipt = status, now
        valid = status.get('valid') and status.get('tracking') == 'TRACKING'
        if not valid:
            if self.active and self.phase in ('RUNNING', 'FINISHING', 'WAIT_LOOP'):
                self.active['invalid_reasons'].add(status.get('reason', 'tracking_lost'))
            return
        try:
            local, mapped = unpack(status['local']), unpack(status['mapped'])
            stamp = float(status['stamp'])
            if not math.isfinite(stamp):
                raise ValueError('invalid stamp')
        except (KeyError, ValueError, TypeError):
            if self.active:
                self.active['invalid_reasons'].add('invalid_estimator_payload')
            return
        if previous_status and previous_status.get('valid'):
            if stamp <= previous_status.get('stamp', -math.inf):
                if self.active:
                    self.active['invalid_reasons'].add('unordered_estimator_stamp')
                return
        self.recent.append((now, stamp, local, mapped))
        if self.active and self.phase in ('RUNNING', 'FINISHING', 'WAIT_LOOP'):
            a = self.active
            if previous_receipt is not None and now-previous_receipt > self.c['status_timeout_s']:
                a['invalid_reasons'].add('tracking_receipt_gap')
            if status.get('reset_counter') != a['reset_counter']:
                a['invalid_reasons'].add('estimator_reset_during_stage')
            displacement = inverse(a['start_local'])@local
            translation = float(np.linalg.norm(displacement[:3, 3]))
            orientation = angle(displacement)
            yaw = float(Rotation.from_matrix(displacement[:3, :3]).as_euler('xyz', degrees=True)[2])
            delta = (yaw-a['previous_yaw']+180.) % 360.-180.
            a['accumulated_yaw'] += delta
            a['previous_yaw'] = yaw
            a['max_position_change_m'] = max(a['max_position_change_m'], translation)
            a['max_orientation_change_deg'] = max(a['max_orientation_change_deg'], orientation)
            a['squared_position_sum'] += translation**2
            a['samples'] += 1
            a['end_local'], a['end_map'], a['end_stamp'] = local, mapped, stamp

    def receive_loop(self, event, now):
        if not self.active or self.phase not in ('RUNNING', 'FINISHING', 'WAIT_LOOP'):
            return
        # Sequence/source are recorded; real node events only, not proximity inference.
        if event.get('source') != 'experimental_pose_graph' or not isinstance(event.get('sequence'), int):
            return
        if event.get('stamp', -1) < self.active['start_stamp']:
            return
        a = self.active
        if any(e['sequence'] == event['sequence'] for e in a['loop_events']):
            return
        a['loop_events'].append(dict(event, received_elapsed_s=now-a['started_wall']))

    def fresh(self, now):
        return (self.latest is not None and self.latest.get('valid') and self.receipt is not None
                and -1e-9 <= now-self.receipt <= self.c['status_timeout_s'])

    def settled(self, now, since=None):
        if not self.fresh(now):
            return False
        rows = [r for r in self.recent if now-r[0] <= self.c['settle_window_s']+.15
                and (since is None or r[0] >= since)]
        if len(rows) < self.c['minimum_settle_samples'] or rows[-1][0]-rows[0][0] < self.c['settle_window_s']*.8:
            return False
        p0 = rows[0][2]
        return (max(np.linalg.norm(r[2][:3, 3]-p0[:3, 3]) for r in rows) <= self.c['settle_position_span_m']
            and max(angle(inverse(p0)@r[2]) for r in rows) <= self.c['settle_angle_span_deg'])

    def command(self, command, now):
        identifier = command.get('id')
        if not isinstance(identifier, str) or not identifier or len(identifier) > 128:
            return dict(id=identifier, ok=False, reason='command_id_required')
        if identifier in self.seen_commands:
            return self.seen_commands[identifier]
        try:
            action = command['action']
            if self.closed:
                raise ValueError('session_closed')
            if action == 'reference':
                if self.phase != 'READY':
                    raise ValueError('references_only_before_start')
                if 'reference_xyz_m' in command:
                    self.spec['reference_xyz_m'] = finite_vector(command['reference_xyz_m'], 3).tolist()
                for key in ('reference_yaw_deg', 'duration_s'):
                    if key in command:
                        value = float(command[key])
                        if not math.isfinite(value) or (key == 'duration_s' and value <= 0):
                            raise ValueError('invalid_reference')
                        self.spec[key] = value
            elif action == 'start':
                if self.phase != 'READY':
                    raise ValueError('stage_not_ready')
                if not command.get('reference_confirmed'):
                    raise ValueError('operator_reference_confirmation_required')
                if not self.settled(now):
                    raise ValueError('wait_for_fresh_stationary_tracking')
                status = self.latest
                local, mapped = unpack(status['local']), unpack(status['mapped'])
                self.active = dict(spec=dict(self.spec), start_local=local, start_map=mapped,
                    end_local=local.copy(), end_map=mapped.copy(),
                    start_stamp=status['stamp'], end_stamp=status['stamp'], started_wall=now,
                    reset_counter=status.get('reset_counter'), invalid_reasons=set(),
                    accumulated_yaw=0., previous_yaw=0., max_position_change_m=0.,
                    max_orientation_change_deg=0., squared_position_sum=0., samples=0,
                    loop_events=[], finish_requested=None)
                self.phase = 'RUNNING'
            elif action == 'finish':
                if self.phase != 'RUNNING':
                    raise ValueError('stage_not_running')
                if self.spec['kind'] == 'stationary':
                    raise ValueError('stationary_finishes_after_configured_duration')
                self.active['finish_requested'] = now
                self.phase = 'WAIT_LOOP' if self.spec['kind'] == 'loop' else 'FINISHING'
            elif action == 'next':
                if self.phase != 'REVIEW':
                    raise ValueError('review_result_before_next')
                self.index += 1
                self.active = None
                if self.index >= len(self.specs):
                    self.phase, self.closed = 'COMPLETE', True
                else:
                    self.phase = 'READY'
                    self.recent.clear()
            elif action == 'repeat':
                if self.phase not in ('REVIEW', 'RUNNING', 'FINISHING', 'WAIT_LOOP'):
                    raise ValueError('no_stage_to_repeat')
                if self.phase != 'REVIEW':
                    self.active['invalid_reasons'].add('operator_aborted_for_repeat')
                    self.complete(now)
                self.active, self.phase = None, 'READY'
                self.recent.clear()
            elif action == 'skip':
                if self.phase != 'READY':
                    raise ValueError('skip_only_before_start')
                self.results.append(dict(stage=self.spec['name'], status='NOT TESTED', reason='operator_skipped'))
                self.phase = 'REVIEW'
            elif action == 'quit':
                if self.active and self.phase != 'REVIEW':
                    self.active['invalid_reasons'].add('operator_ended_session')
                    self.complete(now)
                self.closed, self.phase = True, 'COMPLETE'
            else:
                raise ValueError('unknown_action')
            result = dict(id=identifier, ok=True, reason='acknowledged', phase=self.phase)
        except (ValueError, KeyError, TypeError) as exc:
            result = dict(id=identifier, ok=False, reason=str(exc), phase=self.phase)
        # Bounded idempotent command history prevents retransmission starting two stages.
        if len(self.seen_commands) >= 1000:
            self.seen_commands.pop(next(iter(self.seen_commands)))
        self.seen_commands[identifier] = result
        return result

    def tick(self, now):
        if self.active and self.phase in ('RUNNING', 'FINISHING', 'WAIT_LOOP'):
            if not self.fresh(now):
                self.active['invalid_reasons'].add('stale_or_lost_tracking')
                self.complete(now)
                return
            a = self.active
            if self.phase == 'RUNNING' and self.spec['kind'] == 'stationary':
                if now-a['started_wall'] >= self.spec['duration_s']:
                    self.complete(now)
            elif self.phase == 'FINISHING' and self.settled(now, a['finish_requested']):
                self.complete(now)
            elif self.phase == 'WAIT_LOOP':
                # Wait full configured interval so asynchronous graph results can arrive.
                if now-a['finish_requested'] >= self.c['loop_wait_s']:
                    if not self.settled(now, a['finish_requested']):
                        a['invalid_reasons'].add('endpoint_not_settled')
                    self.complete(now)
            elif self.phase == 'FINISHING' and now-a['finish_requested'] > 15.:
                a['invalid_reasons'].add('endpoint_settle_timeout')
                self.complete(now)

    def complete(self, now):
        a = self.active
        if a is None:
            return
        kind = a['spec']['kind']
        delta = inverse(a['start_local'])@a['end_local']
        start_map_corrected = a['start_map']
        accepted_loops = [e for e in a['loop_events'] if e.get('accepted')]
        if accepted_loops:
            # Retrospectively update the stage-start anchor with the same graph
            # solution as the endpoint. Never compare two different map revisions.
            graph = accepted_loops[-1].get('graph_solution', [])
            before = [f for f in graph if f['stamp'] <= a['start_stamp']]
            if before:
                anchor = max(before,key=lambda f:f['stamp'])
                start_map_corrected = unpack(anchor['mapped'])@inverse(unpack(anchor['local']))@a['start_local']
        mapped_delta = inverse(start_map_corrected)@a['end_map']
        result = dict(stage=a['spec']['name'], kind=kind, reference=a['spec'],
            source_start_stamp=a['start_stamp'], source_end_stamp=a['end_stamp'],
            duration_wall_s=now-a['started_wall'], samples=a['samples'],
            invalid_reasons=sorted(a['invalid_reasons']),
            displacement_start_sensor_axes_m=delta[:3, 3].tolist(),
            displacement_m=float(np.linalg.norm(delta[:3, 3])), orientation_change_deg=angle(delta),
            accumulated_yaw_deg=a['accumulated_yaw'],
            stationary_max_position_change_m=a['max_position_change_m'],
            stationary_max_orientation_change_deg=a['max_orientation_change_deg'],
            stationary_rms_position_change_m=math.sqrt(a['squared_position_sum']/max(1,a['samples'])),
            mapped_endpoint_error_m=float(np.linalg.norm(mapped_delta[:3, 3])),
            mapped_endpoint_angle_deg=angle(mapped_delta), loop_events=a['loop_events'],
            reference_uncertainty_m=self.c['reference_uncertainty_m'] if self.c['reference_uncertainty_m'] >= 0 else None,
            reference_uncertainty_deg=self.c['reference_uncertainty_deg'] if self.c['reference_uncertainty_deg'] >= 0 else None,
            continuous_ground_truth_available=False)
        result['stationary_endpoint_drift_m_per_min'] = (result['displacement_m']*60./max(1e-9,now-a['started_wall']))
        result['accuracy_gate_scope'] = 'only_explicitly_configured_gates_listed_below'
        result['map_anchor_policy'] = 'stage_start_revised_with_last_accepted_graph_if_available'
        gates = []
        def gate(name, metric, limit):
            if limit >= 0:
                gates.append(dict(name=name, value=metric, limit=limit, passed=metric <= limit))
        if kind in ('translation', 'loop'):
            reference = np.asarray(a['spec']['reference_xyz_m'])
            error = delta[:3, 3]-reference
            result.update(position_error_xyz_m=error.tolist(), position_error_m=float(np.linalg.norm(error)),
                          distance_error_m=float(np.linalg.norm(delta[:3, 3])-np.linalg.norm(reference)))
            gate('position_error_m', result['mapped_endpoint_error_m'] if kind == 'loop'
                 else result['position_error_m'], self.c['max_position_error_m'])
            if kind == 'translation':
                result['orientation_error_deg'] = angle(delta)
                gate('orientation_error_deg', result['orientation_error_deg'], self.c['max_angle_error_deg'])
        if kind == 'stationary':
            gate('max_drift_m', a['max_position_change_m'], self.c['max_stationary_drift_m'])
            gate('max_orientation_deg', a['max_orientation_change_deg'], self.c['max_stationary_orientation_deg'])
        if kind == 'rotation':
            result['yaw_error_deg'] = a['accumulated_yaw']-a['spec']['reference_yaw_deg']
            gate('yaw_error_deg', abs(result['yaw_error_deg']), self.c['max_angle_error_deg'])
        if kind == 'loop':
            accepted = [e for e in a['loop_events'] if e.get('accepted')]
            result['loop_detection_status'] = 'DETECTED' if accepted else 'NOT_DETECTED'
            result['loop_detection_delay_from_return_s'] = (min(e['received_elapsed_s'] for e in accepted)
                -(a['finish_requested']-a['started_wall'])) if accepted else None
            gate('mapped_endpoint_angle_deg', result['mapped_endpoint_angle_deg'], self.c['max_angle_error_deg'])
            result['geometric_acceptance_is_not_independent_correctness'] = True
            if not accepted:
                a['invalid_reasons'].add('no_accepted_loop_within_test')
                result['invalid_reasons'] = sorted(a['invalid_reasons'])
        result['gates'] = gates
        result['status'] = ('FAIL' if a['invalid_reasons'] or any(not g['passed'] for g in gates) else
                            'PASS' if gates and a['samples'] else 'NOT TESTED')
        result['reason'] = ('invalid_or_gate_failed' if result['status'] == 'FAIL' else
                            'configured_gates_met' if result['status'] == 'PASS' else 'acceptance_limits_not_configured')
        self.results.append(result)
        self.phase = 'REVIEW'

    def view(self, now):
        a = self.active
        return dict(phase=self.phase, stage=self.spec['name'] if not self.closed else 'session_complete',
            index=self.index, reference=self.spec, tracking_fresh=bool(self.fresh(now)),
            can_start=bool(self.phase == 'READY' and self.settled(now)),
            elapsed_s=now-a['started_wall'] if a else None,
            wait_remaining_s=max(0.,self.c['loop_wait_s']-(now-a['finish_requested']))
                if a and self.phase=='WAIT_LOOP' else None,
            live_displacement_m=float(np.linalg.norm(a['end_local'][:3, 3]-a['start_local'][:3, 3])) if a else None,
            accumulated_yaw_deg=a['accumulated_yaw'] if a else None,
            invalid_reasons=sorted(a['invalid_reasons']) if a else [],
            results=self.results, closed=self.closed,
            note='stage_reference_reset_only_estimator_and_map_continue')
