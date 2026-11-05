"""Bounded experimental IMU-seeded ICP and SE(3) pose graph.

Not a tightly coupled LIO/filter. No point deskew, place descriptors, or global
relocalisation. Local odometry never changes when loop optimisation is accepted.
"""
import time
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from .geometry import pose, inverse, transform, angle, pack, downsample

DEFAULTS = dict(voxel_m=.12, max_points=4000, min_points=100, icp_iterations=25,
    correspondence_m=.5, min_overlap=.55, max_rmse_m=.12, min_geometry_ratio=.02,
    max_scan_gap_s=.7, max_speed_mps=1., max_rotation_step_deg=35.,
    keyframe_translation_m=.35, keyframe_rotation_deg=15., max_keyframes=160,
    loop_enabled=True, loop_radius_m=1., loop_min_separation=12, loop_min_travel_m=4.,
    loop_min_seconds=10., loop_cooldown_s=5., loop_overlap=.75, loop_rmse_m=.08,
    loop_max_correction_m=1., loop_max_correction_deg=25.,
    graph_translation_sigma_m=.05, graph_rotation_sigma_deg=3., graph_max_evaluations=50)


def icp(source, target, initial, c):
    if len(source) < c['min_points'] or len(target) < c['min_points']:
        return None, dict(reason='insufficient_points')
    tree, t = cKDTree(target), initial.copy()
    converged = False
    for iteration in range(c['icp_iterations']):
        moved = transform(t, source)
        distances, ids = tree.query(moved)
        keep = distances < c['correspondence_m']
        if keep.sum() < c['min_points'] or keep.mean() < c['min_overlap']:
            return None, dict(reason='poor_overlap', overlap=float(keep.mean()))
        # Trim the worst 10% of accepted correspondences, reducing outlier leverage.
        keep &= distances <= np.percentile(distances[keep], 90)
        a, b = moved[keep], target[ids[keep]]
        ca, cb = a.mean(axis=0), b.mean(axis=0)
        u, singular, vt = np.linalg.svd((a-ca).T@(b-cb))
        if singular[-1]/max(singular[0], 1e-12) < c['min_geometry_ratio']:
            return None, dict(reason='degenerate_geometry')
        r = vt.T@u.T
        if np.linalg.det(r) < 0:
            vt[-1] *= -1
            r = vt.T@u.T
        correction = np.eye(4)
        correction[:3, :3], correction[:3, 3] = r, cb-r@ca
        t = correction@t
        # Reproject numerical products onto SO(3). Transpose-based inverse
        # assumes orthonormality; uncorrected roundoff compounds at keyframes.
        t[:3, :3] = Rotation.from_matrix(t[:3, :3]).as_matrix()
        if np.linalg.norm(correction[:3, 3]) < 1e-4 and angle(correction) < .01:
            converged = True
            break
    distances, _ = tree.query(transform(t, source))
    keep = distances < c['correspondence_m']
    overlap = float(keep.mean())
    rmse = float(np.sqrt(np.mean(distances[keep]**2))) if keep.any() else None
    reverse_distances, _ = cKDTree(transform(t, source)).query(target)
    reverse = float((reverse_distances < c['correspondence_m']).mean())
    ok = converged and rmse is not None and rmse <= c['max_rmse_m'] and overlap >= c['min_overlap']
    return (t if ok else None), dict(reason='ok' if ok else 'icp_not_converged_or_quality',
        converged=converged, rmse_m=rmse, overlap=overlap, reverse_overlap=reverse,
        iterations=iteration+1)


def optimise_graph(poses, edges, c):
    """First pose fixed; constraints measured in keyframe i coordinates."""
    def vector(t):
        return np.r_[t[:3, 3], Rotation.from_matrix(t[:3, :3]).as_rotvec()]

    def matrix(x):
        t = np.eye(4)
        t[:3, 3], t[:3, :3] = x[:3], Rotation.from_rotvec(x[3:]).as_matrix()
        return t

    def decode(x):
        return [poses[0]]+[matrix(row) for row in x.reshape(-1, 6)]

    def residual(x):
        values, out = decode(x), []
        for i, j, measurement in edges:
            error = inverse(measurement)@inverse(values[i])@values[j]
            out.extend(error[:3, 3]/c['graph_translation_sigma_m'])
            out.extend(Rotation.from_matrix(error[:3, :3]).as_rotvec()/np.radians(c['graph_rotation_sigma_deg']))
        return np.asarray(out)

    x = np.concatenate([vector(t) for t in poses[1:]])
    sparsity = lil_matrix((len(edges)*6, len(x)), dtype=int)
    for k, (i, j, _) in enumerate(edges):
        for index in (i, j):
            if index:
                sparsity[6*k:6*k+6, 6*(index-1):6*index] = 1
    before = float(np.linalg.norm(residual(x)))
    fit = least_squares(residual, x, jac_sparsity=sparsity.tocsr(), loss='soft_l1',
                        max_nfev=c['graph_max_evaluations'])
    after = float(np.linalg.norm(residual(fit.x)))
    ok = bool(fit.success and np.all(np.isfinite(fit.x)) and after <= before+1e-6)
    return (decode(fit.x) if ok else None), dict(success=ok, evaluations=int(fit.nfev),
        residual_before=before, residual_after=after, message=str(fit.message))


class SlamEstimator:
    def __init__(self, configuration=None):
        self.c = dict(DEFAULTS, **(configuration or {}))
        for key, value in self.c.items():
            if key != 'loop_enabled' and (not np.isfinite(value) or value <= 0):
                raise ValueError('positive finite estimator setting required: '+key)
        for name in ('max_points', 'min_points', 'icp_iterations', 'max_keyframes',
                     'loop_min_separation', 'graph_max_evaluations'):
            if not isinstance(self.c[name], int):
                raise ValueError(name+' must be integer')
        if self.c['min_points'] > self.c['max_points'] or self.c['max_keyframes'] < 3:
            raise ValueError('invalid point/keyframe budgets')
        if max(self.c['min_overlap'], self.c['loop_overlap'], self.c['min_geometry_ratio']) > 1:
            raise ValueError('ratio > 1')
        self.local_pose = np.eye(4)
        self.correction = np.eye(4)
        self.last_stamp = self.last_imu_rotation = None
        self.keyframes, self.map_poses, self.edges = [], [], []
        self.last_loop_stamp = -np.inf
        self.loop_sequence = 0
        self.state, self.reason = 'INITIALIZING', 'waiting_for_imu_and_cloud'
        self.travel = 0.
        self.velocity = np.zeros(3)
        self.status = {}

    def fail(self, reason):
        self.state, self.reason = 'LOST', reason

    def scan(self, points, stamp, imu_rotation):
        started = time.perf_counter()
        events = []
        if self.state == 'LOST':
            return self.snapshot(stamp, events, started)
        cloud = downsample(points, self.c['voxel_m'], self.c['max_points'])
        if len(cloud) < self.c['min_points']:
            self.fail('insufficient_points')
            return self.snapshot(stamp, events, started)
        if not self.keyframes:
            t = pose()
            t[:3, :3] = imu_rotation
            self.local_pose = t
            self.add_keyframe(cloud, stamp)
            self.last_stamp, self.last_imu_rotation = stamp, imu_rotation.copy()
            self.state, self.reason = 'TRACKING', 'bootstrap_not_accuracy_validation'
            self.status = dict(rmse_m=None, overlap=None, covariance_available=False)
            return self.snapshot(stamp, events, started)
        dt = stamp-self.last_stamp
        if not 0 < dt <= self.c['max_scan_gap_s']:
            self.fail('scan_timestamp_order_or_gap')
            return self.snapshot(stamp, events, started)
        # IMU predicts relative rotation only; translation comes from LiDAR registration.
        predicted = self.local_pose.copy()
        predicted[:3, :3] = self.local_pose[:3, :3]@self.last_imu_rotation.T@imu_rotation
        predicted[:3, 3] += self.velocity*dt
        anchor = self.keyframes[-1]
        guess = inverse(anchor['local'])@predicted
        registered, quality = icp(cloud, anchor['cloud'], guess, self.c)
        self.status = quality
        if registered is None:
            self.fail(quality['reason'])
            return self.snapshot(stamp, events, started)
        candidate = anchor['local']@registered
        candidate[:3, :3] = Rotation.from_matrix(candidate[:3, :3]).as_matrix()
        change = inverse(self.local_pose)@candidate
        distance = float(np.linalg.norm(candidate[:3, 3]-self.local_pose[:3, 3]))
        if distance/dt > self.c['max_speed_mps'] or angle(change) > self.c['max_rotation_step_deg']:
            self.fail('implausible_pose_jump')
            return self.snapshot(stamp, events, started)
        self.velocity = (candidate[:3, 3]-self.local_pose[:3, 3])/dt
        self.travel += distance
        self.local_pose = candidate
        self.last_stamp, self.last_imu_rotation = stamp, imu_rotation.copy()
        relative = inverse(anchor['local'])@candidate
        if (np.linalg.norm(relative[:3, 3]) >= self.c['keyframe_translation_m']
                or angle(relative) >= self.c['keyframe_rotation_deg']):
            if len(self.keyframes) >= self.c['max_keyframes']:
                self.fail('keyframe_budget_reached_stop_and_save')
            else:
                self.add_keyframe(cloud, stamp)
                if self.c['loop_enabled']:
                    event = self.try_loop(stamp)
                    if event:
                        events.append(event)
        self.reason = self.reason if self.state == 'LOST' else 'registered'
        return self.snapshot(stamp, events, started)

    def add_keyframe(self, cloud, stamp):
        index = len(self.keyframes)
        if index:
            self.edges.append((index-1, index, inverse(self.keyframes[-1]['local'])@self.local_pose))
        self.keyframes.append(dict(local=self.local_pose.copy(), cloud=cloud.copy(),
                                  stamp=stamp, travel=self.travel))
        self.map_poses.append(self.correction@self.local_pose)

    def try_loop(self, stamp):
        j = len(self.keyframes)-1
        if stamp-self.last_loop_stamp < self.c['loop_cooldown_s']:
            return None
        candidates = []
        for i in range(max(0, j-self.c['loop_min_separation']+1)):
            old = self.keyframes[i]
            if (stamp-old['stamp'] < self.c['loop_min_seconds'] or
                    self.travel-old['travel'] < self.c['loop_min_travel_m']):
                continue
            distance = np.linalg.norm(self.map_poses[i][:3, 3]-self.map_poses[j][:3, 3])
            if distance <= self.c['loop_radius_m']:
                candidates.append((distance, i))
        if not candidates:
            return None
        _, i = min(candidates)
        self.last_loop_stamp = stamp
        self.loop_sequence += 1
        event = dict(sequence=self.loop_sequence, stamp=float(stamp), source='experimental_pose_graph',
                     from_keyframe=j, to_keyframe=i, accepted=False, state='REJECTED')
        guess = inverse(self.map_poses[i])@self.map_poses[j]
        settings = dict(self.c, min_overlap=self.c['loop_overlap'], max_rmse_m=self.c['loop_rmse_m'])
        registration, quality = icp(self.keyframes[j]['cloud'], self.keyframes[i]['cloud'], guess, settings)
        event['registration'] = quality
        if registration is None or quality.get('reverse_overlap', 0.) < self.c['loop_overlap']:
            event['reason'] = 'loop_registration_failed'
            return event
        correction = inverse(guess)@registration
        if (np.linalg.norm(correction[:3, 3]) > self.c['loop_max_correction_m'] or
                angle(correction) > self.c['loop_max_correction_deg']):
            event['reason'] = 'loop_correction_outside_gate'
            return event
        edges = self.edges+[(i, j, registration)]
        optimised, quality_graph = optimise_graph(self.map_poses, edges, self.c)
        event['optimisation'] = quality_graph
        if optimised is None:
            event['reason'] = 'graph_optimisation_failed'
            return event
        before = self.map_poses[j].copy()
        self.map_poses, self.edges = optimised, edges
        self.correction = optimised[j]@inverse(self.local_pose)
        change = inverse(before)@optimised[j]
        event.update(accepted=True, state='ACCEPTED', reason='registered_and_graph_optimised',
            correction_m=float(np.linalg.norm(change[:3, 3])), correction_deg=angle(change),
            before=pack(before), after=pack(optimised[j]),
            graph_solution=[dict(stamp=f['stamp'], local=pack(f['local']), mapped=pack(t))
                            for f,t in zip(self.keyframes,optimised)],
            note='geometric_acceptance_not_independent_proof_of_correct_place')
        return event

    def snapshot(self, stamp, events, started):
        valid = self.state == 'TRACKING'
        return dict(stamp=float(stamp), tracking=self.state, reason=self.reason, valid=valid,
            local=pack(self.local_pose) if valid else None,
            mapped=pack(self.correction@self.local_pose) if valid else None,
            velocity_mps=self.velocity.tolist() if valid else None,
            travel_m=float(self.travel), keyframes=len(self.keyframes), loops_accepted=len(self.edges)-max(0,len(self.keyframes)-1),
            quality=self.status, loop_events=events, processing_ms=(time.perf_counter()-started)*1000.,
            covariance_available=False, deskewed=False, reset_counter=0,
            backend='experimental_imu_seeded_ICP_SE3_graph_not_tightly_coupled_LIO',
            pose_reference='LiDAR_origin_sensor_to_map', body_extrinsics_applied=False)
