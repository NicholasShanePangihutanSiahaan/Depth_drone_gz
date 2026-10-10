"""Safety and geometry regressions; usable without ROS installation."""
import copy
import math
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from polinasi_nav.benchmark import fixture_map
from polinasi_nav.config import load_config
from polinasi_nav.control import MissionController
from polinasi_nav.frames import ENU_TO_NED, FLU_TO_FRD, body_velocity, transform_pose
from polinasi_nav.inspection import view_candidates
from polinasi_nav.localisation import LidarImuOdometry
from polinasi_nav.mapping import VoxelMap, FREE, UNKNOWN, OCCUPIED
from polinasi_nav.planning import Brake, Health, OrbitTrajectory, Trajectory, plan
from polinasi_nav.simulation import Scene


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.c = load_config()

    def test_occluded_unknown_and_free_expiry(self):
        g = VoxelMap(self.c)
        origin = np.array([0.15, 0.15, 2.25])
        hit = np.array([[3.15, 0.15, 2.25]])
        g.integrate(origin, hit, [True], 1.)
        self.assertEqual(g.state[tuple(g.indices([1.65, .15, 2.25]))], FREE)
        self.assertEqual(g.state[tuple(g.indices(hit[0]))], OCCUPIED)
        self.assertEqual(g.state[tuple(g.indices([4.05, .15, 2.25]))], UNKNOWN)
        g.rebuild(1.+self.c['free_ttl']+1)
        self.assertEqual(g.state[tuple(g.indices([1.65, .15, 2.25]))], UNKNOWN)
        self.assertEqual(g.state[tuple(g.indices(hit[0]))], OCCUPIED)

    def test_invalid_ray_does_not_clear_unknown(self):
        g = VoxelMap(self.c)
        g.integrate([0., 0., 2.], [[np.nan, 0., 2.]], [False], 1.)
        self.assertTrue(np.all(g.state == UNKNOWN))

    def test_ground_and_foliage_are_retained(self):
        g = fixture_map(Scene(self.c, 'leaf_blocked'), self.c)
        self.assertEqual(g.state[tuple(g.indices([0., 0., 0.]))], OCCUPIED)
        self.assertFalse(g.safe([2., -1., 2.]))
        self.assertIsNone(plan(g, [0., 0., 2.], [2., -1., 2.]))

    def test_swept_volume_inflation_and_unknown(self):
        g = VoxelMap(self.c)
        g.state[:] = FREE
        g.seen[:] = 0.
        g.state[tuple(g.indices([3., 0., 2.]))] = OCCUPIED
        g.rebuild(0.)
        self.assertFalse(g.safe([2.5, 0., 2.]))
        self.assertTrue(g.safe([0., 0., 2.]))
        g.state[tuple(g.indices([0., 0., 2.]))] = UNKNOWN
        g.rebuild(0.)
        self.assertFalse(g.safe([0., 0., 2.]))

    def test_shortening_cannot_miss_thin_corner(self):
        self.c['resolution'] = .3  # Preserve the original reported corner fixture.
        g = fixture_map(Scene(self.c, 'leaf_blocked'), self.c)
        # Regression from the return route: fixed-spaced sampling missed a
        # short visit to this blocked voxel at x ~ .925, y ~ .899.
        self.assertFalse(g.line_safe([6.15, 2.55, 3.75], [.45, .75, 2.25]))
        path = plan(g, np.array([6.15, 2.55, 3.75]), np.array([0., 0., 2.]))
        self.assertIsNotNone(path)
        self.assertTrue(all(g.line_safe(a, b) for a, b in zip(path[:-1], path[1:])))

    def test_zero_width_corner_contact_is_checked(self):
        g = VoxelMap(self.c)
        # Isolate traversal from inflation: the diagonal touches the corner of
        # an adjacent forbidden voxel but never visits its interior.
        g.blocked[:] = False
        start = g.centers(g.indices([.15, .15, 2.25]))
        adjacent = start + np.array([0., g.res, 0.])
        end = start + np.array([g.res, g.res, 0.])
        g.blocked[tuple(g.indices(adjacent))] = True
        self.assertFalse(g.line_safe(start, end))

    def test_vertical_detour(self):
        g = fixture_map(Scene(self.c, 'vertical_detour'), self.c)
        path = plan(g, np.array([0., 0., 2.]), np.array([6., -2., 2.]))
        self.assertIsNotNone(path)
        self.assertGreater(max(p[2] for p in path), 3.2)
        self.assertTrue(Trajectory(path, self.c).safe(g))

    def test_fully_blocked_has_no_route(self):
        g = fixture_map(Scene(self.c, 'fully_blocked'), self.c)
        self.assertIsNone(plan(g, np.array([0., 0., 2.]), np.array([6., 0., 2.])))

    def test_quintic_derivative_limits(self):
        tr = Trajectory([[0., 0., 2.], [2., 1., 3.], [3., 2., 3.]], self.c)
        times = np.linspace(0., tr.duration, 2001)
        samples = [tr.sample(t) for t in times]
        v = np.array([x[1] for x in samples]); a = np.array([x[2] for x in samples])
        jerk = np.diff(a, axis=0)/(times[1]-times[0])
        self.assertLessEqual(np.linalg.norm(v, axis=1).max(), self.c['speed']+1e-5)
        self.assertLessEqual(np.linalg.norm(a, axis=1).max(), self.c['acceleration']+1e-4)
        self.assertLessEqual(np.linalg.norm(jerk, axis=1).max(), self.c['jerk']+1e-3)
        for stamp in tr.ends:
            _, velocity, acceleration = tr.sample(stamp)
            self.assertLess(np.linalg.norm(velocity), 1e-8)
            self.assertLess(np.linalg.norm(acceleration), 1e-8)

    def test_continuous_orbit_is_bounded_and_closed(self):
        tr = OrbitTrajectory(self.c['trees'][0], self.c)
        positions, velocities, accelerations = map(np.array, zip(*[tr.sample(t) for t in np.linspace(0., tr.duration, 1001)]))
        np.testing.assert_allclose(positions[0], positions[-1], atol=1e-9)
        self.assertTrue(np.all(np.linalg.norm(velocities[1:-1], axis=1) > 0))
        self.assertLessEqual(np.linalg.norm(velocities, axis=1).max(), self.c['orbit_speed']+1e-5)
        self.assertLessEqual(np.linalg.norm(accelerations, axis=1).max(), self.c['acceleration'])

    def test_brake_preserves_initial_state_and_limits(self):
        v, a = np.array([.35, .1, 0.]), np.array([.2, .05, 0.])
        stop = Brake([0., 0., 2.], v, a, self.c)
        np.testing.assert_allclose(stop.v, v)
        np.testing.assert_allclose(stop.a, a)
        previous = a.copy()
        for _ in range(int(math.ceil(stop.duration/.01))):
            stop.step(.01)
            self.assertLessEqual(np.linalg.norm(stop.a), self.c['acceleration']+1e-6)
            self.assertLessEqual(np.linalg.norm(stop.a-previous)/.01, self.c['jerk']+1e-5)
            previous = stop.a.copy()
        self.assertTrue(stop.finished)
        self.assertLess(np.linalg.norm(stop.v), 1e-8)
        self.assertLess(np.linalg.norm(stop.a), 1e-8)

    def test_freshness_uses_measurement_time(self):
        health = Health(1., 1., 1., True)
        self.assertEqual(health.reason(1.1, self.c, True), '')
        self.assertEqual(health.reason(2., self.c), 'stale_localisation')
        health.pose_stamp = 2.
        self.assertEqual(health.reason(2., self.c), 'stale_lidar')
        health.cloud_stamp = 2.
        self.assertEqual(health.reason(2., self.c, True), 'stale_imu')
        health.pose_stamp = 3.
        self.assertEqual(health.reason(2., self.c), 'stale_localisation')

    def test_localisation_failure_latches_and_buzzer_off(self):
        g = fixture_map(Scene(self.c), self.c)
        ctl = MissionController(self.c, g)
        ctl.buzzer_remaining = .3
        ctl.tick(1., .05, np.array(self.c['home']), np.zeros(3))
        self.assertEqual(ctl.failure, 'localisation_loss')
        self.assertEqual(ctl.buzzer_remaining, 0.)
        ctl.health = Health(2., 2., 2., True)
        ctl.tick(2., .05, np.array(self.c['home']), np.zeros(3))
        self.assertEqual(ctl.failure, 'localisation_loss')

    def test_dynamic_route_change_brakes_before_replan(self):
        g = fixture_map(Scene(self.c), self.c)
        ctl = MissionController(self.c, g)
        ctl.command = np.array([0., 0., 2.])
        ctl.state, ctl.goal = 'RETURN', np.array([4., 0., 2.])
        ctl.trajectory = Trajectory([ctl.command, ctl.goal], self.c)
        ctl.health = Health(1., 1., 1., True)
        g.state[tuple(g.indices([2., 0., 2.]))] = OCCUPIED
        g.version += 1
        g.rebuild(1.)
        ctl.tick(1., .05, ctl.command, np.zeros(3))
        self.assertIn('route_changed', ctl.failure_history)
        self.assertEqual(ctl.state, 'BRAKE')
        self.assertEqual(ctl.replans, 0)
        for i in range(20):
            now = 1.05+i*.05
            ctl.health = Health(now, now, now, True)
            ctl.tick(now, .05, ctl.command, np.zeros(3))
        self.assertEqual(ctl.failure, '')
        self.assertGreater(ctl.replans, 0)

    def test_terrain_correction_is_gated(self):
        c = copy.deepcopy(self.c); c['terrain_following'] = True
        g = fixture_map(Scene(c), c)
        ctl = MissionController(c, g)
        ctl.state = 'INSPECT'
        ctl.command = np.array([0., 0., 2.])
        ctl.health = Health(1., 1., 1., True)
        ctl.tick(1., .05, ctl.command, np.zeros(3), range_value=2., range_stamp=0.)
        self.assertEqual(ctl.failure, 'stale_rangefinder')

    def test_viewpoints_offer_high_leaf_clear_views(self):
        g = fixture_map(Scene(self.c, 'leaf_blocked'), self.c)
        candidates = view_candidates(self.c['trees'][0], math.pi, [0., 0., 2.], g, self.c)
        self.assertTrue(candidates)
        self.assertTrue(all(g.safe(v[1]) for v in candidates))
        self.assertTrue(any(v[1][2] > 3. for v in candidates))

    def test_lidar_coverage_mount_and_self_filter(self):
        scene = Scene(self.c)
        origin, world, _, local = scene.scan([0., 0., 2.])
        np.testing.assert_allclose(origin, [0., 0., 2.2])
        elevations = np.degrees(np.arctan2(local[:, 2], np.linalg.norm(local[:, :2], axis=1)))
        self.assertGreaterEqual(elevations.min(), -7.-1e-8)
        self.assertLessEqual(elevations.max(), 52.+1e-8)
        np.testing.assert_allclose(world, local+origin)
        c = copy.deepcopy(self.c); c['lidar_mount'] = [.1, 0., .2, 0., .2, .4]
        _, world, _, local = Scene(c).scan([1., 2., 3.], Rotation.from_euler('z', .5).as_matrix())
        expected = local@Rotation.from_euler('xyz', c['lidar_mount'][3:]).as_matrix().T+c['lidar_mount'][:3]
        expected = expected@Rotation.from_euler('z', .5).as_matrix().T+[1., 2., 3.]
        np.testing.assert_allclose(world, expected, atol=1e-9)

    def test_coordinate_transforms_are_numeric_not_labels(self):
        np.testing.assert_allclose(ENU_TO_NED@[1., 2., 3.], [2., 1., -3.])
        np.testing.assert_allclose(FLU_TO_FRD@[1., 2., 3.], [1., -2., -3.])
        yaw = Rotation.from_euler('z', math.pi/2).as_matrix()
        np.testing.assert_allclose(body_velocity([0., 1., 0.], yaw), [1., 0., 0.], atol=1e-9)
        p, r = transform_pose([1., 0., 0.], np.eye(3), [4., 5., 6.], yaw)
        np.testing.assert_allclose(p, [4., 6., 6.], atol=1e-9)
        np.testing.assert_allclose(r, yaw)

    def test_lio_rejects_planar_and_old_data(self):
        e = LidarImuOdometry(self.c['lidar_mount'])
        for i in range(50):
            e.imu([0., 0., 0.], [0., 0., 9.81], i*.005)
        xy = np.array(np.meshgrid(np.linspace(-4, 4, 20), np.linspace(-4, 4, 20))).reshape(2, -1).T
        points = np.column_stack((xy, np.zeros(len(xy))))
        self.assertFalse(e.scan(points, .245))
        self.assertEqual(e.reason, 'planar_degeneracy')
        e.imu([0., 0., 0.], [0., 0., 9.81], .20)
        self.assertEqual(e.reason, 'imu_timestamp_order')


if __name__ == '__main__':
    unittest.main()
