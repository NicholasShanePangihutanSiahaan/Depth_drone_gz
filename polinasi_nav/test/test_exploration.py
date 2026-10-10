"""Partial-map fixtures: exploration never authorises flight into unknown."""
import unittest
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.control import MissionController
from polinasi_nav.exploration import choose_view, visible_unknown
from polinasi_nav.mapping import FREE, UNKNOWN, OCCUPIED, VoxelMap
from polinasi_nav.planning import Health


def partial_map():
    c = load_config()
    c.update(resolution=0.2, bounds_min=[-2., -3., 0.], bounds_max=[8., 3., 5.],
             drone_dimensions=[0.2, 0.2, 0.2], clearance=0.05,
             tracking_margin=0.05, tracking_limit=0.05, free_ttl=1000.,
             inspection_mode='orbit', trees=[[5., 0., 2.]], home=[0.1, 0.1, 2.1])
    g = VoxelMap(c)
    g.state[:] = FREE
    g.seen[:] = 0.
    ids = np.indices(g.shape).reshape(3, -1).T
    p = g.centers(ids)
    g.state[tuple(ids[p[:, 0] >= 2.].T)] = UNKNOWN
    g.state[:, :, 0] = OCCUPIED
    g.rebuild(0.)
    return c, g


def controller():
    c, g = partial_map()
    ctl = MissionController(c, g)
    ctl.state = 'INSPECT'
    ctl.command = np.asarray(c['home'], float)
    return c, g, ctl


def step(ctl, now, cloud_stamp=None):
    ctl.health = Health(now if cloud_stamp is None else cloud_stamp, now, now, True)
    return ctl.tick(now, .05, ctl.command.copy(), ctl.command_v.copy())


class ExplorationTests(unittest.TestCase):
    def test_view_and_path_are_free_and_map_is_not_modified(self):
        c, g = partial_map()
        before = g.state.copy()
        view = choose_view(g, c['home'], [3., 0., 2.])
        self.assertIsNotNone(view)
        self.assertGreaterEqual(view.gain, c['exploration_min_gain'])
        self.assertTrue(g.safe(view.point))
        self.assertTrue(all(g.line_safe(a, b) for a, b in zip(view.path[:-1], view.path[1:])))
        np.testing.assert_array_equal(g.state, before)

    def test_occupied_wall_occludes_unknown_gain(self):
        c, g = partial_map()
        # A full-height wall lies between sensor and unknown half-space.
        wall = g.indices([1.5, 0., 2.])[0]
        g.state[wall, :, :] = OCCUPIED
        g.rebuild(0.)
        self.assertEqual(len(visible_unknown(g, c['home'])), 0)

    def test_vertical_coverage_and_mount_rotation(self):
        c, g = partial_map()
        g.state[:] = FREE
        ids = np.indices(g.shape).reshape(3, -1).T
        points = g.centers(ids)
        lower = ((points[:, 0] > 1.) & (points[:, 0] < 2.)
                 & (abs(points[:, 1]) < 0.5) & (points[:, 2] < 0.8))
        g.state[tuple(ids[lower].T)] = UNKNOWN
        self.assertEqual(len(visible_unknown(g, [0., 0., 3.])), 0)
        c['lidar_mount'][4] = np.pi/4  # Real extrinsic rotation, not a frame rename.
        self.assertGreater(len(visible_unknown(g, [0., 0., 3.])), 0)

    def test_visited_neighbourhood_is_not_reselected(self):
        c, g = partial_map()
        view = choose_view(g, c['home'], [3., 0., 2.])
        c['exploration_revisit_distance'] = 20.
        self.assertIsNone(choose_view(g, c['home'], [3., 0., 2.], [view.point]))

    def test_known_map_needs_no_exploration(self):
        c, g = partial_map()
        g.state[g.state == UNKNOWN] = FREE
        g.rebuild(0.)
        self.assertIsNone(choose_view(g, c['home'], [3., 0., 2.]))

    def test_mission_pauses_and_resumes_after_new_cloud(self):
        c, g, ctl = controller()
        step(ctl, 0.)
        self.assertEqual(ctl.state, 'EXPLORE')
        self.assertLessEqual(np.linalg.norm(ctl.command_v), c['exploration_speed'])
        for i in range(1, 800):
            now = i*.05
            step(ctl, now)
            if ctl.exploration_phase == 'OBSERVE':
                break
        self.assertEqual(ctl.exploration_phase, 'OBSERVE')
        self.assertEqual(ctl.tree_index, 0)
        self.assertEqual(ctl.view_index, 0)
        self.assertEqual(ctl.buzzer_events, 0)
        # A controlled map-update fixture, not a claim of LiDAR mapping.
        g.state[g.state == UNKNOWN] = FREE
        g.seen[:] = now
        g.version += 1
        g.rebuild(now)
        for i in range(30):
            now += .05
            g.last_observation_stamp = now  # Simulate a mapped scan acquired after arrival.
            step(ctl, now)
            if ctl.state != 'EXPLORE':
                break
        self.assertEqual(ctl.state, 'INSPECT')
        self.assertIsNone(ctl.goal)
        step(ctl, now+.05)
        self.assertEqual(ctl.state, 'INSPECT')
        np.testing.assert_allclose(ctl.goal, [3., 0., 2.], atol=1e-9)

    def test_new_obstacle_invalidates_exploration_trajectory(self):
        c, g, ctl = controller()
        step(ctl, 0.)
        point = ctl.trajectory.sample(ctl.trajectory.duration*.8)[0]
        g.state[tuple(g.indices(point))] = OCCUPIED
        g.version += 1
        g.rebuild(.05)
        step(ctl, .05)
        self.assertEqual(ctl.failure, 'route_changed')
        self.assertIn(ctl.state, ('BRAKE', 'HOLD_ABORT'))

    def test_no_new_measurement_does_not_finish_observation(self):
        c, g, ctl = controller()
        c['cloud_timeout'] = 5.
        step(ctl, 0.)
        now = 0.
        for _ in range(800):
            now += .05
            step(ctl, now)
            if ctl.exploration_phase == 'OBSERVE':
                break
        stamp = ctl.health.cloud_stamp
        for _ in range(30):
            now += .05
            step(ctl, now, stamp)
        self.assertEqual(ctl.state, 'EXPLORE')
        self.assertEqual(ctl.exploration_phase, 'OBSERVE')

    def test_dropout_during_exploration_brakes_and_buzzer_stays_off(self):
        c, g, ctl = controller()
        step(ctl, 0.)
        for i in range(1, 10):
            step(ctl, i*.05)
        step(ctl, 1.5, 0.)
        self.assertEqual(ctl.failure, 'stale_lidar')
        self.assertIn(ctl.state, ('BRAKE', 'HOLD_ABORT'))
        self.assertEqual(ctl.buzzer_remaining, 0.)

    def test_exploration_time_and_visit_budgets(self):
        c, g, ctl = controller()
        step(ctl, 0.)
        c['exploration_timeout'] = .1
        step(ctl, .2)
        self.assertEqual(ctl.failure, 'exploration_timeout')
        c, g, ctl = controller()
        ctl.exploration_visited = [np.zeros(3)]*c['exploration_max_views']
        step(ctl, 0.)
        self.assertEqual(ctl.failure, 'exploration_exhausted')

    def test_disabled_exploration_and_takeoff_do_not_peek(self):
        c, g, ctl = controller()
        c['exploration_enabled'] = False
        step(ctl, 0.)
        self.assertEqual(ctl.exploration_views, 0)
        self.assertEqual(ctl.state, 'INSPECT')
        c['exploration_enabled'] = True
        ctl.state = 'TAKEOFF'
        self.assertIsNone(ctl._begin_exploration(2., ctl.command))


if __name__ == '__main__':
    unittest.main()
