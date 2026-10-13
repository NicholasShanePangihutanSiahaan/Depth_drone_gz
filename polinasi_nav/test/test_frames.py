import numpy as np
import math
from scipy.spatial.transform import Rotation
from polinasi_nav.frames import FCULocalAlignment


def test_pose_pairing_interpolates_measurement_time_and_yaw_wrap_without_extrapolation():
    from polinasi_nav.frames import pose_at
    samples=[(1.,np.array([0.,0.,0.]),math.radians(179)),
             (1.1,np.array([.1,0.,0.]),math.radians(-179))]
    p,yaw=pose_at(samples,1.05)
    np.testing.assert_allclose(p,[.05,0,0])
    assert abs(abs(yaw)-math.pi)<1e-9
    assert pose_at(samples,.9) is None and pose_at(samples,1.11) is None


def test_fcu_height_reset_is_numerically_transformed_not_relabelled():
    alignment = FCULocalAlignment([0., 0., .2], 0., [0., 0., 0.], 0.)
    np.testing.assert_allclose(alignment.to_fcu([1., 2., 2.]), [1., 2., 1.8])
    np.testing.assert_allclose(alignment.to_map([1., 2., 1.8]), [1., 2., 2.])
    np.testing.assert_allclose(alignment.fcu_origin_in_map, [0., 0., .2])


def test_rotation_translation_and_tf_inverse_consistent():
    alignment = FCULocalAlignment([4., 5., .2], .2, [-2., 3., 0.], .3)
    point = np.array([6., 8., 2.])
    fcu = alignment.to_fcu(point)
    np.testing.assert_allclose(alignment.to_map(fcu), point)
    np.testing.assert_allclose(alignment.rotation.T@fcu+alignment.fcu_origin_in_map, point)
    np.testing.assert_allclose(alignment.rotation, Rotation.from_euler('z', .1).as_matrix())


def test_position_time_matching_removes_motion_not_real_frame_error():
    from polinasi_nav.frames import position_at
    samples = [(1., np.array([0., 0., 2.]), np.array([.3, 0., 0.])),
               (1.1, np.array([.03, 0., 2.]), np.array([.3, 0., 0.]))]
    matched = position_at(samples, 1.05)
    np.testing.assert_allclose(matched, [.015, 0., 2.])
    assert np.linalg.norm(samples[-1][1]-matched) > .01
    assert np.linalg.norm((matched+[.02, 0., 0.])-matched) > .01
    np.testing.assert_allclose(position_at(samples, 1.12), [.036, 0., 2.])
    assert position_at(samples, .9) is None
    assert position_at(samples, 1.2) is None
    assert position_at([], 1.) is None
