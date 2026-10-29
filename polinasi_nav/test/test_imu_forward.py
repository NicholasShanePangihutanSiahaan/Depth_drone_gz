import numpy as np
from scipy.spatial.transform import Rotation
from polinasi_nav.imu_forward import ForwardImuOdometry


def correct(f,t,p=(0,0,0),v=(0,0,0)):
    return f.correct(t,p,np.eye(3),v,np.eye(6)*.005,[0,0,0],9.8)


def test_forward_constant_acceleration_uses_real_imu_stamps_and_grows_uncertainty():
    f=ForwardImuOdometry();assert f.snapshot() is None;assert correct(f,1.)
    for i in range(1,21):assert f.push(1.+i*.005,[0,0,0],[1.,0,9.8])
    s=f.snapshot();np.testing.assert_allclose(s['position'],[.005,0,0],atol=1e-12)
    np.testing.assert_allclose(s['velocity'],[.1,0,0],atol=1e-12)
    assert s['stamp']==1.1 and s['correction_stamp']==1.
    assert np.all(np.diag(s['covariance'])>=.005)


def test_delayed_lidar_correction_replays_only_future_imu_and_does_not_double_integrate():
    f=ForwardImuOdometry();correct(f,1.)
    for i in range(1,41):f.push(1.+i*.005,[0,0,0],[1.,0,9.8])
    assert correct(f,1.1,p=(2,0,0),v=(.1,0,0))
    s=f.snapshot();assert s['stamp']==1.2
    np.testing.assert_allclose(s['position'],[2.015,0,0],atol=1e-12)
    np.testing.assert_allclose(s['velocity'],[.2,0,0],atol=1e-12)
    assert not correct(f,1.1) and not f.push(1.2,[0,0,0],[0,0,9.8])
    np.testing.assert_array_equal(f.snapshot()['position'],s['position'])


def test_gap_invalidates_forward_output_and_fresh_lidar_can_reinitialise_it():
    f=ForwardImuOdometry();correct(f,1.)
    f.push(1.2,[0,0,0],[0,0,9.8]);assert f.snapshot() is None
    assert f.reason=='imu_prediction_gap'
    assert correct(f,1.2) and f.snapshot()['stamp']==1.2
    assert not f.push(1.21,[float('nan'),0,0],[0,0,9.8])
    assert f.last_input==1.2


def test_body_rotation_and_gravity_are_not_treated_as_world_acceleration():
    f=ForwardImuOdometry();correct(f,1.)
    for i in range(1,21):f.push(1.+i*.005,[0,0,1.],[0,0,9.8])
    s=f.snapshot();np.testing.assert_allclose(s['position'],0.,atol=1e-12)
    np.testing.assert_allclose(s['rotation'],Rotation.from_euler('z',.1).as_matrix(),atol=1e-12)


def test_common_alignment_uses_latest_overlapping_time_instead_of_extrapolation():
    from polinasi_nav.frames import common_pose_pair
    a=[(1.,np.array([0.,0.,0.]),0.),(1.1,np.array([1.,0.,0.]),.1)]
    b=[(1.,np.array([2.,0.,0.]),0.),(1.05,np.array([2.5,0.,0.]),.05)]
    t,first,second=common_pose_pair(a,b)
    assert t==1.05
    np.testing.assert_allclose(first[0],[.5,0,0]);np.testing.assert_allclose(second[0],[2.5,0,0])
    assert abs(first[1]-.05)<1e-12


def test_common_alignment_rejects_nonoverlap_and_large_interpolation_gaps():
    from polinasi_nav.frames import common_pose_pair
    point=np.zeros(3)
    assert common_pose_pair([(1.,point,0.)],[(2.,point,0.)]) is None
    assert common_pose_pair([(1.,point,0.),(2.,point,0.)],[(1.5,point,0.)]) is None
    assert common_pose_pair([],[(1.,point,0.)]) is None
