import numpy as np
from polinasi_nav.localisation import LidarImuOdometry


def scene():
    x,y=np.meshgrid(np.linspace(-4,4,40),np.linspace(-4,4,40))
    a,z=np.meshgrid(np.linspace(-4,4,20),np.linspace(2,6,20))
    ground=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    wall_x=np.column_stack([np.full(a.size,5.),a.ravel(),z.ravel()])
    wall_y=np.column_stack([a.ravel(),np.full(a.size,5.),z.ravel()])
    return ground,wall_x,wall_y


def estimator():
    e=LidarImuOdometry([0,0,0,0,0,0],body_imu=True,robust_planes=True)
    for i in range(50):e.imu([0,0,0],[0,0,9.81],i*.005)
    assert e.scan(np.vstack(scene()),.245)
    for j in range(20):e.imu([0,0,0],[0,0,9.81],.25+j*.005)
    return e


def test_robust_planes_reject_outliers_without_relaxing_pose_acceptance():
    e=estimator();clouds=[]
    for axis,cloud in zip([2,0,1],scene()):
        cloud=cloud.copy();cloud[::5,axis]+=.25;clouds.append(cloud)
    assert e.scan(np.vstack(clouds),.345)
    assert np.linalg.norm(e.p)<.02
    assert e.rmse<.08 and e.last_registration['raw_plane_rmse_m']>e.rmse
    assert e.last_registration['consistent_plane_fraction']>=.6
    assert e.last_registration['plane_count']>=80


def test_majority_inconsistent_planes_do_not_refresh_last_good_pose():
    e=estimator();clouds=[]
    for axis,cloud in zip([2,0,1],scene()):
        cloud=cloud.copy();cloud[np.arange(len(cloud))%5!=0,axis]+=.25;clouds.append(cloud)
    assert not e.scan(np.vstack(clouds),.345)
    assert e.last_good==.245 and e.reason=='insufficient_consistent_planes'
    assert e.last_registration['iteration']==0
    assert np.all(np.isfinite(e.last_registration['predicted_position']))
    assert e.last_registration['candidate_rotation_delta_rad']==0.


def test_trimming_cannot_hide_loss_of_translation_observability():
    e=estimator();ground,wall_x,wall_y=scene()
    wall_x[:,0]+=.25;wall_y[:,1]+=.25
    assert not e.scan(np.vstack([ground,wall_x,wall_y]),.345)
    assert e.last_good==.245
    assert e.reason=='translation_degeneracy'
