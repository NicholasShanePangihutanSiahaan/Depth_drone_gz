import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from mid360_slam_test.geometry import pose,inverse,transform,angle
from mid360_slam_test.estimator import SlamEstimator,icp,optimise_graph,DEFAULTS


def scene():
    # Asymmetric 3D landmarks; analytic fixture, not a measured environment.
    return np.random.default_rng(42).uniform([-3,-3,-1],[3,3,2],(1000,3))


def settings():
    return dict(voxel_m=.03,min_points=50,keyframe_translation_m=.08,
        loop_min_separation=5,loop_min_travel_m=.8,loop_min_seconds=.4,loop_cooldown_s=.2,
        correspondence_m=.3,max_rmse_m=.03,loop_rmse_m=.03,loop_radius_m=.22)


def test_icp_recovers_rigid_motion_with_imu_seed():
    world=scene()
    truth=pose([.1,-.08,.03],Rotation.from_euler('xyz',[2,1,7],degrees=True).as_quat())
    source=transform(inverse(truth),world)
    guess=truth.copy(); guess[:3,3]=0
    estimated,quality=icp(source,world,guess,dict(DEFAULTS,**settings()))
    assert estimated is not None and quality['converged']
    assert np.linalg.norm(estimated[:3,3]-truth[:3,3])<1e-6
    assert angle(inverse(truth)@estimated)<1e-5


def test_real_registration_loop_event_and_local_odom_unchanged():
    backend=SlamEstimator(settings())
    world=scene(); events=[]
    positions=[0,.1,.2,.3,.4,.5,.5,.4,.3,.2,.1,0]
    for i,x in enumerate(positions):
        output=backend.scan(transform(inverse(pose([x,0,0])),world),1+i*.25,np.eye(3))
        assert output['valid'], output
        assert output['local']['position'][0]==pytest.approx(x,abs=1e-5)
        events.extend(output['loop_events'])
    accepted=[e for e in events if e['accepted']]
    assert accepted and accepted[0]['optimisation']['success']
    assert accepted[0]['graph_solution']
    assert backend.local_pose[0,3]==pytest.approx(0,abs=1e-5)


def test_rotating_scene_uses_all_rotation_axes():
    world=scene(); backend=SlamEstimator(settings())
    for i,degrees in enumerate(range(0,361,10)):
        r=Rotation.from_euler('z',degrees,degrees=True)
        output=backend.scan(r.inv().apply(world),1+i*.25,r.as_matrix())
        assert output['valid'], output
        assert angle(pose(quaternion=output['local']['quaternion'])@pose(quaternion=r.inv().as_quat()))<1e-4


def test_sparse_or_planar_cloud_rejected_and_loss_latched():
    b=SlamEstimator(settings())
    assert not b.scan(np.zeros((2,3)),1,np.eye(3))['valid']
    assert not b.scan(scene(),1.1,np.eye(3))['valid']
    grid=np.array([[x,y,0.] for x in np.linspace(-2,2,20) for y in np.linspace(-2,2,20)])
    result,quality=icp(grid,grid,np.eye(4),dict(DEFAULTS,**settings()))
    assert result is None and quality['reason']=='degenerate_geometry'


def test_scan_timestamp_gap_rejected():
    b=SlamEstimator(settings()); b.scan(scene(),1,np.eye(3))
    output=b.scan(scene(),2,np.eye(3))
    assert output['tracking']=='LOST' and 'gap' in output['reason']


def test_proximity_without_travel_does_not_create_loop():
    b=SlamEstimator(settings())
    for i in range(30):
        output=b.scan(scene(),1+i*.1,np.eye(3))
        assert not output['loop_events']
    assert output['loops_accepted']==0


def test_bad_loop_geometry_is_rejected_without_map_change():
    b=SlamEstimator(settings())
    world=scene()
    for i,x in enumerate([0,.1,.2,.3,.4,.5]):
        b.scan(transform(inverse(pose([x,0,0])),world),1+i*.25,np.eye(3))
    b.local_pose=pose([0,0,0]); b.travel=2.
    b.add_keyframe(world+20.,3.)
    before=[t.copy() for t in b.map_poses]
    event=b.try_loop(4.)
    assert event and not event['accepted']
    assert all(np.array_equal(a,c) for a,c in zip(before,b.map_poses))


def test_graph_corrects_drift_and_keeps_first_pose_fixed():
    poses=[pose([x,0,0]) for x in (0,.3,.6,.3,.12)]
    edges=[(i,i+1,inverse(poses[i])@poses[i+1]) for i in range(4)]
    edges.append((0,4,np.eye(4)))
    updated,quality=optimise_graph(poses,edges,DEFAULTS)
    assert updated is not None and quality['residual_after']<quality['residual_before']
    assert np.array_equal(updated[0],poses[0])
    assert abs(updated[-1][0,3])<.12
    assert poses[-1][0,3]==.12


@pytest.mark.parametrize('key,value',[('voxel_m',0),('min_points',-1),('max_keyframes',1),
    ('min_overlap',1.1),('graph_translation_sigma_m',float('nan'))])
def test_estimator_rejects_bad_settings(key,value):
    with pytest.raises(ValueError): SlamEstimator({key:value})
