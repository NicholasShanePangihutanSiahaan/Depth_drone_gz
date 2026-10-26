"""Exact correspondence pruning preserves the existing ICP acceptance policy."""
import numpy as np
from scipy.spatial import cKDTree
from polinasi_nav.localisation import LidarImuOdometry


def test_bounded_nearest_search_preserves_inliers_and_masks_missing_indices():
    e=LidarImuOdometry([0,0,0,0,0,0],body_imu=True)
    e.tree=cKDTree([[0.,0.,0.],[2.,0.,0.]])
    points=np.array([[0.,0.,0.],[.4999999,0.,0.],[.5,0.,0.],[2.1,0.,0.],[20.,0.,0.]])
    old,old_ids=e.tree.query(points)
    new,new_ids=e.query_correspondences(points)
    valid=old<.5
    np.testing.assert_array_equal(new<.5,valid)
    np.testing.assert_array_equal(new[valid],old[valid])
    np.testing.assert_array_equal(new_ids[valid],old_ids[valid])
    assert np.all(new_ids<e.tree.n) and np.isinf(new[-1])
    assert not (new[2]<.5)  # Exact threshold remains excluded.


def test_pruned_and_unbounded_icp_have_same_acceptance_pose_and_residual():
    def estimator(legacy=False):
        e=LidarImuOdometry([0,0,0,0,0,0],body_imu=True,robust_planes=True)
        if legacy:e.query_correspondences=lambda points:e.tree.query(points)
        for i in range(50):e.imu([0,0,0],[0,0,9.81],i*.005)
        return e
    bounded,legacy=estimator(),estimator(True)
    x,y=np.meshgrid(np.linspace(-4,4,25),np.linspace(-4,4,25));x,y=x.ravel(),y.ravel()
    world=np.vstack((np.column_stack((x,y,np.zeros_like(x))),
        np.column_stack((np.full_like(x,5.),x,y+5)),
        np.column_stack((x,np.full_like(x,5.),y+5))))
    assert bounded.scan(world,.245) and legacy.scan(world,.245)
    rng=np.random.default_rng(42)
    for k in range(1,9):
        stamp=.245+k*.1
        cloud=np.vstack((world+rng.normal(0,.003,world.shape),rng.uniform(20,30,(40,3))))
        for e in (bounded,legacy):
            for j in range(20):e.imu([0,0,0],[0,0,9.81],stamp-.095+j*.005)
        assert bounded.scan(cloud,stamp)==legacy.scan(cloud,stamp)
        assert bounded.reason==legacy.reason=='tracking'
        for name in ('p','r','v','covariance'):np.testing.assert_allclose(getattr(bounded,name),getattr(legacy,name),rtol=0,atol=1e-12)
        assert bounded.rmse==legacy.rmse and bounded.last_good==legacy.last_good
