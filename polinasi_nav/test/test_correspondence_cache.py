import numpy as np
from scipy.spatial import cKDTree
from polinasi_nav.correspondence_cache import ExactCorrespondenceCache
from polinasi_nav.localisation import LidarImuOdometry


def equivalent(tree,cache,points):
    d,i=tree.query(points,distance_upper_bound=.5);e,j=cache.query(points)
    valid=d<.5
    np.testing.assert_array_equal(valid,e<.5)
    np.testing.assert_array_equal(i[valid],j[valid])
    np.testing.assert_allclose(d[valid],e[valid],rtol=0,atol=1e-14)
    assert np.all(j<max(1,tree.n))


def test_exact_cache_handles_cumulative_motion_crossings_ties_and_radius_boundary():
    tree=cKDTree([[0,0,0],[.2,0,0],[2.,0,0]])
    cache=ExactCorrespondenceCache(tree)
    for x in (.001,.03,.06,.09,.1,.1000000000001,.11,.15,.21,.49,.5,.5000000000001,1.49,1.5,1.51):
        equivalent(tree,cache,np.array([[x,0,0],[2.+x,0,0],[20.,20.,20.]]))
    assert cache.stats['reused_points']>0 and cache.stats['tie_fallback_points']>0


def test_random_exact_queries_and_shape_changes_never_keep_stale_cache_rows():
    rng=np.random.default_rng(42);data=rng.uniform(-3,3,(1000,3));tree=cKDTree(data)
    cache=ExactCorrespondenceCache(tree);base=data[:100]+rng.normal(0,.001,(100,3))
    for i in range(30):equivalent(tree,cache,base+rng.normal(0,.001+i*.0001,base.shape))
    equivalent(tree,cache,base[:10]);equivalent(tree,cache,base)
    equivalent(tree,cache,np.empty((0,3)))
    assert cache.stats['reused_points']>1000


def test_cached_icp_preserves_acceptance_state_and_covariance_against_uncached():
    def estimator(enabled):
        e=LidarImuOdometry([0,0,0,0,0,0],body_imu=True,robust_planes=True,cache_correspondences=enabled)
        for i in range(50):e.imu([0,0,0],[0,0,9.81],i*.005)
        return e
    cached,plain=estimator(True),estimator(False)
    x,y=np.meshgrid(np.linspace(-4,4,25),np.linspace(-4,4,25));x,y=x.ravel(),y.ravel()
    world=np.vstack((np.column_stack((x,y,np.zeros_like(x))),
        np.column_stack((np.full_like(x,5.),x,y+5)),
        np.column_stack((x,np.full_like(x,5.),y+5))))
    assert cached.scan(world,.245) and plain.scan(world,.245)
    rng=np.random.default_rng(17);reuse=0
    for k in range(1,9):
        stamp=.245+k*.1
        cloud=np.vstack([world+rng.normal(0,.003,world.shape),rng.uniform(20,30,(30,3))])
        for e in (cached,plain):
            for j in range(20):e.imu([0,0,0],[0,0,9.81],stamp-.095+j*.005)
        assert cached.scan(cloud,stamp)==plain.scan(cloud,stamp)
        assert cached.reason==plain.reason=='tracking'
        for name in ('p','r','v','covariance'):
            np.testing.assert_allclose(getattr(cached,name),getattr(plain,name),rtol=0,atol=1e-12)
        assert cached.last_good==plain.last_good
        reuse+=cached.last_scan_timing['correspondence_cache']['reused_points']
    assert reuse>0
