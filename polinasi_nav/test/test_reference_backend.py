"""A detached reference build cannot mutate active ICP arrays or refresh poses."""
from concurrent.futures import Future
import numpy as np
from polinasi_nav.localisation import LidarImuOdometry,build_keyframe_reference


def fixture(asynchronous=True):
    e=LidarImuOdometry([0,0,0,0,0,0],body_imu=True,robust_planes=True,
        asynchronous_reference=asynchronous)
    for i in range(50):e.imu([0,0,0],[0,0,9.81],i*.005)
    x,y=np.meshgrid(np.linspace(-4,4,18),np.linspace(-4,4,18));x,y=x.ravel(),y.ravel()
    world=np.vstack((np.column_stack((x,y,np.zeros_like(x))),
        np.column_stack((np.full_like(x,5.),x,y+5)),
        np.column_stack((x,np.full_like(x,5.),y+5))))
    assert e.scan(world,.245)
    assert e.reference_job is None  # Bootstrap is synchronous.
    return e,world


class ControlledExecutor:
    def __init__(self):self.jobs=[]
    def submit(self,fn,*args):
        f=Future();self.jobs.append((f,fn,args));return f


def test_active_reference_is_unchanged_until_complete_swap_and_pending_jobs_bounded():
    e,world=fixture();executor=ControlledExecutor();e.reference_executor=executor
    reference,tree,normals=e.reference,e.tree,e.normals
    e.p[0]=.6;cloud=world+e.p
    pose=e.p.copy();velocity=e.v.copy();stamp=e.last_good
    e.freeze_keyframe(cloud)
    assert len(executor.jobs)==1 and e.reference is reference and e.tree is tree and e.normals is normals
    assert e.collect_reference() and e.reference is reference
    cloud[:]=99.  # Submitted cloud is an owned copy.
    e.freeze_keyframe(world+.7)
    assert len(executor.jobs)==1 and e.reference_stats['busy_skips']==1
    future,fn,args=executor.jobs[0]
    expected=build_keyframe_reference(args[0],True)
    future.set_result(fn(*args))
    assert e.reference is reference  # Completion never mutates the live core.
    assert e.collect_reference() and e.reference is not reference
    np.testing.assert_array_equal(e.reference,expected['reference'])
    np.testing.assert_array_equal(e.normals,expected['normals'])
    np.testing.assert_array_equal(e.normal_valid,expected['normal_valid'])
    np.testing.assert_array_equal(e.p,pose);np.testing.assert_array_equal(e.v,velocity)
    assert e.last_good==stamp and e.reference_stats['completed']==1
    assert e.tree.n==len(e.reference)==len(e.normals)
    assert e.reference_job is None


def test_failed_worker_does_not_commit_partial_reference_or_accept_a_scan():
    e,world=fixture();executor=ControlledExecutor();e.reference_executor=executor
    old=e.reference;e.p[0]=.6;e.freeze_keyframe(world+.6)
    executor.jobs[0][0].set_exception(RuntimeError('test backend failure'))
    callbacks=[]
    assert not e.scan(world,.345,on_accepted=lambda:callbacks.append(True))
    assert e.reason=='reference_backend_error' and e.reference is old
    assert not callbacks and e.last_good==.245
    assert 'test backend failure' in e.reference_stats['error']


def test_real_worker_has_same_reference_and_next_scan_result_as_serial_backend():
    asynchronous,world=fixture();serial,_=fixture(False)
    try:
        for e in (asynchronous,serial):
            e.last_keyframe_position=e.p+np.array([.6,0,0])
            for j in range(20):e.imu([0,0,0],[0,0,9.81],.25+j*.005)
            assert e.scan(world,.345)
        asynchronous.reference_job[0].result(timeout=5.)
        assert asynchronous.collect_reference()
        np.testing.assert_array_equal(asynchronous.reference,serial.reference)
        np.testing.assert_array_equal(asynchronous.normals,serial.normals)
        for e in (asynchronous,serial):
            for j in range(20):e.imu([0,0,0],[0,0,9.81],.35+j*.005)
            assert e.scan(world,.445)
        for name in ('p','r','v','covariance'):
            np.testing.assert_allclose(getattr(asynchronous,name),getattr(serial,name),rtol=0,atol=1e-12)
        assert asynchronous.last_good==serial.last_good
    finally:asynchronous.close()
