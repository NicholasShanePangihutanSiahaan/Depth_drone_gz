import json
import math
from polinasi_nav.planning import Health
from polinasi_nav.time_checks import fresh_stamp
from polinasi_nav.depth_health import fresh_depth


def test_recorded_pose_boundary_is_inclusive_despite_binary_float_subtraction():
    now=29.900000000000002;stamp=29.6;c=dict(pose_timeout=.3,cloud_timeout=.5)
    assert now-stamp>.3
    health=Health(cloud_stamp=now,pose_stamp=stamp,imu_stamp=now,localisation_ok=True)
    assert health.reason(now,c,True)==''
    assert fresh_stamp(now,stamp,.3) and fresh_depth(now,stamp,.3)
    detail=health.status(now,c)
    assert detail['pose_stamp']==stamp and detail['pose_age']>.3
    assert detail['pose_timeout_seconds']==.3
    json.dumps(detail,allow_nan=False)


def test_real_pose_lidar_and_imu_lateness_remains_rejected():
    now=29.90001;c=dict(pose_timeout=.3,cloud_timeout=.5)
    assert Health(now,29.6,now,True).reason(now,c,True)=='stale_localisation'
    assert Health(29.4,now,now,True).reason(now,c,True)=='stale_lidar'
    assert Health(now,now,29.6,True).reason(now,c,True)=='stale_imu'
    assert not fresh_stamp(.30000001,0.,.3)  # 10 ns is not floating cancellation.
    assert not fresh_stamp(121.064,120.955,.1)  # Genuine .109-second lateness stays false.
    assert Health(now,now,now,False).reason(now,c,True)=='localisation_loss'


def test_future_missing_nonfinite_timestamps_never_authorize_freshness():
    for stamp in (None,math.nan,math.inf,-math.inf,2.001):
        assert not fresh_stamp(2.,stamp,.3)
    assert not fresh_stamp(math.nan,2.,.3)
    json.dumps(Health().status(2.,dict(pose_timeout=.3,cloud_timeout=.5)),allow_nan=False)


def test_causal_imu_selection_reproduces_run33_without_accepting_future_data():
    from polinasi_nav.time_checks import CausalStampQueue
    q=CausalStampQueue();q.push(230.995)
    assert q.select(231.)==230.995
    q.push(231.005)
    assert q.select(231.003)==230.995
    assert not fresh_stamp(231.003,231.005,.3)
    assert fresh_stamp(231.003,q.selected,.3)
    assert q.status()['future_pending_at_last_tick']==1
    assert q.select(231.01)==231.005
    assert q.select(231.306)==231.005
    assert not fresh_stamp(231.306,q.selected,.3)  # Real dropout stays stale.


def test_future_only_flood_and_invalid_packets_never_freshen_imu_health():
    from polinasi_nav.time_checks import CausalStampQueue
    q=CausalStampQueue(capacity=3)
    for t in (5.,6.,7.,8.):q.push(t)
    assert q.select(1.)==-math.inf and q.status()['overflow_drops']==1
    for t in (None,math.nan,math.inf):assert not q.push(t)
    assert q.select(6.)==6.
    assert not q.push(5.) and q.selected==6.
    assert q.select(5.)==-math.inf  # Never expose a future stamp on clock rewind.
    json.dumps(q.status(),allow_nan=False)


def test_out_of_order_pending_packets_choose_latest_eligible_original_time():
    from polinasi_nav.time_checks import CausalStampQueue
    q=CausalStampQueue()
    for t in (3.,2.,2.5):q.push(t)
    assert q.select(2.6)==2.5
    assert q.select(2.7)==2.5
    assert q.select(3.)==3.
