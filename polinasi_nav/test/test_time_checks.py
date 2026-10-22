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
