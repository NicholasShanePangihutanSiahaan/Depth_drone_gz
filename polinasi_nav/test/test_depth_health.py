from polinasi_nav.depth_health import DepthHoldGuard,fresh_depth


def test_exact_timeout_is_not_roundoff_fault():
    assert fresh_depth(255.4,254.4,1.)
    assert not fresh_depth(255.45,254.4,1.)


def test_stale_integration_can_only_wait_while_safe_stationary():
    guard = DepthHoldGuard()
    depth = dict(last_integrated_stamp=3.,last_valid_stamp=4.)
    assert guard.assess(4.2,depth,True,1.)=='hold_wait_for_integration'
    assert guard.assess(4.3,depth,False,1.)=='fail'
    depth['last_integrated_stamp'] = 4.2
    assert guard.assess(4.4,depth,True,1.)=='fresh'
    assert guard.wait_since is None


def test_receiving_valid_frames_does_not_bypass_bounded_hold_timeout():
    guard = DepthHoldGuard()
    for now in (4.2,5.,6.):
        assert guard.assess(now,dict(last_integrated_stamp=3.,last_valid_stamp=now),True,1.)=='hold_wait_for_integration'
    assert guard.assess(6.3,dict(last_integrated_stamp=3.,last_valid_stamp=6.3),True,1.)=='fail'
    assert DepthHoldGuard().assess(4.2,dict(last_integrated_stamp=3.,last_valid_stamp=3.),True,1.)=='fail'
