from polinasi_nav.takeoff import TakeoffSequence


def test_original_single_takeoff_then_hover_gate():
    seq = TakeoffSequence(0., .2)
    assert seq.step(.2, .2, True, False, True, 0.) is None
    assert seq.step(.4, .2, True, False, True, 0.) == 'takeoff'
    assert seq.step(1., .2, True, False, True, 0.) is None
    seq.acknowledge(0)
    assert seq.step(1.1, .2, True, False, True, 0.) is None
    assert seq.step(4., 2., True, True, True, 0.) is None
    assert seq.step(6.1, 2., True, True, True, 0.) == 'ready'
    assert seq.attempts == 1


def test_temporary_rejection_only_bounded_retries():
    seq = TakeoffSequence(0., .2)
    for start in (.4, 3., 5.6):
        assert seq.step(start, .2, True, False, True, 0.) == 'takeoff'
        seq.acknowledge(1)
        action = seq.step(start+.1, .2, True, False, True, 0.)
        if seq.attempts < 3:
            assert action is None
            seq.step(start+2.2, .2, True, False, True, 0.)
        else:
            assert action == 'abort'
    assert seq.failure == 'takeoff_rejected_1'


def test_hard_rejection_not_retried():
    seq = TakeoffSequence(0., .2)
    seq.step(.4, .2, True, False, True, 0.)
    seq.acknowledge(4)
    assert seq.step(.5, .2, True, False, True, 0.) == 'abort'
    assert seq.attempts == 1


def test_accepted_but_no_climb_is_not_success():
    seq = TakeoffSequence(0., .2)
    seq.step(.4, .2, True, False, True, 0.)
    seq.acknowledge(0)
    seq.step(.5, .2, True, False, True, 0.)
    assert seq.step(20.6, .2, True, False, True, 0.) == 'abort'
    assert seq.failure == 'takeoff_no_climb'


def test_unsafe_hover_never_hands_off_navigation():
    seq = TakeoffSequence(0., .2)
    seq.step(.4, .2, True, False, True, 0.)
    seq.acknowledge(0)
    seq.step(.5, .2, True, False, True, 0.)
    seq.step(4., 2., True, True, True, 0.)
    seq.step(5., 2., True, True, False, 0.)
    assert seq.step(6.1, 2., True, True, True, 0.) is None
