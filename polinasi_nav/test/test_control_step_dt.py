import pytest
from polinasi_nav.runtime import control_step_dt


@pytest.mark.parametrize('raw,expected',[(.05,.05),(.1,.1),(.10000000000001,.1),(.109,.1),(.2,.1)])
def test_reference_and_pid_use_the_same_bounded_integration_step(raw,expected):
    assert control_step_dt(raw)==expected


@pytest.mark.parametrize('raw',[0.,-.01,.201,float('nan'),float('inf')])
def test_real_clock_gaps_and_invalid_intervals_are_not_hidden(raw):
    with pytest.raises(ValueError):control_step_dt(raw)
