import pytest
from polinasi_nav.simulation_ports import mavros_endpoint


def test_fixed_port_and_default_both_remain_loopback_only():
    assert mavros_endpoint({})=='udp://127.0.0.1:14550@'
    assert mavros_endpoint({'POLINASI_MAVROS_UDP_PORT':'14650'})=='udp://127.0.0.1:14650@'


@pytest.mark.parametrize('port',['0','80','65536','tcp://remote','14650;echo unsafe','-1',''])
def test_invalid_port_cannot_become_an_arbitrary_endpoint(port):
    with pytest.raises(ValueError):mavros_endpoint({'POLINASI_MAVROS_UDP_PORT':port})
