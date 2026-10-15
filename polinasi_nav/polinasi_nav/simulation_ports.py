"""Simulation loopback MAVROS endpoint; never a serial/hardware connection."""
import os


def mavros_endpoint(environment=None):
    environment=os.environ if environment is None else environment
    value=environment.get('POLINASI_MAVROS_UDP_PORT','14550')
    if not str(value).isdigit() or not 1024<=int(value)<=65535:
        raise ValueError('invalid simulation MAVROS UDP port')
    return f'udp://127.0.0.1:{int(value)}@'
