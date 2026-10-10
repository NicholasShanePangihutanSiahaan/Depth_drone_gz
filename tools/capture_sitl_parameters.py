"""Read-only parameter snapshot, loopback SITL only, no serial/hardware option."""
import argparse
import json
import time
from pathlib import Path
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=90.)
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError('refusing to overwrite parameter snapshot')
    link = mavutil.mavlink_connection('tcp:127.0.0.1:5762', source_system=251)
    heartbeat = link.wait_heartbeat(timeout=15.)
    if heartbeat is None or heartbeat.get_srcSystem() != 1:
        raise RuntimeError('expected loopback simulation system 1')
    link.mav.param_request_list_send(1, 1)
    parameters, expected = {}, None
    deadline = time.monotonic()+args.timeout
    while time.monotonic() < deadline:
        msg = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.)
        if msg is not None:
            name = msg.param_id.rstrip('\x00')
            parameters[name] = float(msg.param_value)
            expected = msg.param_count
            if len(parameters) >= expected:
                break
    link.close()
    if expected is None or len(parameters) != expected:
        raise RuntimeError(f'incomplete parameter snapshot: {len(parameters)}/{expected}')
    if parameters.get('SIM_GPS_DISABLE') != 1 or parameters.get('GPS1_TYPE') != 0:
        raise RuntimeError('expected GPS-disabled SITL configuration')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(endpoint='tcp:127.0.0.1:5762', parameters=parameters), indent=2)+'\n')
    print(f'Saved {len(parameters)} SITL parameters: {args.output}')


if __name__ == '__main__':
    main()
