"""Request mission start and require acknowledgement; simulation topics only."""
import argparse
import json
import time
import rclpy
from std_msgs.msg import Bool, String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('simulation_mission_start')
    status = {}
    def update(msg):
        status.update(json.loads(msg.data))
    node.create_subscription(String, '/navigation/status', update, 10)
    pub = node.create_publisher(Bool, '/mission/start', 10)
    deadline, next_send = time.monotonic()+args.timeout, 0.
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if status.get('mission_start_received'):
                print('Mission start acknowledged. Monitor /navigation/status for flight progress.')
                return
            if (status and status.get('health_reason') == ''
                    and pub.get_subscription_count() > 0 and time.monotonic() >= next_send):
                pub.publish(Bool(data=True))
                next_send = time.monotonic()+.5
        raise RuntimeError(f'Mission did not acknowledge start; latest status: {status}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
