"""Read-only check of the mapping I/O process's real visualization messages."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path as PathMsg
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--output', type=Path, default=Path('reports/mapping_output.json'))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('mapping_output_probe')
    result = {}
    topics = {'/mapping/global_cloud': PointCloud2, '/navigation/occupancy': MarkerArray,
              '/navigation/planned_path': PathMsg, '/navigation/executed_path': PathMsg,
              '/navigation/survey_route': PathMsg}
    def receive(topic, message):
        if isinstance(message, MarkerArray):
            frame = message.markers[0].header.frame_id if message.markers else ''
            count = sum(len(marker.points) for marker in message.markers)
        else:
            frame = message.header.frame_id
            count = message.width*message.height if isinstance(message, PointCloud2) else len(message.poses)
        result[topic] = dict(frame=frame, displayed_elements=count)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic, kind in topics.items():
        node.create_subscription(kind, topic, lambda msg, t=topic: receive(t, msg), qos)
    deadline = time.monotonic()+args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    for topic, entry in result.items():
        entry['publishers'] = node.count_publishers(topic)
    passed = (len(result) == len(topics) and all(item['publishers'] == 1 for item in result.values())
              and all(item['frame'] == ('odom' if topic == '/navigation/executed_path' else 'map')
                      for topic, item in result.items())
              and all(result[topic]['displayed_elements'] > 0 for topic in
                      ('/mapping/global_cloud', '/navigation/occupancy', '/navigation/survey_route')))
    report = dict(passed=passed, topics=result, commands_sent=False,
                  scope='Visualization message check, not flight completion or LiDAR localisation validation')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
