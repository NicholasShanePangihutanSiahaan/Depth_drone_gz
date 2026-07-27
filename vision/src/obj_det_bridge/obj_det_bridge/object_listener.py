#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped

class ObjectsListener(Node):
    def __init__(self):
        super().__init__('objects_listener')
        self.sub = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.callback,
            10
        )
        self.get_logger().info('Listening ke /zed/zed_node/obj_det/objects ...')

    def callback(self, msg: ObjectsStamped):
        if not msg.objects:
            return

        self.get_logger().info(f'--- {len(msg.objects)} objek terdeteksi ---')
        for obj in msg.objects:
            label = getattr(obj, 'label', '?')
            label_id = getattr(obj, 'label_id', '?')
            confidence = getattr(obj, 'confidence', 0.0)
            pos = getattr(obj, 'position', None)
            pos_str = f'({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})' if pos else 'N/A'

            self.get_logger().info(
                f'  label={label} | label_id={label_id} | '
                f'confidence={confidence:.1f}% | posisi_3d={pos_str}'
            )


def main():
    rclpy.init()
    node = ObjectsListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()