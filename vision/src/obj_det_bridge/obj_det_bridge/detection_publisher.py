#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('detection_publisher')
        self.sub = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            Detection2DArray,
            '/zed/zed_node/obj_det/detections_2d',
            10
        )
        self.get_logger().info('detection_publisher aktif -> /obj_det/detections_2d')

    def callback(self, msg: ObjectsStamped):
        out = Detection2DArray()
        out.header = msg.header

        for obj in msg.objects:
            corners = obj.bounding_box_2d.corners
            xs = [c.kp[0] for c in corners]
            ys = [c.kp[1] for c in corners]
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)

            width = x_max - x_min
            height = y_max - y_min
            center_x = x_min + width / 2.0
            center_y = y_min + height / 2.0

            det = Detection2D()
            det.header = msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = obj.label       # nama class, e.g. "fruits"
            hyp.hypothesis.score = obj.confidence / 100.0
            det.results.append(hyp)

            det.bbox.center.position.x = float(center_x)
            det.bbox.center.position.y = float(center_y)
            det.bbox.size_x = float(width)
            det.bbox.size_y = float(height)

            out.detections.append(det)

        self.pub.publish(out)
        if out.detections:
            self.get_logger().info(f'Published {len(out.detections)} detections')


def main():
    rclpy.init()
    node = DetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()