#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped
from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, TextAnnotation, Point2, Color


class ObjDetBridge(Node):
    def __init__(self):
        super().__init__('obj_det_annotation_bridge')
        self.sub = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            ImageAnnotations,
            '/zed/zed_node/obj_det/annotations',
            10
        )
        self.get_logger().info('Bridge aktif: obj_det/objects -> obj_det/annotations')

    def callback(self, msg: ObjectsStamped):
        out = ImageAnnotations()

        for obj in msg.objects:
            # --- Kotak bounding box ---
            box = PointsAnnotation()
            box.timestamp = msg.header.stamp
            box.type = PointsAnnotation.LINE_LOOP
            box.outline_color = Color(r=0.0, g=1.0, b=0.0, a=1.0)
            box.thickness = 2.0

            for corner in obj.bounding_box_2d.corners:
                p = Point2()
                p.x = float(corner.kp[0])
                p.y = float(corner.kp[1])
                box.points.append(p)

            out.points.append(box)

            # --- Label + confidence di atas kotak ---
            text = TextAnnotation()
            text.timestamp = msg.header.stamp
            if box.points:
                text.position = box.points[0]
            text.text = f'{obj.label} ({obj.confidence:.0f}%)'
            text.font_size = 14.0
            text.text_color = Color(r=1.0, g=1.0, b=1.0, a=1.0)
            text.background_color = Color(r=0.0, g=0.0, b=0.0, a=0.6)

            out.texts.append(text)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ObjDetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()