#!/usr/bin/env python3
import sys
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from zed_msgs.msg import ObjectsStamped
from cv_bridge import CvBridge
import message_filters


class VisualSubscriber(Node):
    def __init__(self, save_mode=False):
        super().__init__('visual_subscriber')
        self.bridge = CvBridge()
        self.save_mode = save_mode
        self.frame_count = 0

        self.last_time = time.time()
        self.fps = 0.0

        image_sub = message_filters.Subscriber(
            self, Image, '/zed/zed_node/left/image_rect_color')
        objects_sub = message_filters.Subscriber(
            self, ObjectsStamped, '/zed/zed_node/obj_det/objects')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, objects_sub], queue_size=10, slop=0.2)
        self.ts.registerCallback(self.callback)

        mode_str = 'SAVE ke file' if save_mode else 'LIVE cv2.imshow'
        self.get_logger().info(f'Visual subscriber aktif, mode: {mode_str}')

    def draw_info(self, frame, obj_msg: ObjectsStamped):
        x, y = 10, 25
        line_height = 26

        labels = [obj.label for obj in obj_msg.objects]

        overlay = frame.copy()
        box_height = line_height * (2 + max(len(labels), 1))
        cv2.rectangle(overlay, (0, 0), (280, box_height + 10), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        cv2.putText(frame, f'FPS: {self.fps:.1f}', (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        y += line_height

        if labels:
            for label in labels:
                cv2.putText(frame, f'ID Class Detected: {label}', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                y += line_height
        else:
            cv2.putText(frame, 'ID Class Detected: ---', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    

    def callback(self, img_msg: Image, obj_msg: ObjectsStamped):
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # Hitung FPS aktual
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 0:
            self.fps = 1.0 / dt

        for obj in obj_msg.objects:
            corners = obj.bounding_box_2d.corners
            xs = [c.kp[0] for c in corners]
            ys = [c.kp[1] for c in corners]
            x1, y1 = int(min(xs)), int(min(ys))
            x2, y2 = int(max(xs)), int(max(ys))

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f'{obj.label} {obj.confidence:.0f}%'
            cv2.putText(frame, text, (x1, max(y1 - 8, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.draw_info(frame, obj_msg)

        if self.save_mode:
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                filename = f'/tmp/detection_snapshot.jpg'
                cv2.imwrite(filename, frame)
                self.get_logger().info(f'Snapshot disimpan: {filename}')
        else:
            display_frame = cv2.resize(frame, None, fx=1.5, fy=1.5)
            cv2.imshow('ZED Object Detection', frame)
            cv2.waitKey(1)

def main():
    save_mode = '--save' in sys.argv
    rclpy.init()
    node = VisualSubscriber(save_mode=save_mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()