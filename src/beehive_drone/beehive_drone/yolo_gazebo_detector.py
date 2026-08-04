#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from zed_msgs.msg import BoundingBox2Di, Keypoint2Di, Object, ObjectsStamped

from beehive_drone.mission_params import MissionConfig


class YoloGazeboDetector(Node):
    """YOLO + registered depth detector for Gazebo.

    The node publishes a standard PointStamped in the camera optical frame, so
    localization is performed through TF rather than hard-coded UAV yaw math.
    """

    def __init__(self) -> None:
        super().__init__("yolo_gazebo_detector")

        self.declare_parameter("model_path", "")
        self.declare_parameter("device", MissionConfig.YOLO_DEVICE)
        self.declare_parameter("confidence", MissionConfig.YOLO_CONFIDENCE)
        self.declare_parameter("iou", MissionConfig.YOLO_IOU)
        self.declare_parameter("min_depth", MissionConfig.YOLO_MIN_DEPTH)
        self.declare_parameter("max_depth", MissionConfig.YOLO_MAX_DEPTH)
        self.declare_parameter("target_labels", MissionConfig.YOLO_TARGET_LABELS)
        self.declare_parameter("rgb_topic", "/zed2i/left/image_rect_color")
        self.declare_parameter("depth_topic", "/zed2i/depth/depth_registered")
        self.declare_parameter("camera_info_topic", "/zed2i/left/camera_info")
        self.declare_parameter("show_preview", False)
        self.declare_parameter("publish_zed_objects", True)

        model_path = str(self.get_parameter("model_path").value)
        if not model_path:
            raise RuntimeError("Parameter model_path wajib diisi untuk YOLO detector.")

        self.device = str(self.get_parameter("device").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.target_labels = {
            str(label).strip().lower()
            for label in self.get_parameter("target_labels").value
        }
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.publish_zed_objects = bool(
            self.get_parameter("publish_zed_objects").value
        )

        self.bridge = CvBridge()
        self.model = YOLO(model_path, task="detect")
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        rgb_topic = str(self.get_parameter("rgb_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)

        self.create_subscription(
            CameraInfo, info_topic, self.camera_info_callback, qos_profile_sensor_data
        )
        self.rgb_sub = message_filters.Subscriber(
            self, Image, rgb_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.08
        )
        self.sync.registerCallback(self.sync_callback)

        self.point_pub = self.create_publisher(
            PointStamped, "/perception/tree_position_camera", 10
        )
        self.objects_pub = self.create_publisher(
            ObjectsStamped, "/zed/zed_node/obj_det/objects", 10
        )
        self.annotated_pub = self.create_publisher(
            Image, "/perception/yolo/annotated", 10
        )
        self.last_error_log = -1e9
        self.get_logger().info(f"YOLO detector aktif: {model_path}, device={self.device}.")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 6 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def depth_to_meters(self, msg: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth = np.asarray(depth)
        if msg.encoding in {"16UC1", "mono16"}:
            return depth.astype(np.float32) * 0.001
        return depth.astype(np.float32)

    def calculate_depth(
        self, depth_image: np.ndarray, x1: float, y1: float, x2: float, y2: float
    ) -> Optional[float]:
        width = max(1.0, x2 - x1)
        height = max(1.0, y2 - y1)
        margin_x = int(width * 0.25)
        margin_y = int(height * 0.25)
        rx1 = max(0, int(x1) + margin_x)
        ry1 = max(0, int(y1) + margin_y)
        rx2 = min(depth_image.shape[1], int(x2) - margin_x)
        ry2 = min(depth_image.shape[0], int(y2) - margin_y)
        if rx2 <= rx1 or ry2 <= ry1:
            return None

        roi = depth_image[ry1:ry2, rx1:rx2]
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= self.min_depth) & (valid <= self.max_depth)]
        if valid.size < 5:
            return None
        return float(np.median(valid))

    def project(self, u: float, v: float, depth: float) -> Optional[Tuple[float, float, float]]:
        if None in (self.fx, self.fy, self.cx, self.cy):
            return None
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        return float(x), float(y), float(depth)

    def sync_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        if None in (self.fx, self.fy, self.cx, self.cy):
            return
        try:
            image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_image = self.depth_to_meters(depth_msg)
            results = self.model.predict(
                source=image,
                device=self.device,
                conf=self.confidence,
                iou=self.iou,
                verbose=False,
            )
        except Exception as exc:  # noqa: BLE001
            if self.now_sec() - self.last_error_log > 2.0:
                self.last_error_log = self.now_sec()
                self.get_logger().error(f"YOLO/depth callback gagal: {exc}")
            return

        objects_msg = ObjectsStamped()
        objects_msg.header = rgb_msg.header

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                confidence = float(box.conf[0].item())
                class_id = int(box.cls[0].item())
                label = str(results[0].names[class_id])
                if self.target_labels and label.lower() not in self.target_labels:
                    continue

                depth = self.calculate_depth(depth_image, x1, y1, x2, y2)
                if depth is None:
                    continue
                center_u = 0.5 * (x1 + x2)
                center_v = 0.5 * (y1 + y2)
                point = self.project(center_u, center_v, depth)
                if point is None:
                    continue

                point_msg = PointStamped()
                point_msg.header = rgb_msg.header
                point_msg.point.x, point_msg.point.y, point_msg.point.z = point
                self.point_pub.publish(point_msg)

                if self.publish_zed_objects:
                    obj = Object()
                    obj.label = label
                    obj.label_id = class_id
                    obj.confidence = confidence * 100.0
                    obj.position = [point[0], point[1], point[2]]
                    obj.bounding_box_2d = BoundingBox2Di()
                    for px, py in ((x1, y1), (x2, y1), (x2, y2), (x1, y2)):
                        corner = Keypoint2Di()
                        corner.kp = [int(round(px)), int(round(py))]
                        obj.bounding_box_2d.corners.append(corner)
                    objects_msg.objects.append(obj)

                cv2.rectangle(
                    image,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    image,
                    f"{label} {confidence:.2f} {depth:.2f}m",
                    (int(x1), max(20, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2,
                )

        if self.publish_zed_objects:
            self.objects_pub.publish(objects_msg)
        annotated = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        annotated.header = rgb_msg.header
        self.annotated_pub.publish(annotated)

        if self.show_preview:
            cv2.imshow("YOLO Gazebo", image)
            cv2.waitKey(1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloGazeboDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
