#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from zed_msgs.msg import ObjectsStamped, Object, BoundingBox2Di, Keypoint2Di
from cv_bridge import CvBridge

import cv2
import numpy as np
import message_filters
from ultralytics import YOLO

class YoloGazeboDetector(Node):
    def __init__(self):
        super().__init__("yolo_gazebo_detector")
        self.bridge = CvBridge()

        # ==========================================
        # 1. Load Model YOLOv8 ONNX
        # ==========================================
        self.model = YOLO('/home/shane/ProjekAtaka/gazebo_sim/vision/scripts/best_detection_palm_oil.onnx', task='detect')

        # ==========================================
        # 2. Parameter Intrinsik Kamera ZED 2i 
        # ==========================================
        self.fx = 381.3611502479812
        self.fy = 381.3611502479812
        self.cx = 320.0
        self.cy = 240.0

        # ==========================================
        # 3. Subscriber (Membaca Kamera Gazebo)
        # ==========================================
        self.rgb_sub = message_filters.Subscriber(
            self, Image, "/zed2i/left/image_rect_color"
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, "/zed2i/depth/depth_registered"
        )

        # Sinkronisasi waktu antara RGB dan Depth (Maksimal selisih 0.05 detik)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.05
        )
        self.ts.registerCallback(self.sync_callback)

        # ==========================================
        # 4. Publisher (Mengelabui sistem seolah ini dari ZED SDK)
        # ==========================================
        self.pub = self.create_publisher(
            ObjectsStamped, "/zed/zed_node/obj_det/objects", 10
        )

        self.get_logger().info("YOLO Gazebo Detector (ZED Emulator + Live View) Aktif!")

    def calculate_depth(self, depth_image, x, y, w, h):
        """Mengekstrak median kedalaman dari bounding box dengan margin 40%"""
        margin_x = int(w * 0.40)
        margin_y = int(h * 0.40)

        x1 = max(0, int(x) + margin_x)
        y1 = max(0, int(y) + margin_y)
        x2 = min(depth_image.shape[1], int(x + w) - margin_x)
        y2 = min(depth_image.shape[0], int(y + h) - margin_y)

        roi = depth_image[y1:y2, x1:x2]

        if x2 <= x1 or y2 <= y1:
            return None

        # Filter nilai NaN, Infinity, dan batasan jarak sensor (0.2m - 20m)
        roi = roi[np.isfinite(roi)]
        roi = roi[(roi > 0.2) & (roi < 20.0)]

        if roi.size == 0:
            return None

        return float(np.median(roi))

    def sync_callback(self, rgb_msg, depth_msg):
        # Konversi ROS Image ke format OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        # ==========================================
        # JALANKAN INFERENSI DENGAN CONFIDENCE RENDAH
        # ==========================================
        results = self.model.predict(
                    source=cv_image,
                    device="cpu",
                    conf=0.10,  # <-- Turunkan batas ke 5% untuk memaksa deteksi
                    verbose=False
                )

        # Siapkan wadah pesan ZED ObjectsStamped
        out_msg = ObjectsStamped()
        out_msg.header = rgb_msg.header

        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            
            for box in boxes:
                # Ekstraksi Bounding Box, Confidence, dan Class
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = box.conf[0].item()
                cls_id = int(box.cls[0].item())
                label_str = results[0].names[cls_id]

                w = x2 - x1
                h = y2 - y1
                center_x = x1 + w / 2.0
                center_y = y1 + h / 2.0

                # Dapatkan kedalaman dari depth map Gazebo
                depth = self.calculate_depth(depth_image, x1, y1, w, h)

                if depth is None:
                    continue

                # ==========================================
                # Kalkulasi Posisi 3D Relatif Kamera (X, Y, Z)
                # ==========================================
                X_cam = (center_x - self.cx) * depth / self.fx
                Y_cam = (center_y - self.cy) * depth / self.fy
                Z_cam = depth

                # ==========================================
                # Konstruksi Objek ZED
                # ==========================================
                obj = Object()
                obj.label = label_str
                obj.label_id = cls_id
                obj.confidence = float(conf * 100.0) 
                obj.position = [X_cam, Y_cam, Z_cam]

                # Bounding Box 2D Corners
                corners = [
                    [int(round(x1)), int(round(y1))],
                    [int(round(x2)), int(round(y1))],
                    [int(round(x2)), int(round(y2))],
                    [int(round(x1)), int(round(y2))]
                ]
                
                obj.bounding_box_2d = BoundingBox2Di()
                for pt in corners:
                    kp = Keypoint2Di()
                    kp.kp = pt
                    obj.bounding_box_2d.corners.append(kp)

                out_msg.objects.append(obj)

                # ==========================================
                # FITUR LIVE STREAMING VISUALISASI OPENCV
                # ==========================================
                pt1 = (int(x1), int(y1))
                pt2 = (int(x2), int(y2))
                
                # Gambar Kotak Bounding Box (Warna Hijau)
                cv2.rectangle(cv_image, pt1, pt2, (0, 255, 0), 2)
                
                # Format Teks Label + Confidence + Jarak (Depth)
                text_label = f"{label_str} {conf*100:.1f}% | {depth:.2f}m"
                
                # Tampilkan teks di atas kotak
                cv2.putText(
                    cv_image, 
                    text_label, 
                    (int(x1), max(int(y1) - 10, 20)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    (0, 255, 255), 
                    2
                )

        # Publikasikan ke topik AI ZED
        self.pub.publish(out_msg)

        # Tampilkan Jendela Live Streaming OpenCV
        cv2.imshow("YOLO Gazebo Live Stream (ZED Emulator)", cv_image)
        cv2.waitKey(1)
        self.get_logger().info("SYNC CALLBACK")

def main(args=None):
    rclpy.init(args=args)
    node = YoloGazeboDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()