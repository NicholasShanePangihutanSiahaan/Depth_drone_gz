#!/usr/bin/env python3

import math
import statistics
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

from uav_interfaces.msg import Tree
from uav_interfaces.msg import TreeArray
from pcl_cstm_msg.msg import TrackedCylinderArray
from beehive_drone.mission_params import MissionConfig
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy

class TreeMapper(Node):

    def __init__(self):
        super().__init__("tree_mapper")

        ##################################################
        # Parameters
        ##################################################
        self.frame_id = "odom"

        # maksimum jarak agar dianggap pohon yang sama
        self.merge_distance = MissionConfig.TREE_MERGE_DISTANCE

        # confidence model
        self.max_confidence = MissionConfig.TREE_MAX_CONFIDENCE
        self.new_tree_confidence = MissionConfig.TREE_NEW_CONFIDENCE
        self.confidence_increment = MissionConfig.TREE_CONFIDENCE_INCREMENT
        self.confidence_decay = MissionConfig.TREE_CONFIDENCE_DECAY

        # waktu hilang sebelum confidence turun
        self.timeout = MissionConfig.TREE_TIMEOUT
        self.unvalidated_retention = MissionConfig.TREE_UNVALIDATED_RETENTION

        ##################################################
        # Database
        ##################################################
        self.tree_database = {}
        self.next_tree_id = 1
        self.pcl_track_to_tree = {}
        self.position_window = MissionConfig.TREE_POSITION_WINDOW
        self.track_reassociate_distance = (
            MissionConfig.TREE_TRACK_REASSOCIATE_DISTANCE
        )

        ##################################################
        # Subscribers
        ##################################################
        # hasil deteksi perception
        self.sub = self.create_subscription(
            Point,
            "/perception/tree_position_camera",
            self.tree_callback,
            10
        )

        # Cylinder PCL sudah berada pada frame global dan tidak perlu
        # diproyeksikan lagi lewat pixel kamera.
        self.min_pcl_seen_count = self.declare_parameter(
            "min_pcl_seen_count", 3
        ).value
        pcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pcl_sub = self.create_subscription(
            TrackedCylinderArray,
            "/perception/tracked_trees",
            self.pcl_tree_callback,
            pcl_qos
        )

        # hasil inspeksi
        self.create_subscription(
            Tree,
            "/map/tree_update",
            self.tree_update_callback,
            10
        )

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        ##################################################
        # Publishers
        ##################################################
        self.tree_pub = self.create_publisher(
            TreeArray,
            "/map/trees",
            map_qos
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/tree_markers",
            10
        )

        # Event ini hanya terbit ketika PCL benar-benar melihat cylinder pada
        # frame terbaru. FSM memakainya untuk verifikasi ulang sebelum orbit.
        self.observation_pub = self.create_publisher(
            Tree,
            "/perception/tree_observation",
            10
        )

        ##################################################
        # Timer
        ##################################################
        self.timer = self.create_timer(
            5.0,
            self.update_confidence
        )

        self.get_logger().info("Tree Mapper Started")


    def pcl_tree_callback(self, msg):
        """Masukkan cylinder PCL yang stabil ke database pohon lama."""
        accepted = 0
        # Satu array berasal dari satu siklus pemrosesan PCL. Jangan menerbitkan
        # beberapa cylinder yang tergabung ke map ID sama sebagai beberapa
        # observasi waktu yang berbeda untuk verifikasi pra-orbit.
        frame_observations = {}
        for tracked in msg.cylinders:
            cylinder = tracked.cylinder
            if (
                not cylinder.is_valid
                or tracked.missed_count != 0
                or tracked.seen_count < self.min_pcl_seen_count
            ):
                continue

            position = cylinder.pose.position
            point = Point()
            point.x = position.x
            point.y = position.y
            point.z = position.z
            preferred_id = self.pcl_track_to_tree.get(tracked.id)
            if preferred_id not in self.tree_database:
                preferred_id = None
            tree_id = self.tree_callback(point, preferred_id=preferred_id)
            if tree_id is None:
                continue
            self.pcl_track_to_tree[tracked.id] = tree_id

            previous = frame_observations.get(tree_id)
            if previous is None or cylinder.confidence > previous[0]:
                # Simpan koordinat cylinder saat ini, bukan running-average
                # database. FSM memerlukan pengukuran baru untuk mengoreksi
                # pusat pohon yang mungkin sudah bergeser di peta.
                frame_observations[tree_id] = (
                    cylinder.confidence,
                    position.x,
                    position.y,
                    position.z,
                )
            accepted += 1

        for tree_id, (confidence, x, y, z) in frame_observations.items():
            mapped = self.tree_database[tree_id]
            observation = Tree()
            observation.id = tree_id
            observation.x = x
            observation.y = y
            observation.z = z
            observation.confidence = confidence
            observation.inspected = mapped["inspected"]
            self.observation_pub.publish(observation)

        if accepted:
            self.get_logger().debug(
                f"Accepted {accepted} tracked PCL tree(s)"
            )


    ##################################################
    # Receive tree detection
    ##################################################
    def tree_callback(self, msg, preferred_id=None):
        x = msg.x
        y = msg.y
        z = msg.z

        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            self.get_logger().warning("Invalid tree position ignored")
            return None

        nearest_id = None
        nearest_distance = float("inf")

        if preferred_id in self.tree_database:
            preferred = self.tree_database[preferred_id]
            preferred_distance = math.hypot(x - preferred["x"], y - preferred["y"])
            if preferred_distance <= self.track_reassociate_distance:
                nearest_id = preferred_id
                nearest_distance = preferred_distance
            else:
                self.get_logger().warning(
                    f"Lompatan track PCL ditolak untuk tree {preferred_id}: "
                    f"{preferred_distance:.2f} m"
                )
                return None

        ##################################################
        # Search nearest tree
        ##################################################
        for tree_id,tree in self.tree_database.items():
            if nearest_id is not None:
                break
            distance = math.sqrt(
                (x-tree["x"])**2 +
                (y-tree["y"])**2
            )

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_id = tree_id

        ##################################################
        # Existing tree
        ##################################################
        if nearest_id is not None and nearest_distance < self.merge_distance:
            tree = self.tree_database[nearest_id]
            tree["count"] += 1
            tree["observations"].append((x, y, z))
            del tree["observations"][:-self.position_window]

            # Median window rejects a transient bad cylinder and, unlike a
            # lifetime average, can still converge as the drone gets closer.
            tree["x"] = statistics.median(p[0] for p in tree["observations"])
            tree["y"] = statistics.median(p[1] for p in tree["observations"])
            tree["z"] = statistics.median(p[2] for p in tree["observations"])

            ##################################################
            # Confidence hanya naik jika belum inspected
            ##################################################
            if not tree["inspected"]:
                tree["confidence"] = min(
                    tree["confidence"] + self.confidence_increment,
                    self.max_confidence
                )

            tree["last_seen"] = time.time()
            self.get_logger().debug(f"Tree {nearest_id} updated")

        ##################################################
        # New tree
        ##################################################
        else:
            tree_id = self.next_tree_id
            self.tree_database[tree_id] = {
                "id":tree_id,
                "x":x,
                "y":y,
                "z":z,
                "confidence": self.new_tree_confidence,
                "count":1,
                "inspected":False,
                "validated":False,
                "observations":[(x, y, z)],
                "last_seen": time.time()
            }
            self.next_tree_id += 1
            self.get_logger().info(f"New tree {tree_id} ({x:.2f},{y:.2f},{z:.2f})")

        self.publish_tree()
        self.publish_marker()
        return nearest_id if nearest_id is not None and nearest_distance < self.merge_distance else tree_id


    ##################################################
    # Receive inspection result
    ##################################################
    def tree_update_callback(self,msg):
        
        ##################################################
        # 1. CEK SINYAL PEMUSNAHAN DARI FSM
        ##################################################
        if msg.confidence == -1.0:
            if msg.id in self.tree_database:
                del self.tree_database[msg.id]
                self.pcl_track_to_tree = {
                    track_id: tree_id
                    for track_id, tree_id in self.pcl_track_to_tree.items()
                    if tree_id != msg.id
                }
                self.get_logger().info(f"Pohon Hantu ID:{msg.id} resmi DIHAPUS dari database peta.")
                
                # Update visualisasi untuk segera membuang pohon yang dihapus
                self.publish_tree()
                self.publish_marker()
            return
        ##################################################

        ##################################################
        # 2. LOGIKA UPDATE NORMAL
        ##################################################
        if msg.id not in self.tree_database:
            self.get_logger().warning(f"Unknown tree ID {msg.id}")
            return

        tree = self.tree_database[msg.id]

        # Update full information
        tree["x"] = msg.x
        tree["y"] = msg.y
        tree["z"] = msg.z
        tree["confidence"] = msg.confidence
        tree["inspected"] = msg.inspected
        tree["validated"] = msg.validated
        tree["observations"] = [(msg.x, msg.y, msg.z)]
        tree["last_seen"] = time.time()

        self.get_logger().info(f"Tree {msg.id} inspected={msg.inspected}")

        self.publish_tree()
        self.publish_marker()


    ##################################################
    # Confidence aging
    ##################################################
    def update_confidence(self):
        now = time.time()

        expired_ids = []
        for tree_id, tree in self.tree_database.items():
            ##################################################
            # Jangan decay pohon selesai inspeksi
            ##################################################
            if tree["inspected"]:
                continue

            elapsed = now - tree["last_seen"]

            if not tree["validated"] and elapsed > self.unvalidated_retention:
                expired_ids.append(tree_id)
                continue

            if elapsed > self.timeout:
                tree["confidence"] -= self.confidence_decay
                if tree["confidence"] < 0:
                    tree["confidence"] = 0.0

        for tree_id in expired_ids:
            del self.tree_database[tree_id]
        if expired_ids:
            self.pcl_track_to_tree = {
                track_id: tree_id
                for track_id, tree_id in self.pcl_track_to_tree.items()
                if tree_id not in expired_ids
            }
            self.get_logger().info(
                f"Menghapus {len(expired_ids)} kandidat PCL yang sudah stale"
            )

        self.publish_tree()
        self.publish_marker()


    ##################################################
    # Publish TreeArray
    ##################################################
    def publish_tree(self):
        msg = TreeArray()

        for tree in self.tree_database.values():
            t = Tree()
            t.id = tree["id"]
            t.x = tree["x"]
            t.y = tree["y"]
            t.z = tree["z"]
            t.confidence = tree["confidence"]
            t.inspected = tree["inspected"]
            t.validated = tree["validated"]

            msg.trees.append(t)

        self.tree_pub.publish(msg)


    ##################################################
    # RVIZ Marker
    ##################################################
    def publish_marker(self):
        markers = MarkerArray()

        # Membersihkan teks lama di RVIZ sebelum mempublikasikan ulang
        # Ini mencegah teks pohon hantu tertinggal di layar setelah dihapus
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        markers.markers.append(delete_all_marker)

        ##################################################
        # Sphere marker
        ##################################################
        sphere = Marker()
        sphere.header.frame_id = self.frame_id
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = "trees"
        sphere.id = 0
        sphere.type = Marker.SPHERE_LIST
        sphere.action = Marker.ADD

        sphere.scale.x = 0.5
        sphere.scale.y = 0.5
        sphere.scale.z = 0.5
        sphere.pose.orientation.w = 1.0

        for tree in self.tree_database.values():
            p = Point()
            p.x = tree["x"]
            p.y = tree["y"]
            p.z = tree["z"]
            sphere.points.append(p)

        markers.markers.append(sphere)

        ##################################################
        # Text marker
        ##################################################
        marker_id = 1000

        for tree in self.tree_database.values():
            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "tree_id"
            text.id = marker_id
            marker_id += 1

            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = tree["x"]
            text.pose.position.y = tree["y"]
            text.pose.position.z = 1.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.5

            status = "DONE" if tree["inspected"] else "NEW"
            text.text = f"ID:{tree['id']} C:{tree['confidence']:.2f} {status}"

            markers.markers.append(text)

        self.marker_pub.publish(markers)

##################################################
def main(args=None):
    rclpy.init(args=args)
    node = TreeMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
