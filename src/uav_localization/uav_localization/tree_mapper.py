#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class TreeMapper(Node):

    def __init__(self):

        super().__init__("tree_mapper")

        # Jarak maksimum agar dianggap pohon yang sama
        self.distance_threshold = 1.5

        self.tree_database = []

        self.next_tree_id = 1

        self.create_subscription(
            Point,
            "/perception/tree_target",
            self.tree_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseArray,
            "/map/tree_locations",
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/tree_markers",
            10
        )

        self.get_logger().info("Tree Mapper Started")

    ###############################################################

    def tree_callback(self, msg):

        x = msg.x
        y = msg.y

        current_time = self.get_clock().now().nanoseconds / 1e9

        nearest_tree = None
        nearest_distance = 9999.0

        for tree in self.tree_database:

            d = math.hypot(
                x - tree["x"],
                y - tree["y"]
            )

            if d < nearest_distance:
                nearest_distance = d
                nearest_tree = tree

        # =====================================================
        # Update pohon lama
        # =====================================================

        if nearest_tree is not None and nearest_distance < self.distance_threshold:

            nearest_tree["count"] += 1

            alpha = 1.0 / nearest_tree["count"]

            nearest_tree["x"] += alpha * (x - nearest_tree["x"])
            nearest_tree["y"] += alpha * (y - nearest_tree["y"])

            nearest_tree["last_seen"] = current_time

            self.publish_pose_array()
            self.publish_markers()

            return

        # =====================================================
        # Pohon baru
        # =====================================================

        tree = {

            "id": self.next_tree_id,
            "x": x,
            "y": y,
            "count": 1,
            "last_seen": current_time

        }

        self.tree_database.append(tree)

        self.next_tree_id += 1

        self.publish_pose_array()
        self.publish_markers()

        self.get_logger().info(
            f"Tree {tree['id']} added "
            f"({tree['x']:.2f}, {tree['y']:.2f})"
        )

    ###############################################################

    def publish_pose_array(self):

        msg = PoseArray()

        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()

        for tree in self.tree_database:

            pose = Pose()

            pose.position.x = tree["x"]
            pose.position.y = tree["y"]
            pose.position.z = 0.0

            pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.pose_pub.publish(msg)

    ###############################################################

    def publish_markers(self):

        marker_array = MarkerArray()

        # =======================================================
        # Marker pohon
        # =======================================================

        sphere = Marker()

        sphere.header.frame_id = "odom"
        sphere.header.stamp = self.get_clock().now().to_msg()

        sphere.ns = "trees"
        sphere.id = 0

        sphere.type = Marker.SPHERE_LIST
        sphere.action = Marker.ADD

        sphere.scale.x = 0.6
        sphere.scale.y = 0.6
        sphere.scale.z = 0.6

        sphere.pose.orientation.w = 1.0

        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 1.0

        for tree in self.tree_database:

            p = Point()

            p.x = tree["x"]
            p.y = tree["y"]
            p.z = 0.0

            sphere.points.append(p)

        marker_array.markers.append(sphere)

        # =======================================================
        # Nomor pohon
        # =======================================================

        marker_id = 1000

        for tree in self.tree_database:

            text = Marker()

            text.header.frame_id = "odom"
            text.header.stamp = self.get_clock().now().to_msg()

            text.ns = "tree_id"
            text.id = marker_id

            marker_id += 1

            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = tree["x"]
            text.pose.position.y = tree["y"]
            text.pose.position.z = 1.2

            text.pose.orientation.w = 1.0

            text.scale.z = 0.5

            confidence = min(tree["count"] / 5.0, 1.0)

            text.color.r = 1.0 - confidence
            text.color.g = confidence
            text.color.b = 0.0
            text.color.a = 1.0

            text.text = f"{tree['id']} ({tree['count']})"

            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

###############################################################

def main(args=None):

    rclpy.init(args=args)

    node = TreeMapper()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()