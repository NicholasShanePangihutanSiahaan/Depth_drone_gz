#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String
from uav_interfaces.msg import Tree, TreeArray

from beehive_drone.math_utils import distance_2d, quaternion_from_yaw, wrap_pi, yaw_from_quaternion
from beehive_drone.mission_params import MissionConfig


class MissionStateMachine(Node):
    """Mission coordinator for row exploration, tree approach, orbit and landing."""

    def __init__(self) -> None:
        super().__init__("mission_state_machine")

        defaults = {
            "world_frame": MissionConfig.WORLD_FRAME,
            "flight_mode": MissionConfig.FLIGHT_MODE,
            "flight_altitude": MissionConfig.FLIGHT_ALTITUDE,
            "prestream_sec": MissionConfig.PRESTREAM_SEC,
            "takeoff_timeout_sec": MissionConfig.TAKEOFF_TIMEOUT_SEC,
            "takeoff_progress_check_sec": MissionConfig.TAKEOFF_PROGRESS_CHECK_SEC,
            "min_takeoff_progress": MissionConfig.MIN_TAKEOFF_PROGRESS,
            "land_retry_sec": MissionConfig.LAND_RETRY_SEC,
            "hold_after_takeoff": False,
            "explore_step": MissionConfig.EXPLORE_STEP,
            "crab_step": MissionConfig.CRAB_STEP,
            "end_of_row_dist": MissionConfig.END_OF_ROW_DIST,
            "end_of_farm_dist": MissionConfig.END_OF_FARM_DIST,
            "tree_search_radius": MissionConfig.TREE_SEARCH_RADIUS,
            "tree_min_confidence": MissionConfig.TREE_MIN_CONFIDENCE,
            "verify_duration_sec": MissionConfig.VERIFY_DURATION_SEC,
            "verify_position_tolerance": MissionConfig.VERIFY_POSITION_TOLERANCE,
            "orbit_radius": MissionConfig.ORBIT_RADIUS,
            "orbit_start_tolerance": MissionConfig.ORBIT_START_TOLERANCE,
            "orbit_timeout_sec": MissionConfig.ORBIT_TIMEOUT_SEC,
            "command_retry_sec": MissionConfig.COMMAND_RETRY_SEC,
            "pose_timeout_sec": MissionConfig.POSE_TIMEOUT_SEC,
            "map_timeout_sec": MissionConfig.MAP_TIMEOUT_SEC,
            "land_complete_altitude": MissionConfig.LAND_COMPLETE_ALTITUDE,
            "home_reached_tolerance": MissionConfig.HOME_REACHED_TOLERANCE,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.flight_mode = str(self.get_parameter("flight_mode").value)
        self.flight_altitude = float(self.get_parameter("flight_altitude").value)
        self.prestream_sec = float(self.get_parameter("prestream_sec").value)
        self.takeoff_timeout_sec = float(self.get_parameter("takeoff_timeout_sec").value)
        self.takeoff_progress_check_sec = float(
            self.get_parameter("takeoff_progress_check_sec").value
        )
        self.min_takeoff_progress = float(
            self.get_parameter("min_takeoff_progress").value
        )
        self.land_retry_sec = float(self.get_parameter("land_retry_sec").value)
        self.hold_after_takeoff = bool(
            self.get_parameter("hold_after_takeoff").value
        )
        self.explore_step = float(self.get_parameter("explore_step").value)
        self.crab_step = float(self.get_parameter("crab_step").value)
        self.end_of_row_dist = float(self.get_parameter("end_of_row_dist").value)
        self.end_of_farm_dist = float(self.get_parameter("end_of_farm_dist").value)
        self.tree_search_radius = float(self.get_parameter("tree_search_radius").value)
        self.tree_min_confidence = float(self.get_parameter("tree_min_confidence").value)
        self.verify_duration_sec = float(self.get_parameter("verify_duration_sec").value)
        self.verify_position_tolerance = float(
            self.get_parameter("verify_position_tolerance").value
        )
        self.orbit_radius = float(self.get_parameter("orbit_radius").value)
        self.orbit_start_tolerance = float(
            self.get_parameter("orbit_start_tolerance").value
        )
        self.orbit_timeout_sec = float(self.get_parameter("orbit_timeout_sec").value)
        self.command_retry_sec = float(self.get_parameter("command_retry_sec").value)
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.map_timeout_sec = float(self.get_parameter("map_timeout_sec").value)
        self.land_complete_altitude = float(
            self.get_parameter("land_complete_altitude").value
        )
        self.home_reached_tolerance = float(
            self.get_parameter("home_reached_tolerance").value
        )

        self.state = "WAIT_CONNECTION"
        self.state_enter_time = self.now_sec()
        self.last_command_time = -1e9
        self.current_pose: Optional[PoseStamped] = None
        self.last_pose_time: Optional[float] = None
        self.last_map_time: Optional[float] = None
        self.trees = []
        self.target_tree: Optional[Tree] = None
        self.target_reference = None
        self.target_verify_failures = 0
        self.verify_start_time: Optional[float] = None
        self.orbit_prepare_start: Optional[float] = None
        self.orbit_start_time: Optional[float] = None
        self.orbit_status = "IDLE"

        self.connected = False
        self.armed = False
        self.current_mode = ""
        self.hovering = False
        self.altitude = 0.0

        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.takeoff_target_z = self.flight_altitude
        self.takeoff_start_z = 0.0
        self.takeoff_start_time: Optional[float] = None
        self.takeoff_command_sent = False
        self.last_land_command_time = -1e9
        self.home_captured = False
        self.explore_dir_x = 1.0
        self.explore_dir_y = 1.0
        self.last_tree_or_row_x = 0.0
        self.last_tree_or_row_y = 0.0
        self.crab_start_y = 0.0
        self.final_spin_last_yaw = 0.0
        self.final_spin_angle = 0.0

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_callback, qos_sensor
        )
        self.create_subscription(TreeArray, "/map/trees", self.tree_callback, qos_map)
        self.create_subscription(
            String, "/control/orbit_status", self.orbit_status_callback, 10
        )
        self.create_subscription(
            Bool, "/flight/telemetry/is_connected", self.connected_callback, 10
        )
        self.create_subscription(
            Bool, "/flight/telemetry/is_armed", self.armed_callback, 10
        )
        self.create_subscription(
            String, "/flight/telemetry/current_mode", self.mode_callback, 10
        )
        self.create_subscription(
            Bool, "/flight/telemetry/is_hovering", self.hover_callback, 10
        )
        self.create_subscription(
            Float32, "/flight/telemetry/altitude", self.altitude_callback, 10
        )

        self.mode_pub = self.create_publisher(String, "/flight/cmd/set_mode", 10)
        self.arm_pub = self.create_publisher(Bool, "/flight/cmd/set_arm", 10)
        self.takeoff_pub = self.create_publisher(
            Float32, "/flight/cmd/takeoff", 10
        )
        self.target_altitude_pub = self.create_publisher(
            Float32, "/flight/target_altitude", 10
        )
        self.land_pub = self.create_publisher(Bool, "/flight/cmd/land", 10)
        self.local_goal_pub = self.create_publisher(
            PoseStamped, "/navigation/local_goal", 10
        )
        self.orbit_target_pub = self.create_publisher(
            Point, "/control/orbit_target", 10
        )
        self.orbit_start_pub = self.create_publisher(
            Bool, "/control/orbit_start", 10
        )
        self.active_tree_pub = self.create_publisher(
            Int32, "/control/active_tree_id", 10
        )
        self.tree_update_pub = self.create_publisher(Tree, "/map/tree_update", 10)
        self.fsm_state_pub = self.create_publisher(
            String, "/mission/fsm_state", 10
        )
        self.mission_status_pub = self.create_publisher(
            String, "/mission/status", 10
        )

        self.create_timer(0.1, self.loop)
        self.get_logger().info("Mission State Machine revisi aktif.")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def transition(self, new_state: str, reason: str = "") -> None:
        if new_state == self.state:
            return
        old = self.state
        self.state = new_state
        self.state_enter_time = self.now_sec()
        self.last_command_time = -1e9
        if new_state == "TAKEOFF":
            self.takeoff_start_time = self.state_enter_time
            self.takeoff_start_z = self.altitude
            self.takeoff_command_sent = False
        if new_state == "LAND":
            self.last_land_command_time = -1e9
        text = f"{old} -> {new_state}"
        if reason:
            text += f" | {reason}"
        self.get_logger().info(text)

    def pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg
        self.last_pose_time = self.now_sec()
        if not self.home_captured:
            self.home_x = float(msg.pose.position.x)
            self.home_y = float(msg.pose.position.y)
            self.home_z = float(msg.pose.position.z)
            self.takeoff_target_z = self.home_z + self.flight_altitude
            self.last_tree_or_row_x = self.home_x
            self.last_tree_or_row_y = self.home_y
            self.home_captured = True

    def tree_callback(self, msg: TreeArray) -> None:
        self.trees = list(msg.trees)
        self.last_map_time = self.now_sec()

    def orbit_status_callback(self, msg: String) -> None:
        self.orbit_status = msg.data

    def connected_callback(self, msg: Bool) -> None:
        self.connected = bool(msg.data)

    def armed_callback(self, msg: Bool) -> None:
        self.armed = bool(msg.data)

    def mode_callback(self, msg: String) -> None:
        self.current_mode = msg.data

    def hover_callback(self, msg: Bool) -> None:
        self.hovering = bool(msg.data)

    def altitude_callback(self, msg: Float32) -> None:
        self.altitude = float(msg.data)

    def pose_fresh(self) -> bool:
        return (
            self.last_pose_time is not None
            and self.now_sec() - self.last_pose_time <= self.pose_timeout_sec
        )

    def map_fresh(self) -> bool:
        return (
            self.last_map_time is not None
            and self.now_sec() - self.last_map_time <= self.map_timeout_sec
        )

    def publish_state(self) -> None:
        msg = String()
        msg.data = self.state
        self.fsm_state_pub.publish(msg)
        self.mission_status_pub.publish(msg)

    def publish_goal(self, x: float, y: float, yaw: float, z: Optional[float] = None) -> None:
        goal = PoseStamped()
        goal.header.frame_id = self.world_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(self.flight_altitude if z is None else z)
        goal.pose.orientation = quaternion_from_yaw(yaw)
        self.local_goal_pub.publish(goal)

    def publish_takeoff_goal(self) -> None:
        """Continuously command a vertical climb above the captured home pose."""
        self.publish_goal(
            self.home_x,
            self.home_y,
            0.0,
            z=self.takeoff_target_z,
        )
        target = Float32()
        target.data = float(self.takeoff_target_z)
        self.target_altitude_pub.publish(target)

    def send_bool(self, publisher, value: bool) -> None:
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def command_due(self) -> bool:
        now = self.now_sec()
        if now - self.last_command_time >= self.command_retry_sec:
            self.last_command_time = now
            return True
        return False

    @staticmethod
    def is_valid_tree(tree: Tree, min_confidence: float) -> bool:
        if tree.inspected or tree.confidence < min_confidence:
            return False
        if hasattr(tree, "validated") and not bool(tree.validated):
            return False
        return all(math.isfinite(float(v)) for v in (tree.x, tree.y, tree.z))

    def find_tree_by_id(self, tree_id: int) -> Optional[Tree]:
        return next((tree for tree in self.trees if int(tree.id) == tree_id), None)

    def find_uninspected_tree(self, require_ahead: bool = True) -> Optional[Tree]:
        if self.current_pose is None or not self.map_fresh():
            return None
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        candidates = []
        for tree in self.trees:
            if not self.is_valid_tree(tree, self.tree_min_confidence):
                continue
            distance = distance_2d(cx, cy, tree.x, tree.y)
            if distance > self.tree_search_radius:
                continue
            if require_ahead and (tree.x - cx) * self.explore_dir_x < -1.0:
                continue
            candidates.append((distance, -float(tree.confidence), int(tree.id), tree))
        return min(candidates, default=(None, None, None, None))[3]

    def set_active_tree(self, tree_id: int) -> None:
        msg = Int32()
        msg.data = int(tree_id)
        self.active_tree_pub.publish(msg)

    def clear_orbit(self) -> None:
        self.send_bool(self.orbit_start_pub, False)
        self.set_active_tree(-1)
        self.orbit_start_time = None
        self.orbit_prepare_start = None
        self.orbit_status = "IDLE"

    def loop(self) -> None:
        self.publish_state()

        if self.current_pose is None or not self.pose_fresh():
            return

        if self.armed and not self.connected and self.state not in {"LAND", "WAIT_LANDED", "DONE"}:
            self.clear_orbit()
            self.transition("LAND", "Koneksi MAVROS hilang saat armed")

        cx = float(self.current_pose.pose.position.x)
        cy = float(self.current_pose.pose.position.y)
        now = self.now_sec()

        if self.state == "WAIT_CONNECTION":
            if self.connected and self.home_captured:
                # ArduPilot GUIDED tidak membutuhkan PRESTREAM seperti PX4.
                self.transition("SET_MODE", "MAVROS dan local pose tersedia")

        elif self.state == "PRESTREAM":
            # Dipertahankan hanya untuk kompatibilitas state lama.
            # Tidak boleh mengirim target gerak sebelum takeoff.
            self.transition("SET_MODE", "Lewati prestream untuk ArduPilot")

        elif self.state == "SET_MODE":
            if self.current_mode == self.flight_mode:
                self.transition("ARM")
            elif self.command_due():
                msg = String()
                msg.data = self.flight_mode
                self.mode_pub.publish(msg)

        elif self.state == "ARM":
            # Jangan kirim target posisi/kecepatan ketika menunggu arming.
            if self.armed:
                self.transition("TAKEOFF", "Motor armed; mulai climb dengan velocity setpoint")
            elif self.command_due():
                self.send_bool(self.arm_pub, True)

        elif self.state == "TAKEOFF":
            # Saat TAKEOFF, controller internal ArduPilot menjadi satu-satunya
            # pengendali climb. Jangan kirim position/velocity setpoint.
            #
            # Target altitude ini hanya dipakai FlightManager untuk mendeteksi
            # hover, bukan dikirim sebagai perintah gerak ke MAVROS.
            altitude_target = Float32()
            altitude_target.data = float(self.takeoff_target_z)
            self.target_altitude_pub.publish(altitude_target)

            if not self.takeoff_command_sent:
                command = Float32()
                command.data = float(self.flight_altitude)
                self.takeoff_pub.publish(command)
                self.takeoff_command_sent = True
                self.get_logger().info(
                    f"NAV_TAKEOFF dikirim satu kali: {self.flight_altitude:.1f} m"
                )

            if not self.armed and now - self.state_enter_time > 2.0:
                self.get_logger().error("Autopilot disarm saat takeoff.")
                self.transition("ABORTED", "Unexpected disarm")
                return

            if self.hovering:
                self.last_tree_or_row_x = cx
                self.last_tree_or_row_y = cy
                if self.hold_after_takeoff:
                    self.transition("HOLD", "Hover stabil; flight test berhasil")
                else:
                    self.transition("EXPLORE_ROW", "Hover stabil")
                return

            elapsed = now - self.state_enter_time
            climb = self.altitude - self.takeoff_start_z
            if (
                elapsed >= self.takeoff_progress_check_sec
                and climb < self.min_takeoff_progress
            ):
                self.get_logger().error(
                    "TAKEOFF belum menunjukkan progres ketinggian. "
                    "NAV_TAKEOFF telah dikirim tanpa velocity override."
                )
                self.transition("LAND", "Tidak ada progres ketinggian")
                return

            if elapsed >= self.takeoff_timeout_sec:
                self.get_logger().error("TAKEOFF timeout; mission dibatalkan.")
                self.transition("LAND", "Takeoff timeout")
                return

        elif self.state == "HOLD":
            self.publish_takeoff_goal()

        elif self.state == "EXPLORE_ROW":
            tree = self.find_uninspected_tree(require_ahead=True)
            if tree is not None:
                self.target_tree = tree
                self.target_reference = (float(tree.x), float(tree.y))
                self.target_verify_failures = 0
                self.set_active_tree(int(tree.id))
                self.transition("APPROACH_TREE", f"Target ID={tree.id}")
                return

            yaw = 0.0 if self.explore_dir_x > 0.0 else math.pi
            self.publish_goal(cx + self.explore_step * self.explore_dir_x, cy, yaw)
            if distance_2d(cx, cy, self.last_tree_or_row_x, self.last_tree_or_row_y) >= self.end_of_row_dist:
                self.crab_start_y = cy
                self.transition("CRAB_SCAN", "Tidak ada pohon baru di ujung lorong")

        elif self.state == "APPROACH_TREE":
            if self.target_tree is None:
                self.transition("EXPLORE_ROW", "Target kosong")
                return
            latest = self.find_tree_by_id(int(self.target_tree.id))
            if latest is None:
                self.set_active_tree(-1)
                self.target_tree = None
                self.transition("EXPLORE_ROW", "Target hilang dari peta")
                return
            self.target_tree = latest

            dx = cx - latest.x
            dy = cy - latest.y
            radius = math.hypot(dx, dy)
            if radius < 0.1:
                dx = -self.explore_dir_x
                dy = 0.0
                radius = 1.0
            stop_x = latest.x + self.orbit_radius * dx / radius
            stop_y = latest.y + self.orbit_radius * dy / radius
            yaw = math.atan2(latest.y - cy, latest.x - cx)
            self.publish_goal(stop_x, stop_y, yaw)

            if distance_2d(cx, cy, stop_x, stop_y) <= 0.35:
                self.verify_start_time = now
                self.target_reference = (float(latest.x), float(latest.y))
                self.transition("VERIFY_TREE", "Posisi awal orbit tercapai")

        elif self.state == "VERIFY_TREE":
            if self.target_tree is None:
                self.transition("EXPLORE_ROW")
                return
            latest = self.find_tree_by_id(int(self.target_tree.id))
            if latest is None:
                self.set_active_tree(-1)
                self.target_tree = None
                self.transition("EXPLORE_ROW", "Target tidak lagi terdeteksi")
                return

            yaw = math.atan2(latest.y - cy, latest.x - cx)
            self.publish_goal(cx, cy, yaw)
            if self.verify_start_time is None or now - self.verify_start_time < self.verify_duration_sec:
                return

            drift = distance_2d(
                latest.x,
                latest.y,
                self.target_reference[0],
                self.target_reference[1],
            )
            radius_error = abs(distance_2d(cx, cy, latest.x, latest.y) - self.orbit_radius)
            if (
                drift <= self.verify_position_tolerance
                and radius_error <= self.orbit_start_tolerance
                and self.is_valid_tree(latest, self.tree_min_confidence)
            ):
                self.target_tree = latest
                self.orbit_prepare_start = now
                self.transition("PREPARE_ORBIT", f"Target ID={latest.id} tervalidasi")
            else:
                self.target_verify_failures += 1
                self.target_tree = latest
                if self.target_verify_failures >= 3:
                    reject = Tree()
                    reject.id = int(latest.id)
                    reject.confidence = -1.0
                    self.tree_update_pub.publish(reject)
                    self.set_active_tree(-1)
                    self.target_tree = None
                    self.transition("EXPLORE_ROW", "Target tidak stabil setelah 3 verifikasi")
                else:
                    self.transition("APPROACH_TREE", "Posisi target bergeser; pendekatan ulang")

        elif self.state == "PREPARE_ORBIT":
            if self.target_tree is None:
                self.clear_orbit()
                self.transition("EXPLORE_ROW")
                return
            point = Point()
            point.x = float(self.target_tree.x)
            point.y = float(self.target_tree.y)
            point.z = float(self.target_tree.z)
            self.orbit_target_pub.publish(point)
            self.set_active_tree(int(self.target_tree.id))

            if self.orbit_prepare_start is None:
                self.orbit_prepare_start = now
            if now - self.orbit_prepare_start >= 0.5:
                self.send_bool(self.orbit_start_pub, True)
                self.orbit_start_time = now
                self.transition("WAIT_ORBIT")

        elif self.state == "WAIT_ORBIT":
            if self.orbit_status == "ORBIT_COMPLETED":
                completed = self.target_tree
                self.clear_orbit()
                if completed is not None:
                    update = Tree()
                    update.id = int(completed.id)
                    update.x = float(completed.x)
                    update.y = float(completed.y)
                    update.z = float(completed.z)
                    update.confidence = float(completed.confidence)
                    update.inspected = True
                    if hasattr(update, "validated"):
                        update.validated = True
                    self.tree_update_pub.publish(update)
                    self.last_tree_or_row_x = float(completed.x)
                    self.last_tree_or_row_y = float(completed.y)
                self.target_tree = None
                self.transition("EXPLORE_ROW", "Orbit selesai")
            elif self.orbit_status in {"ORBIT_FAILED", "ORBIT_TIMEOUT"} or (
                self.orbit_start_time is not None
                and now - self.orbit_start_time > self.orbit_timeout_sec
            ):
                self.clear_orbit()
                self.target_tree = None
                self.transition("EXPLORE_ROW", "Orbit gagal/timeout")

        elif self.state == "CRAB_SCAN":
            yaw = 0.0 if self.explore_dir_x > 0.0 else math.pi
            self.publish_goal(cx, cy + self.crab_step * self.explore_dir_y, yaw)

            tree = self.find_uninspected_tree(require_ahead=False)
            if tree is not None:
                self.explore_dir_x *= -1.0
                self.last_tree_or_row_x = cx
                self.last_tree_or_row_y = cy
                self.target_tree = tree
                self.target_reference = (float(tree.x), float(tree.y))
                self.set_active_tree(int(tree.id))
                self.transition("APPROACH_TREE", "Pohon ditemukan di lorong berikutnya")
            elif abs(cy - self.crab_start_y) >= self.end_of_farm_dist:
                self.transition("RETURN_HOME", "Batas pencarian lateral tercapai")

        elif self.state == "RETURN_HOME":
            yaw = math.atan2(self.home_y - cy, self.home_x - cx)
            self.publish_goal(self.home_x, self.home_y, yaw)
            if distance_2d(cx, cy, self.home_x, self.home_y) <= self.home_reached_tolerance:
                self.final_spin_last_yaw = yaw_from_quaternion(
                    self.current_pose.pose.orientation
                )
                self.final_spin_angle = 0.0
                self.transition("FINAL_SCAN")

        elif self.state == "FINAL_SCAN":
            tree = self.find_uninspected_tree(require_ahead=False)
            if tree is not None:
                self.target_tree = tree
                self.target_reference = (float(tree.x), float(tree.y))
                self.set_active_tree(int(tree.id))
                self.transition("APPROACH_TREE", "Pohon terlewat ditemukan")
                return

            current_yaw = yaw_from_quaternion(self.current_pose.pose.orientation)
            delta = wrap_pi(current_yaw - self.final_spin_last_yaw)
            if abs(delta) < 0.5:
                self.final_spin_angle += abs(delta)
            self.final_spin_last_yaw = current_yaw
            self.publish_goal(cx, cy, current_yaw + 0.25)
            if self.final_spin_angle >= 2.0 * math.pi:
                self.transition("LAND", "Final scan selesai")

        elif self.state == "LAND":
            self.clear_orbit()
            if not self.armed:
                self.transition("DONE", "Sudah disarm")
                return
            if now - self.last_land_command_time >= self.land_retry_sec:
                self.last_land_command_time = now
                self.send_bool(self.land_pub, True)
            self.transition("WAIT_LANDED")

        elif self.state == "WAIT_LANDED":
            if self.altitude <= self.home_z + self.land_complete_altitude or not self.armed:
                self.transition("DONE", "Pendaratan selesai")
            elif now - self.last_land_command_time >= self.land_retry_sec:
                self.last_land_command_time = now
                self.send_bool(self.land_pub, True)

        elif self.state == "ABORTED":
            self.clear_orbit()

        elif self.state == "DONE":
            self.clear_orbit()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.clear_orbit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
