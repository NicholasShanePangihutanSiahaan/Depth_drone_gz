#!/usr/bin/env python3

import math
import struct
from copy import deepcopy
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32, String
from uav_interfaces.msg import Tree, TreeArray

from beehive_drone.math_utils import distance_2d, quaternion_from_yaw, yaw_from_quaternion
from beehive_drone.mission_params import MissionConfig


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MissionStateMachine(Node):
    """Single-tree mission with reverse return path and pilot takeover safety."""

    def __init__(self) -> None:
        super().__init__("mission_state_machine")

        defaults = {
            "world_frame": MissionConfig.WORLD_FRAME,
            "flight_mode": MissionConfig.FLIGHT_MODE,
            "flight_altitude": MissionConfig.FLIGHT_ALTITUDE,
            "takeoff_timeout_sec": MissionConfig.TAKEOFF_TIMEOUT_SEC,
            "takeoff_progress_check_sec": MissionConfig.TAKEOFF_PROGRESS_CHECK_SEC,
            "min_takeoff_progress": MissionConfig.MIN_TAKEOFF_PROGRESS,
            "land_retry_sec": MissionConfig.LAND_RETRY_SEC,
            "hold_after_takeoff": False,
            "tree_search_radius": MissionConfig.TREE_SEARCH_RADIUS,
            "tree_min_confidence": MissionConfig.TREE_MIN_CONFIDENCE,
            "mission_mode": "single",
            "target_forward_cone_deg": MissionConfig.TARGET_FORWARD_CONE_DEG,
            "verify_duration_sec": MissionConfig.VERIFY_DURATION_SEC,
            "verify_position_tolerance": MissionConfig.VERIFY_POSITION_TOLERANCE,
            "orbit_obstacle_clearance": MissionConfig.ORBIT_OBSTACLE_CLEARANCE,
            "active_tree_alias_radius": MissionConfig.TREE_ASSOCIATION_DISTANCE,
            "scan_duration_sec": 18.0,
            "scan_yaw_rate": 0.25,
            "map_settle_sec": 3.0,
            "approach_distance": MissionConfig.APPROACH_DISTANCE,
            "approach_tolerance": MissionConfig.APPROACH_TOLERANCE,
            "orbit_radius": MissionConfig.ORBIT_RADIUS,
            "orbit_start_tolerance": MissionConfig.ORBIT_START_TOLERANCE,
            "pre_orbit_position_tolerance": 0.45,
            "depth_topic": "/zed2i/depth/depth_registered",
            "depth_center_roi_px": 50,
            "depth_target_tolerance": 0.65,
            "depth_timeout_sec": 1.0,
            "orbit_timeout_sec": MissionConfig.ORBIT_TIMEOUT_SEC,
            "pre_orbit_hover_sec": MissionConfig.PRE_ORBIT_HOVER_SEC,
            "post_orbit_hover_sec": MissionConfig.POST_ORBIT_HOVER_SEC,
            "return_hover_sec": MissionConfig.RETURN_HOVER_SEC,
            "home_hover_sec": MissionConfig.HOME_HOVER_SEC,
            "hover_wait_timeout_sec": MissionConfig.HOVER_WAIT_TIMEOUT_SEC,
            "command_retry_sec": MissionConfig.COMMAND_RETRY_SEC,
            "map_timeout_sec": MissionConfig.MAP_TIMEOUT_SEC,
            "require_tree_map": MissionConfig.REQUIRE_TREE_MAP,
            "map_startup_timeout_sec": MissionConfig.MAP_STARTUP_TIMEOUT_SEC,
            "map_loss_grace_sec": MissionConfig.MAP_LOSS_GRACE_SEC,
            "land_complete_altitude": MissionConfig.LAND_COMPLETE_ALTITUDE,
            "home_reached_tolerance": MissionConfig.HOME_REACHED_TOLERANCE,
            "enable_rc_takeover": MissionConfig.ENABLE_RC_TAKEOVER,
            "rc_takeover_confirm_sec": MissionConfig.RC_TAKEOVER_CONFIRM_SEC,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.flight_mode = str(self.get_parameter("flight_mode").value)
        self.flight_altitude = float(self.get_parameter("flight_altitude").value)
        self.takeoff_timeout_sec = float(self.get_parameter("takeoff_timeout_sec").value)
        self.takeoff_progress_check_sec = float(
            self.get_parameter("takeoff_progress_check_sec").value
        )
        self.min_takeoff_progress = float(
            self.get_parameter("min_takeoff_progress").value
        )
        self.land_retry_sec = float(self.get_parameter("land_retry_sec").value)
        self.hold_after_takeoff = bool(self.get_parameter("hold_after_takeoff").value)
        self.tree_search_radius = float(self.get_parameter("tree_search_radius").value)
        self.tree_min_confidence = float(self.get_parameter("tree_min_confidence").value)
        self.mission_mode = str(self.get_parameter("mission_mode").value).strip().lower()
        if self.mission_mode not in {"single", "all"}:
            raise ValueError("mission_mode harus 'single' atau 'all'")
        self.target_forward_cone = math.radians(
            float(self.get_parameter("target_forward_cone_deg").value)
        )
        self.verify_duration_sec = max(0.5, float(self.get_parameter("verify_duration_sec").value))
        self.verify_position_tolerance = max(0.1, float(self.get_parameter("verify_position_tolerance").value))
        self.orbit_obstacle_clearance = max(0.5, float(self.get_parameter("orbit_obstacle_clearance").value))
        self.active_tree_alias_radius = max(
            0.5, float(self.get_parameter("active_tree_alias_radius").value)
        )
        self.scan_duration_sec = max(2.0, float(self.get_parameter("scan_duration_sec").value))
        self.scan_yaw_rate = max(0.05, float(self.get_parameter("scan_yaw_rate").value))
        self.map_settle_sec = max(0.0, float(self.get_parameter("map_settle_sec").value))
        self.approach_distance = float(self.get_parameter("approach_distance").value)
        self.approach_tolerance = float(self.get_parameter("approach_tolerance").value)
        self.orbit_radius = float(self.get_parameter("orbit_radius").value)
        self.orbit_start_tolerance = float(
            self.get_parameter("orbit_start_tolerance").value
        )
        self.pre_orbit_position_tolerance = max(
            self.approach_tolerance,
            float(self.get_parameter("pre_orbit_position_tolerance").value),
        )
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.depth_center_roi_px = max(
            4, int(self.get_parameter("depth_center_roi_px").value)
        )
        self.depth_target_tolerance = max(
            0.1, float(self.get_parameter("depth_target_tolerance").value)
        )
        self.depth_timeout_sec = max(
            0.1, float(self.get_parameter("depth_timeout_sec").value)
        )
        self.orbit_timeout_sec = float(self.get_parameter("orbit_timeout_sec").value)
        self.pre_orbit_hover_sec = float(self.get_parameter("pre_orbit_hover_sec").value)
        self.post_orbit_hover_sec = float(self.get_parameter("post_orbit_hover_sec").value)
        self.return_hover_sec = float(self.get_parameter("return_hover_sec").value)
        self.home_hover_sec = float(self.get_parameter("home_hover_sec").value)
        self.hover_wait_timeout_sec = float(
            self.get_parameter("hover_wait_timeout_sec").value
        )
        self.command_retry_sec = float(self.get_parameter("command_retry_sec").value)
        self.map_timeout_sec = float(self.get_parameter("map_timeout_sec").value)
        self.require_tree_map = bool(self.get_parameter("require_tree_map").value)
        self.map_startup_timeout_sec = float(
            self.get_parameter("map_startup_timeout_sec").value
        )
        self.map_loss_grace_sec = float(
            self.get_parameter("map_loss_grace_sec").value
        )
        self.land_complete_altitude = float(
            self.get_parameter("land_complete_altitude").value
        )
        self.home_reached_tolerance = float(
            self.get_parameter("home_reached_tolerance").value
        )
        self.enable_rc_takeover = bool(
            self.get_parameter("enable_rc_takeover").value
        )
        self.rc_takeover_confirm_sec = max(
            0.0, float(self.get_parameter("rc_takeover_confirm_sec").value)
        )

        if self.approach_distance <= 0.0:
            raise ValueError("approach_distance harus lebih besar dari 0")
        if abs(self.approach_distance - self.orbit_radius) > 0.05:
            self.get_logger().warning(
                "approach_distance dan orbit_radius berbeda. Orbit controller akan "
                "melakukan penyesuaian radial sebelum mulai mengorbit."
            )

        self.state = "WAIT_HOME"
        self.state_enter_time = self.now_sec()
        self.last_command_time = -1e9
        self.last_land_command_time = -1e9

        self.current_pose: Optional[PoseStamped] = None
        self.last_map_time: Optional[float] = None
        self.map_ready = False
        self.map_not_ready_since: Optional[float] = None
        self.trees = []
        self.map_id_signature: Tuple[int, ...] = tuple()
        self.map_structure_change_time = self.now_sec()

        self.armed = False
        self.current_mode = ""
        self.hovering = False
        self.altitude = 0.0

        self.home_captured = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.home_yaw = 0.0
        self.takeoff_target_z = self.flight_altitude
        self.takeoff_start_z = 0.0
        self.takeoff_command_sent = False
        self.unexpected_disarm_since: Optional[float] = None
        self.mode_mismatch_since: Optional[float] = None
        self.pilot_override_latched = False

        self.target_tree: Optional[Tree] = None
        self.pre_orbit_point: Optional[Tuple[float, float, float]] = None
        self.post_orbit_hold_point: Optional[Tuple[float, float, float, float]] = None
        self.return_path_samples: list[Tuple[float, float, float, float]] = []
        self.return_retrace_index: Optional[int] = None
        self.orbit_prepare_start: Optional[float] = None
        self.orbit_start_time: Optional[float] = None
        self.orbit_status = "IDLE"
        self.orbit_succeeded = False
        self.target_observations: list[Tuple[float, float, float]] = []
        self.scan_anchor: Optional[Tuple[float, float, float]] = None
        self.failed_tree_ids: set[int] = set()
        self.completed_tree_ids: set[int] = set()
        self.cross_check_complete = False
        self.pre_orbit_stable_since: Optional[float] = None
        self.last_search_diagnostic_time = -1e9
        self.center_depth_m: Optional[float] = None
        self.last_depth_time: Optional[float] = None

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
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_sensor)
        self.create_subscription(TreeArray, "/map/trees", self.tree_callback, qos_map)
        self.create_subscription(Bool, "/map/trees_ready", self.map_ready_callback, qos_map)
        self.create_subscription(String, "/control/orbit_status", self.orbit_status_callback, 10)
        self.create_subscription(Bool, "/flight/telemetry/is_armed", self.armed_callback, 10)
        self.create_subscription(String, "/flight/telemetry/current_mode", self.mode_callback, 10)
        self.create_subscription(Bool, "/flight/telemetry/is_hovering", self.hover_callback, 10)
        self.create_subscription(Float32, "/flight/telemetry/altitude", self.altitude_callback, 10)

        # ==========================================
        # Variabel State & Navigasi
        # ==========================================
        self.state = "INIT"
        self.retry_counter = 0
        self.hover_timer = 0
        self.orbit_status = "IDLE"
        self.current_pose = None
        self.home_pose = None
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_yaw = 0.0
        self.trees = []
        self.target_tree = None

        # Variabel Telemetri Penerbangan (Dari Flight Manager)
        self.is_armed = False
        self.current_mode = ""
        self.is_hovering = False

        self.explore_dir_x = 1.0          
        self.explore_dir_y = 1.0          
        
        self.last_tree_x = 0.0
        self.last_tree_y = 0.0
        self.crab_start_y = 0.0
        
        self.spin_accumulated = 0.0
        self.last_yaw = 0.0

        # ==========================================
        # Subscriber
        # ==========================================
        self.pose_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_cb, qos_sensor)
        self.orbit_status_sub = self.create_subscription(String, "/control/orbit_status", self.orbit_status_cb, 10)
        self.tree_sub = self.create_subscription(TreeArray, "/map/trees", self.tree_cb, qos_map)

        # Telemetri dari Flight Manager
        self.telemetry_arm_sub = self.create_subscription(Bool, "/flight/telemetry/is_armed", self.arm_cb, 10)
        self.telemetry_mode_sub = self.create_subscription(String, "/flight/telemetry/current_mode", self.mode_cb, 10)
        self.telemetry_hover_sub = self.create_subscription(Bool, "/flight/telemetry/is_hovering", self.hover_cb, 10)

        # ==========================================
        # Publisher
        # ==========================================
        # Command ke Flight Manager
        self.cmd_mode_pub = self.create_publisher(String, "/flight/cmd/set_mode", 10)
        self.cmd_arm_pub = self.create_publisher(Bool, "/flight/cmd/set_arm", 10)
        self.cmd_takeoff_pub = self.create_publisher(Float32, "/flight/cmd/takeoff", 10)
        self.cmd_land_pub = self.create_publisher(Bool, "/flight/cmd/land", 10)
        
        # Command ke Dynamic Orbit Controller
        self.orbit_start_pub = self.create_publisher(Bool, "/control/orbit_start", 10)
        self.orbit_target_pub = self.create_publisher(Point, "/control/orbit_target", 10)
        
        # Command navigasi lokal
        self.local_goal_pub = self.create_publisher(PoseStamped, "/navigation/local_goal", 10)
        self.orbit_target_pub = self.create_publisher(Point, "/control/orbit_target", 10)
        self.orbit_start_pub = self.create_publisher(Bool, "/control/orbit_start", 10)
        self.active_tree_pub = self.create_publisher(Int32, "/control/active_tree_id", 10)
        self.tree_update_pub = self.create_publisher(Tree, "/map/tree_update", 10)
        self.fsm_state_pub = self.create_publisher(String, "/mission/fsm_state", 10)
        self.mission_status_pub = self.create_publisher(String, "/mission/status", 10)
        self.autonomy_enabled_pub = self.create_publisher(
            Bool, "/mission/autonomy_enabled", 10
        )
        self.pilot_override_pub = self.create_publisher(
            Bool, "/mission/pilot_override", 10
        )

        self.create_timer(0.1, self.loop)
        if self.mission_mode == "all":
            self.get_logger().info(
                "Mission multi-pohon aktif: orbit semua target valid, scan, retrace home, land."
            )
        else:
            self.get_logger().info(
                "Mission satu pohon aktif: takeoff -> 1 orbit -> kembali home -> land."
            )

    # --- Callbacks Sensor & Status ---
    def pose_cb(self, msg):
        self.current_pose = msg
        # Pose pertama sebelum takeoff menjadi home dinamis. Salin nilainya,
        # jangan simpan referensi message yang akan terus diperbarui.
        if self.home_pose is None and self.state in ("INIT", "WAIT_GUIDED", "WAIT_ARM"):
            self.home_pose = (
                msg.pose.position.x,
                msg.pose.position.y,
                quaternion_to_yaw(msg.pose.orientation)
            )
    def orbit_status_cb(self, msg): self.orbit_status = msg.data
    def tree_cb(self, msg): self.trees = msg.trees
    
    # --- Callbacks Telemetri ---
    def arm_cb(self, msg): self.is_armed = msg.data
    def mode_cb(self, msg): self.current_mode = msg.data
    def hover_cb(self, msg): self.is_hovering = msg.data

    def transition(self, new_state: str, reason: str = "") -> None:
        if new_state == self.state:
            return
        old_state = self.state
        self.state = new_state
        self.state_enter_time = self.now_sec()
        self.last_command_time = -1e9
        if new_state == "TAKEOFF":
            self.takeoff_start_z = self.altitude
            self.takeoff_command_sent = False
            self.unexpected_disarm_since = None
            self.return_path_samples = []
            self.return_retrace_index = None
        if new_state == "RETRACE_HOME":
            self.return_retrace_index = None
        if new_state == "HOVER_BEFORE_ORBIT":
            self.cross_check_complete = False
            self.pre_orbit_stable_since = None
        if new_state == "LAND":
            self.last_land_command_time = -1e9
        text = f"{old_state} -> {new_state}"
        if reason:
            text += f" | {reason}"
        self.get_logger().info(text)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def current_yaw(self):
        return quaternion_to_yaw(self.current_pose.pose.orientation)

    def find_uninspected_tree(self):
        if self.current_pose is None: return None
        cx, cy = self.current_pose.pose.position.x, self.current_pose.pose.position.y
        
        best_tree = None
        min_dist = float('inf')

    def depth_callback(self, msg: Image) -> None:
        """Read only a center ROI; PCL remains responsible for tree detection."""
        if msg.width == 0 or msg.height == 0 or msg.step == 0:
            return
        encoding = msg.encoding.upper()
        if encoding in {"32FC1", "TYPE_32FC1"}:
            fmt, size, scale = "<f", 4, 1.0
        elif encoding in {"16UC1", "MONO16", "TYPE_16UC1"}:
            fmt, size, scale = "<H", 2, 0.001
        else:
            return
        half = self.depth_center_roi_px // 2
        x0 = max(0, int(msg.width) // 2 - half)
        x1 = min(int(msg.width), int(msg.width) // 2 + half)
        y0 = max(0, int(msg.height) // 2 - half)
        y1 = min(int(msg.height), int(msg.height) // 2 + half)
        values = []
        data = memoryview(msg.data)
        for y in range(y0, y1):
            row = y * int(msg.step)
            for x in range(x0, x1):
                offset = row + x * size
                if offset + size > len(data):
                    continue
                value = float(struct.unpack_from(fmt, data, offset)[0]) * scale
                if math.isfinite(value) and 0.2 <= value <= 20.0:
                    values.append(value)
        if values:
            values.sort()
            # The trunk can occupy less than half of the ROI; the median then
            # selects background foliage. A robust near-surface percentile
            # represents the centered trunk without using a single noisy pixel.
            self.center_depth_m = values[max(0, len(values) // 5)]
            self.last_depth_time = self.now_sec()

    def depth_target_ready(self, expected_distance: float) -> bool:
        return bool(
            self.center_depth_m is not None
            and self.last_depth_time is not None
            and self.now_sec() - self.last_depth_time <= self.depth_timeout_sec
            and abs(self.center_depth_m - expected_distance) <= self.depth_target_tolerance
        )

    def tree_callback(self, msg: TreeArray) -> None:
        self.trees = list(msg.trees)
        self.last_map_time = self.now_sec()
        signature = tuple(sorted(int(tree.id) for tree in self.trees if tree.validated))
        if signature != self.map_id_signature:
            self.map_id_signature = signature
            self.map_structure_change_time = self.last_map_time

    def map_ready_callback(self, msg: Bool) -> None:
        self.map_ready = bool(msg.data)

    def orbit_status_callback(self, msg: String) -> None:
        self.orbit_status = msg.data

    def armed_callback(self, msg: Bool) -> None:
        self.armed = bool(msg.data)

    def mode_callback(self, msg: String) -> None:
        self.current_mode = msg.data

    def hover_callback(self, msg: Bool) -> None:
        self.hovering = bool(msg.data)

    def altitude_callback(self, msg: Float32) -> None:
        self.altitude = float(msg.data)

    def map_fresh(self) -> bool:
        return (
            self.map_ready
            and bool(self.trees)
            and self.last_map_time is not None
            and self.now_sec() - self.last_map_time <= self.map_timeout_sec
        )

    def publish_state(self) -> None:
        msg = String()
        msg.data = self.state
        self.fsm_state_pub.publish(msg)
        self.mission_status_pub.publish(msg)

    def publish_target_altitude(self) -> None:
        target = Float32()
        target.data = float(self.takeoff_target_z)
        self.target_altitude_pub.publish(target)

    @staticmethod
    def autonomous_states() -> set[str]:
        """States in which companion-computer flight targets are allowed."""
        return {
            "TAKEOFF",
            "HOLD",
            "SEARCH_TREE",
            "VERIFY_TARGET", "APPROACH_TREE", "SCAN_FOR_TREE",
            "HOVER_BEFORE_ORBIT",
            "PREPARE_ORBIT",
            "WAIT_ORBIT",
            "POST_ORBIT_HOVER",
            "RETURN_PRE_ORBIT",
            "HOVER_AT_PRE_ORBIT",
            "RETRACE_HOME",
            "RETURN_HOME",
            "HOME_HOVER",
        }

    @classmethod
    def takeover_monitor_states(cls) -> set[str]:
        """Armed states where an RC mode change must cancel automation."""
        return cls.autonomous_states() | {"ARM", "LAND", "WAIT_LANDED", "ABORTED"}

    def publish_control_authority(self, autonomy_enabled: bool) -> None:
        # Publish the takeover latch first so downstream controllers can label
        # and process the following authority drop correctly.
        override = Bool()
        override.data = bool(self.pilot_override_latched)
        self.pilot_override_pub.publish(override)

        autonomy = Bool()
        autonomy.data = bool(autonomy_enabled and not self.pilot_override_latched)
        self.autonomy_enabled_pub.publish(autonomy)

    def enter_pilot_override(self, observed_mode: str) -> None:
        if self.pilot_override_latched:
            return
        self.pilot_override_latched = True
        self.mode_mismatch_since = None
        self.stop_orbit(clear_active_tree=True)
        self.target_tree = None
        self.post_orbit_hold_point = None
        self.publish_control_authority(False)
        self.transition(
            "PILOT_OVERRIDE",
            f"Pilot takeover: mode {self.flight_mode} -> {observed_mode}; "
            "kontrol otomatis dinonaktifkan sampai node direstart",
        )

    def update_control_authority(self, now: float) -> bool:
        """Return True after a pilot takeover has been latched."""
        current_mode = self.current_mode.strip().upper()
        expected_mode = self.flight_mode.strip().upper()

        if self.pilot_override_latched or self.state == "PILOT_OVERRIDE":
            self.pilot_override_latched = True
            self.publish_control_authority(False)
            return True

        allowed_modes = {expected_mode}
        # A LAND mode entered after our own landing command is not a takeover.
        # Any other RC-selected mode still hands authority to the pilot and
        # prevents further automatic landing retries.
        if self.state in {"LAND", "WAIT_LANDED", "ABORTED"}:
            allowed_modes.add("LAND")

        autonomous = (
            self.armed
            and self.state in self.autonomous_states()
            and bool(current_mode)
            and current_mode == expected_mode
        )

        mismatch = (
            self.enable_rc_takeover
            and self.armed
            and self.state in self.takeover_monitor_states()
            and bool(current_mode)
            and current_mode not in allowed_modes
        )
        if mismatch:
            if self.mode_mismatch_since is None:
                self.mode_mismatch_since = now
                self.get_logger().warning(
                    f"Mode berubah dari {expected_mode} ke {current_mode}; "
                    "kontrol otomatis dihentikan sambil mengonfirmasi pilot takeover."
                )
            elif now - self.mode_mismatch_since >= self.rc_takeover_confirm_sec:
                self.enter_pilot_override(current_mode)
                return True

            # Stop FSM commands immediately on the first mismatched mode sample.
            # The debounce only decides whether the takeover becomes permanent.
            self.publish_control_authority(False)
            return True

        self.mode_mismatch_since = None
        self.publish_control_authority(autonomous)
        return False

    def publish_goal(self, x: float, y: float, yaw: float, z: Optional[float] = None) -> None:
        goal = PoseStamped()
        goal.header.frame_id = self.world_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(self.takeoff_target_z if z is None else z)
        goal.pose.orientation = quaternion_from_yaw(yaw)
        self.local_goal_pub.publish(goal)
        self.publish_target_altitude()

    def publish_home_hover(self) -> None:
        self.publish_goal(self.home_x, self.home_y, self.home_yaw)

    def publish_hold_here(self) -> None:
        if self.current_pose is None:
            return
        yaw = yaw_from_quaternion(self.current_pose.pose.orientation)
        self.publish_goal(
            float(self.current_pose.pose.position.x),
            float(self.current_pose.pose.position.y),
            yaw,
            z=float(self.current_pose.pose.position.z),
        )

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
        return all(math.isfinite(float(value)) for value in (tree.x, tree.y, tree.z))

    @staticmethod
    def is_tree_visible(tree: Tree) -> bool:
        return bool(getattr(tree, "visible", False)) and float(
            getattr(tree, "observation_age", float("inf"))
        ) <= 1.5

    def find_tree_by_id(self, tree_id: int) -> Optional[Tree]:
        return next((tree for tree in self.trees if int(tree.id) == tree_id), None)

    def find_nearest_tree(self) -> Optional[Tree]:
        if self.current_pose is None or not self.map_fresh():
            return None
        cx = float(self.current_pose.pose.position.x)
        cy = float(self.current_pose.pose.position.y)
        yaw = yaw_from_quaternion(self.current_pose.pose.orientation)
        candidates = []
        rejected = {"invalid": 0, "far": 0, "side": 0, "orbit": 0, "failed": 0}
        for tree in self.trees:
            if int(tree.id) in self.failed_tree_ids or int(tree.id) in self.completed_tree_ids:
                rejected["failed"] += 1
                continue
            if not self.is_valid_tree(tree, self.tree_min_confidence):
                rejected["invalid"] += 1
                continue
            if not self.is_tree_visible(tree):
                rejected["invalid"] += 1
                continue
            distance = distance_2d(cx, cy, float(tree.x), float(tree.y))
            bearing = math.atan2(float(tree.y) - cy, float(tree.x) - cx)
            bearing_error = abs(math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw)))
            # The single-tree mission must select the tree actually in front of
            # the aircraft, not simply the nearest trunk at its right/left.
            if distance > self.tree_search_radius:
                rejected["far"] += 1
            elif self.mission_mode != "all" and bearing_error > self.target_forward_cone:
                rejected["side"] += 1
            else:
                # Lock the nearest currently visible trunk. Bearing only
                # breaks ties after the camera-FOV gate has been satisfied.
                candidates.append((distance, bearing_error, -float(tree.confidence), int(tree.id), tree))
        selected = min(candidates, default=(None, None, None, None, None))[4]
        now = self.now_sec()
        if selected is None and now - self.last_search_diagnostic_time >= 3.0:
            self.last_search_diagnostic_time = now
            self.get_logger().warning(
                "Tidak ada target lolos filter: "
                f"map={len(self.trees)}, invalid={rejected['invalid']}, "
                f"di_luar_radius={rejected['far']}, di_luar_kerucut_depan={rejected['side']}, "
                f"koridor_orbit_terhalang={rejected['orbit']}, pernah_gagal={rejected['failed']}."
            )
        return selected

    def orbit_path_clear(self, target: Tree) -> bool:
        """Reject an orbit whose circular corridor crosses another mapped trunk."""
        for obstacle in self.trees:
            if int(obstacle.id) == int(target.id) or not bool(obstacle.validated):
                continue
            center_distance = distance_2d(float(target.x), float(target.y), float(obstacle.x), float(obstacle.y))
            if center_distance <= self.active_tree_alias_radius:
                continue
            if abs(center_distance - self.orbit_radius) < self.orbit_obstacle_clearance:
                return False
        return True

    def lock_verified_target(self) -> bool:
        if self.target_tree is None or not self.target_observations:
            return False
        xs = [p[0] for p in self.target_observations]
        ys = [p[1] for p in self.target_observations]
        zs = [p[2] for p in self.target_observations]
        x, y, z = sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs)
        spread = max(math.hypot(px - x, py - y) for px, py, _ in self.target_observations)
        if spread > self.verify_position_tolerance:
            return False
        locked = deepcopy(self.target_tree)
        locked.x, locked.y, locked.z = float(x), float(y), float(z)
        self.target_tree = locked
        self.pre_orbit_point = self.compute_pre_orbit_point(locked)
        return True

    def set_active_tree(self, tree_id: int) -> None:
        msg = Int32()
        msg.data = int(tree_id)
        self.active_tree_pub.publish(msg)

    def stop_orbit(self, clear_active_tree: bool = True) -> None:
        self.send_bool(self.orbit_start_pub, False)
        if clear_active_tree:
            self.set_active_tree(-1)
        self.orbit_start_time = None
        self.orbit_prepare_start = None
        self.orbit_status = "IDLE"

    def compute_pre_orbit_point(self, tree: Tree) -> Tuple[float, float, float]:
        assert self.current_pose is not None
        cx = float(self.current_pose.pose.position.x)
        cy = float(self.current_pose.pose.position.y)
        dx = cx - float(tree.x)
        dy = cy - float(tree.y)
        distance = math.hypot(dx, dy)
        if distance < 0.1:
            dx, dy, distance = -1.0, 0.0, 1.0
        point_x = float(tree.x) + self.approach_distance * dx / distance
        point_y = float(tree.y) + self.approach_distance * dy / distance
        yaw = math.atan2(float(tree.y) - point_y, float(tree.x) - point_x)
        return point_x, point_y, yaw

    def hover_stage_complete(self, required_sec: float) -> bool:
        elapsed = self.now_sec() - self.state_enter_time
        if elapsed < required_sec:
            return False
        if self.hovering:
            return True
        if elapsed >= self.hover_wait_timeout_sec:
            if int(elapsed) % 5 == 0:
                self.get_logger().error(
                    f"Hover belum terkonfirmasi setelah {elapsed:.1f} detik; "
                    "misi ditahan untuk keselamatan."
                )
            return False
        return False

    def record_return_path_sample(self) -> None:
        if self.current_pose is None:
            return
        if self.state not in {
            "TAKEOFF",
            "SEARCH_TREE",
            "APPROACH_TREE",
            "HOVER_BEFORE_ORBIT",
            "PREPARE_ORBIT",
        }:
            return

        p = self.current_pose.pose.position
        sample = (
            float(p.x),
            float(p.y),
            float(p.z),
            yaw_from_quaternion(self.current_pose.pose.orientation),
        )
        if self.return_path_samples:
            last_x, last_y, last_z, _ = self.return_path_samples[-1]
            if (
                distance_2d(last_x, last_y, sample[0], sample[1]) < 0.15
                and abs(last_z - sample[2]) < 0.10
            ):
                return
        self.return_path_samples.append(sample)

    def mark_target_inspected(self) -> None:
        if self.target_tree is None:
            return
        self.completed_tree_ids.add(int(self.target_tree.id))
        update = Tree()
        update.id = int(self.target_tree.id)
        update.x = float(self.target_tree.x)
        update.y = float(self.target_tree.y)
        update.z = float(self.target_tree.z)
        update.confidence = float(self.target_tree.confidence)
        update.inspected = True
        if hasattr(update, "validated"):
            update.validated = True
        self.tree_update_pub.publish(update)

    def begin_safe_return(self, reason: str) -> None:
        self.stop_orbit(clear_active_tree=False)
        if self.pre_orbit_point is not None:
            self.transition("RETURN_PRE_ORBIT", reason)
        else:
            self.set_active_tree(-1)
            self.transition("RETURN_HOME", reason)

    def loop(self) -> None:
        self.publish_state()
        now = self.now_sec()

        # Remote takeover is checked before pose/map gates. Therefore the pilot
        # can take control even when VSLAM/PCL is stale or unavailable.
        if self.update_control_authority(now):
            if self.state == "PILOT_OVERRIDE" and not self.armed:
                self.transition("DONE", "Drone disarm setelah pilot takeover")
            return

        have_pose = self.current_pose is not None
        if have_pose:
            cx = float(self.current_pose.pose.position.x)
            cy = float(self.current_pose.pose.position.y)
            self.record_return_path_sample()
        else:
            cx = self.home_x
            cy = self.home_y

        tree_dependent_states = {
            "VERIFY_TARGET",
            "HOVER_BEFORE_ORBIT",
        }
        if self.require_tree_map and self.state in tree_dependent_states:
            if not have_pose:
                self.publish_home_hover()
                return
            if not self.map_fresh():
                if self.map_not_ready_since is None:
                    self.map_not_ready_since = now
                    self.get_logger().warning("Peta pohon stale; drone menahan posisi.")
                if self.state == "WAIT_ORBIT":
                    self.begin_safe_return("Peta hilang saat orbit; orbit dibatalkan")
                    return
                self.publish_hold_here()
                if now - self.map_not_ready_since >= self.map_loss_grace_sec:
                    self.begin_safe_return("Peta pohon tidak pulih")
                return
            self.map_not_ready_since = None

        if self.state == "WAIT_HOME":
            self.transition("SET_MODE", "Mulai sequence mode/arm tanpa menunggu pose")

        elif self.state == "SET_MODE":
            if self.current_mode == self.flight_mode:
                self.transition("ARM")
            elif self.command_due():
                msg = String()
                msg.data = self.flight_mode
                self.mode_pub.publish(msg)

        elif self.state == "ARM":
            if self.armed:
                self.transition("TAKEOFF", "Motor armed; menjalankan NAV_TAKEOFF one-shot")
            elif self.command_due():
                self.send_bool(self.arm_pub, True)

        elif self.state == "TAKEOFF":
            self.publish_target_altitude()

            if not self.takeoff_command_sent:
                command = Float32()
                command.data = float(self.flight_altitude)
                self.takeoff_pub.publish(command)
                self.takeoff_command_sent = True
                self.get_logger().info(
                    f"NAV_TAKEOFF dikirim satu kali: {self.flight_altitude:.1f} m"
                )

            if not self.armed:
                if self.unexpected_disarm_since is None:
                    self.unexpected_disarm_since = now
                    self.get_logger().warning(
                        "Autopilot melaporkan disarm saat TAKEOFF; menunggu konfirmasi 5 detik."
                    )
                elif now - self.unexpected_disarm_since >= 5.0:
                    self.transition("ABORTED", "Disarm valid selama TAKEOFF")
                    return
            else:
                self.unexpected_disarm_since = None

            if self.hovering:
                if self.hold_after_takeoff:
                    self.transition("HOLD", "Hover stabil; mode uji hold aktif")
                else:
                    self.transition("SEARCH_TREE", "Hover takeoff stabil; mencari satu pohon")
                return

        elif self.state == "HOLD":
            self.publish_home_hover()

        elif self.state == "SEARCH_TREE":
            if not have_pose:
                self.publish_home_hover()
                return
            # On a multi-tree mission search from the last safe pre-orbit
            # location; on the first/single mission this is the home hover.
            self.publish_hold_here()
            if now - self.map_structure_change_time < self.map_settle_sec:
                return
            tree = self.find_nearest_tree()
            if tree is not None:
                self.target_tree = deepcopy(tree)
                self.target_observations = []
                self.set_active_tree(int(tree.id))
                self.transition(
                    "VERIFY_TARGET",
                    f"Kandidat pohon ID={tree.id}; memverifikasi pusat batang",
                )
            elif now - self.state_enter_time >= self.map_startup_timeout_sec:
                if self.mission_mode == "all" and self.pre_orbit_point is not None:
                    self.scan_anchor = self.pre_orbit_point
                    self.transition("SCAN_FOR_TREE", "Tidak ada kandidat baru; scan 360 derajat")
                else:
                    self.transition("HOME_HOVER", "Pohon tidak ditemukan sebelum timeout")

        elif self.state == "VERIFY_TARGET":
            self.publish_hold_here()
            if self.target_tree is None:
                self.transition("SEARCH_TREE", "Kandidat kosong")
                return
            latest = self.find_tree_by_id(int(self.target_tree.id))
            if (
                latest is None
                or not self.is_valid_tree(latest, self.tree_min_confidence)
                or not self.is_tree_visible(latest)
            ):
                self.target_tree = None
                self.set_active_tree(-1)
                self.transition("SEARCH_TREE", "Kandidat hilang saat verifikasi")
                return
            self.target_tree.confidence = latest.confidence
            self.target_observations.append((float(latest.x), float(latest.y), float(latest.z)))
            if now - self.state_enter_time >= self.verify_duration_sec:
                if self.lock_verified_target():
                    self.transition(
                        "APPROACH_TREE",
                        f"Pusat pohon ID={self.target_tree.id} dikunci; approach {self.approach_distance:.1f} m",
                    )
                else:
                    self.target_tree = None
                    self.set_active_tree(-1)
                    self.transition("SEARCH_TREE", "Kandidat tidak stabil")

        elif self.state == "APPROACH_TREE":
            if not have_pose:
                self.publish_hold_here()
                return
            if self.target_tree is None or self.pre_orbit_point is None:
                self.begin_safe_return("Target/approach point tidak tersedia")
                return

            point_x, point_y, _ = self.pre_orbit_point
            yaw = math.atan2(float(self.target_tree.y) - cy, float(self.target_tree.x) - cx)
            self.publish_goal(point_x, point_y, yaw)
            if distance_2d(cx, cy, point_x, point_y) <= self.approach_tolerance:
                self.transition("HOVER_BEFORE_ORBIT", f"Titik {self.approach_distance:.1f} m sebelum pohon tercapai")

        elif self.state == "HOVER_BEFORE_ORBIT":
            if not have_pose:
                self.publish_hold_here()
                return
            if self.target_tree is None or self.pre_orbit_point is None:
                self.begin_safe_return("Target hilang sebelum orbit")
                return
            if not self.cross_check_complete:
                latest = self.find_tree_by_id(int(self.target_tree.id))
                if latest is None or not self.is_tree_visible(latest):
                    latest = min(
                        [tree for tree in self.trees if self.is_tree_visible(tree)],
                        key=lambda tree: distance_2d(
                            float(self.target_tree.x), float(self.target_tree.y),
                            float(tree.x), float(tree.y),
                        ),
                        default=None,
                    )
                    if latest is not None and distance_2d(
                        float(self.target_tree.x), float(self.target_tree.y),
                        float(latest.x), float(latest.y),
                    ) > self.active_tree_alias_radius:
                        latest = None
                if latest is None:
                    point_x, point_y, _ = self.pre_orbit_point
                    wait_yaw = math.atan2(
                        float(self.target_tree.y) - point_y,
                        float(self.target_tree.x) - point_x,
                    )
                    self.publish_goal(point_x, point_y, wait_yaw)
                    if self.depth_target_ready(self.approach_distance):
                        # At hover the vehicle is commanded to face the locked
                        # trunk. Reconstruct its center from the registered
                        # depth ray when close-range cylinder fitting drops it.
                        measured_yaw = yaw_from_quaternion(
                            self.current_pose.pose.orientation
                        )
                        corrected = deepcopy(self.target_tree)
                        corrected.x = float(
                            cx + self.center_depth_m * math.cos(measured_yaw)
                        )
                        corrected.y = float(
                            cy + self.center_depth_m * math.sin(measured_yaw)
                        )
                        latest = corrected
                        self.get_logger().info(
                            "Cross-check depth: pusat target dikoreksi dari ROI tengah, "
                            f"depth={self.center_depth_m:.2f} m."
                        )
                    else:
                        verify_wait = max(6.0, 3.0 * self.verify_duration_sec)
                        if now - self.state_enter_time >= verify_wait:
                            self.begin_safe_return(
                                "Target tidak terdeteksi selama hover cross-check; "
                                f"depth_tengah={self.center_depth_m}"
                            )
                        return

                center_shift = distance_2d(
                    float(self.target_tree.x), float(self.target_tree.y),
                    float(latest.x), float(latest.y),
                )
                if center_shift > self.active_tree_alias_radius:
                    self.set_active_tree(-1)
                    self.target_tree = None
                    self.pre_orbit_point = None
                    self.transition(
                        "SEARCH_TREE",
                        f"Cross-check gagal: pusat pohon bergeser {center_shift:.2f} m",
                    )
                    return
                if center_shift > self.verify_position_tolerance:
                    self.get_logger().warning(
                        "Cross-check mengoreksi pusat pohon terkunci sebesar "
                        f"{center_shift:.2f} m; ID target tetap dipertahankan."
                    )
                # Accept the final measured center exactly once, then freeze it.
                self.target_tree = deepcopy(latest)
                self.pre_orbit_point = self.compute_pre_orbit_point(self.target_tree)
                self.cross_check_complete = True

            point_x, point_y, _ = self.pre_orbit_point
            yaw = math.atan2(float(self.target_tree.y) - point_y, float(self.target_tree.x) - point_x)
            self.publish_goal(point_x, point_y, yaw)

            radius_error = abs(
                distance_2d(cx, cy, float(self.target_tree.x), float(self.target_tree.y))
                - self.orbit_radius
            )
            position_error = distance_2d(cx, cy, point_x, point_y)
            current_yaw = yaw_from_quaternion(self.current_pose.pose.orientation)
            yaw_error = abs(math.atan2(math.sin(yaw - current_yaw), math.cos(yaw - current_yaw)))
            geometry_ready = (
                radius_error <= self.orbit_start_tolerance
                and position_error <= self.pre_orbit_position_tolerance
                and yaw_error <= math.radians(12.0)
                and self.hovering
                and self.depth_target_ready(self.orbit_radius)
            )
            if (
                radius_error > self.orbit_start_tolerance
                or position_error > self.pre_orbit_position_tolerance
            ):
                self.transition(
                    "APPROACH_TREE",
                    "Menyesuaikan geometri sebelum orbit; "
                    f"radius_error={radius_error:.2f} m, posisi_error={position_error:.2f} m",
                )
            elif geometry_ready:
                if self.pre_orbit_stable_since is None:
                    self.pre_orbit_stable_since = now
                elif now - self.pre_orbit_stable_since >= self.pre_orbit_hover_sec:
                    self.get_logger().info(
                        "Siap orbit: radius aktual="
                        f"{distance_2d(cx, cy, float(self.target_tree.x), float(self.target_tree.y)):.2f} m, "
                        f"yaw_error={math.degrees(yaw_error):.1f} deg, "
                        f"depth_tengah={self.center_depth_m:.2f} m."
                    )
                    self.orbit_prepare_start = now
                    self.transition("PREPARE_ORBIT", "Posisi, radius, yaw, dan hover stabil")
            else:
                self.pre_orbit_stable_since = None

        elif self.state == "PREPARE_ORBIT":
            if not have_pose:
                self.publish_hold_here()
                return
            if self.target_tree is None:
                self.begin_safe_return("Target kosong saat persiapan orbit")
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
                self.transition("WAIT_ORBIT", "Memulai tepat satu putaran")

        elif self.state == "WAIT_ORBIT":
            if self.orbit_status == "ORBIT_COMPLETED":
                # 1. Matikan perintah orbit
                stop_msg = Bool(); stop_msg.data = False
                self.orbit_start_pub.publish(stop_msg)
                
                # 2. UPDATE MAPPER: Tandai pohon ini SUDAH DIINSPEKSI
                if self.target_tree is not None:
                    update_msg = Tree()
                    update_msg.id = self.target_tree.id
                    update_msg.x = self.target_tree.x
                    update_msg.y = self.target_tree.y
                    update_msg.z = self.target_tree.z
                    update_msg.confidence = self.target_tree.confidence
                    
                    # INI KUNCI UTAMANYA:
                    update_msg.inspected = True 
                    
                    self.tree_update_pub.publish(update_msg)
                    self.get_logger().info(f"Pohon ID:{self.target_tree.id} ditandai SELESAI (Inspected).")

                # Misi hanya menginspeksi satu pohon. Tahan posisi akhir orbit
                # sebelum menghadap dan kembali ke titik takeoff.
                self.hold_x = cx
                self.hold_y = cy
                self.hold_yaw = self.current_yaw()
                self.hover_timer = 0
                self.target_tree = None
                self.state = "POST_ORBIT_HOVER"
                self.get_logger().info(
                    "Orbit satu pohon selesai. Hover sebelum kembali ke titik takeoff."
                )

        elif self.state == "POST_ORBIT_HOVER":
            self.publish_goal(self.hold_x, self.hold_y, self.hold_yaw)
            self.hover_timer = self.hover_timer + 1 if self.is_hovering else 0

            required_ticks = int(MissionConfig.POST_ORBIT_HOVER_TIME / 0.1)
            if self.hover_timer >= required_ticks:
                self.hover_timer = 0
                self.state = "ALIGN_HOME"
                self.get_logger().info("Hover stabil. Menyesuaikan yaw menuju home.")

        elif self.state == "ALIGN_HOME":
            if self.home_pose is None:
                self.get_logger().error("Home belum tersimpan; menahan posisi untuk keselamatan.")
                self.publish_goal(self.hold_x, self.hold_y, self.hold_yaw)
                return

            home_x, home_y, _ = self.home_pose
            yaw_to_home = math.atan2(home_y - cy, home_x - cx)
            self.publish_goal(self.hold_x, self.hold_y, yaw_to_home)

            yaw_error = abs(self.normalize_angle(yaw_to_home - self.current_yaw()))
            aligned = yaw_error <= MissionConfig.HOME_YAW_TOLERANCE
            self.hover_timer = self.hover_timer + 1 if aligned and self.is_hovering else 0

            required_ticks = int(MissionConfig.HOME_ALIGN_TIME / 0.1)
            if self.hover_timer >= required_ticks:
                self.hover_timer = 0
                self.state = "RETURN_TO_HOME"
                self.get_logger().info("Arah ke home stabil. Mulai kembali ke titik takeoff.")

        elif self.state == "END_OF_ROW":
            retreat_x = self.last_tree_x - (self.approach_safe_dist * self.explore_dir_x)
            target_yaw = 0.0 if self.explore_dir_x > 0 else math.pi
            
            self.publish_goal(retreat_x, self.last_tree_y, target_yaw)
            
            if abs(cx - retreat_x) < 0.5:
                self.explore_dir_x *= -1.0 
                self.crab_start_y = cy
                self.state = "CRAB_SCAN"
                self.get_logger().info("Mundur selesai. Memulai Crab Scan 90 derajat.")

        elif self.state == "CRAB_SCAN":
            target_y = cy + (self.crab_speed * self.explore_dir_y)
            target_yaw = 0.0 if self.explore_dir_x > 0 else math.pi
            self.publish_goal(cx, target_y, target_yaw)
            
            self.target_tree = self.find_uninspected_tree()
            if self.target_tree is not None:
                self.last_tree_y = self.target_tree.y
                self.state = "APPROACH_TREE"
                self.get_logger().info("Lorong baru ditemukan!")
            else:
                if abs(cy - self.crab_start_y) > self.end_of_farm_dist:
                    self.state = "RETURN_TO_HOME"
                    self.get_logger().info("Lahan habis. Cari jalur untuk pulang (RTH).")

        elif self.state == "RETURN_TO_HOME":
            if self.home_pose is None:
                return

            home_x, home_y, home_yaw = self.home_pose
            target_yaw = math.atan2(home_y - cy, home_x - cx)
            self.publish_goal(home_x, home_y, target_yaw)

            if self.distance(cx, cy, home_x, home_y) < MissionConfig.HOME_POSITION_TOLERANCE:
                self.hold_yaw = home_yaw
                self.hover_timer = 0
                self.state = "HOME_HOVER"
                self.get_logger().info("Tiba di titik takeoff. Hover sebelum landing.")

        elif self.state == "HOME_HOVER":
            home_x, home_y, _ = self.home_pose
            self.publish_goal(home_x, home_y, self.hold_yaw)
            self.hover_timer = self.hover_timer + 1 if self.is_hovering else 0

            required_ticks = int(MissionConfig.HOME_HOVER_TIME / 0.1)
            if self.hover_timer >= required_ticks:
                self.state = "LANDING"
                self.get_logger().info("Hover home selesai. Memulai pendaratan.")

        elif self.state == "FINAL_SPIN":
            qx = self.current_pose.pose.orientation.x
            qy = self.current_pose.pose.orientation.y
            qz = self.current_pose.pose.orientation.z
            qw = self.current_pose.pose.orientation.w
            current_yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
            
            delta = current_yaw - self.last_yaw
            if delta > math.pi: delta -= 2 * math.pi
            elif delta < -math.pi: delta += 2 * math.pi
            
            self.spin_accumulated += abs(delta)
            self.last_yaw = current_yaw
            
            self.target_tree = self.find_uninspected_tree()
            if self.target_tree is not None:
                self.state = "APPROACH_TREE"
                self.get_logger().info("Pohon terlewat ditemukan saat Final Spin!")
                return
            if self.orbit_status == "ORBIT_COMPLETED":
                self.orbit_succeeded = True
                self.mark_target_inspected()
                self.stop_orbit(clear_active_tree=False)
                current_yaw = yaw_from_quaternion(self.current_pose.pose.orientation)
                self.post_orbit_hold_point = (cx, cy, self.takeoff_target_z, current_yaw)
                self.transition(
                    "POST_ORBIT_HOVER",
                    "Satu putaran 360 derajat selesai; menstabilkan drone",
                )
            elif self.orbit_status in {"ORBIT_FAILED", "ORBIT_TIMEOUT"} or (
                self.orbit_start_time is not None
                and now - self.orbit_start_time > self.orbit_timeout_sec
            ):
                self.orbit_succeeded = False
                if self.target_tree is not None:
                    self.failed_tree_ids.add(int(self.target_tree.id))
                self.stop_orbit(clear_active_tree=False)
                self.transition("RETURN_PRE_ORBIT", "Orbit gagal/timeout; kembali dengan aman")

        elif self.state == "POST_ORBIT_HOVER":
            if not have_pose:
                self.publish_hold_here()
                return
            if self.post_orbit_hold_point is None:
                current_yaw = yaw_from_quaternion(self.current_pose.pose.orientation)
                self.post_orbit_hold_point = (cx, cy, self.takeoff_target_z, current_yaw)
            hold_x, hold_y, hold_z, hold_yaw = self.post_orbit_hold_point
            self.publish_goal(hold_x, hold_y, hold_yaw, z=hold_z)
            if self.hover_stage_complete(self.post_orbit_hover_sec):
                self.set_active_tree(-1)
                if self.mission_mode == "all":
                    next_tree = self.find_nearest_tree()
                    if next_tree is not None:
                        self.target_tree = deepcopy(next_tree)
                        self.target_observations = []
                        self.set_active_tree(int(next_tree.id))
                        self.transition(
                            "VERIFY_TARGET",
                            f"Hover pasca-orbit stabil; mengunci kandidat berikut ID={next_tree.id}",
                        )
                    else:
                        self.target_tree = None
                        self.transition("SEARCH_TREE", "Hover pasca-orbit stabil; menunggu target baru")
                else:
                    self.target_tree = None
                    self.transition("RETRACE_HOME", "Hover pasca-orbit stabil; kembali home")

        elif self.state == "RETURN_PRE_ORBIT":
            if not have_pose:
                self.publish_home_hover()
                return
            if self.pre_orbit_point is None:
                self.set_active_tree(-1)
                self.transition("RETURN_HOME", "Titik sebelum orbit tidak tersedia")
                return
            point_x, point_y, point_yaw = self.pre_orbit_point
            self.publish_goal(point_x, point_y, point_yaw)
            if distance_2d(cx, cy, point_x, point_y) <= self.approach_tolerance:
                self.transition("HOVER_AT_PRE_ORBIT", "Kembali ke posisi sebelum orbit")

        elif self.state == "HOVER_AT_PRE_ORBIT":
            if not have_pose:
                self.publish_home_hover()
                return
            if self.pre_orbit_point is None:
                self.set_active_tree(-1)
                self.transition("RETURN_HOME")
                return
            point_x, point_y, point_yaw = self.pre_orbit_point
            self.publish_goal(point_x, point_y, point_yaw)
            if self.hover_stage_complete(self.return_hover_sec):
                self.set_active_tree(-1)
                self.target_tree = None
                self.target_observations = []
                if self.mission_mode == "all":
                    self.transition("SEARCH_TREE", "Pohon selesai; mencari pohon belum diinspeksi")
                else:
                    self.transition("RETRACE_HOME", "Hover di titik sebelum orbit selesai")

        elif self.state == "SCAN_FOR_TREE":
            if not have_pose or self.scan_anchor is None:
                self.transition("RETURN_HOME", "Anchor scan tidak tersedia")
                return
            anchor_x, anchor_y, anchor_yaw = self.scan_anchor
            elapsed = now - self.state_enter_time
            self.publish_goal(anchor_x, anchor_y, anchor_yaw + self.scan_yaw_rate * elapsed)
            tree = self.find_nearest_tree()
            if tree is not None:
                self.target_tree = deepcopy(tree)
                self.target_observations = []
                self.set_active_tree(int(tree.id))
                self.transition("VERIFY_TARGET", f"Scan menemukan pohon baru ID={tree.id}")
            elif elapsed >= self.scan_duration_sec:
                self.transition("RETRACE_HOME", "Scan penuh tidak menemukan pohon baru")

        elif self.state == "RETRACE_HOME":
            if not have_pose:
                self.publish_home_hover()
                return
            if not self.return_path_samples:
                self.transition("RETURN_HOME", "Jejak balik tidak tersedia; direct home")
                return
            if self.return_retrace_index is None:
                self.return_retrace_index = len(self.return_path_samples) - 1

            if self.return_retrace_index < 0:
                self.transition("HOME_HOVER", "Jejak balik selesai")
                return

            point_x, point_y, point_z, point_yaw = self.return_path_samples[
                self.return_retrace_index
            ]
            self.publish_goal(point_x, point_y, point_yaw, z=point_z)
            if distance_2d(cx, cy, point_x, point_y) <= self.approach_tolerance:
                self.return_retrace_index -= 1

        elif self.state == "RETURN_HOME":
            if not have_pose:
                self.publish_home_hover()
                return
            self.publish_goal(self.home_x, self.home_y, self.home_yaw)
            if distance_2d(cx, cy, self.home_x, self.home_y) <= self.home_reached_tolerance:
                self.transition("HOME_HOVER", "Kembali ke titik awal takeoff")

        elif self.state == "HOME_HOVER":
            self.publish_home_hover()
            if self.hover_stage_complete(self.home_hover_sec):
                self.transition("LAND", "Hover di home selesai")

        elif self.state == "LAND":
            self.stop_orbit()
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

        elif self.state == "PILOT_OVERRIDE":
            # No flight commands, goals, mode changes, landing commands, or
            # zero-velocity setpoints are sent in this state. Pixhawk and RC
            # have full authority. Returning the switch to GUIDED does not
            # resume the old mission; restart the node from the ground.
            self.publish_control_authority(False)
            if not self.armed:
                self.transition("DONE", "Drone disarm setelah pilot takeover")

        elif self.state == "ABORTED":
            self.stop_orbit()
            if self.armed:
                self.transition("LAND", "Misi dibatalkan")
            else:
                self.transition("DONE", "Misi dibatalkan sebelum terbang")

        elif self.state == "LANDING":
            land_msg = Bool(); land_msg.data = True
            self.cmd_land_pub.publish(land_msg)
            # Ulangi command sampai kendaraan benar-benar turun atau disarm,
            # agar satu pesan yang hilang tidak menggagalkan landing.
            if not self.is_armed or self.current_pose.pose.position.z < 0.20:
                self.state = "DONE"
                self.get_logger().info("Landing selesai. Misi DONE.")
            
        elif self.state == "DONE":
            self.stop_orbit()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_orbit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
