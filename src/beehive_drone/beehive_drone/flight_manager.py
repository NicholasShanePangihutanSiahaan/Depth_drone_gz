#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String

from beehive_drone.mission_params import MissionConfig


class FlightManager(Node):
    """Hardware-abstraction layer between mission logic and MAVROS services."""

    def __init__(self) -> None:
        super().__init__("flight_manager")

        self.declare_parameter("hover_alt_tolerance", MissionConfig.HOVER_ALT_TOLERANCE)
        self.declare_parameter("hover_speed_tolerance", MissionConfig.HOVER_SPEED_TOLERANCE)
        self.declare_parameter("hover_stable_sec", MissionConfig.HOVER_STABLE_SEC)

        self.hover_alt_tolerance = float(self.get_parameter("hover_alt_tolerance").value)
        self.hover_speed_tolerance = float(self.get_parameter("hover_speed_tolerance").value)
        self.hover_stable_sec = float(self.get_parameter("hover_stable_sec").value)

        self.current_state = State()
        self.current_alt = 0.0
        self.current_speed = 0.0
        self.target_takeoff_alt = 0.0
        self.hover_candidate_since: Optional[float] = None
        self.last_pose_time: Optional[float] = None
        self.pending_mode = False
        self.pending_arm = False
        self.pending_takeoff = False
        self.pending_land = False

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(State, "/mavros/state", self.state_callback, 10)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_callback, qos_sensor
        )
        self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self.velocity_callback,
            qos_sensor,
        )

        self.create_subscription(String, "/flight/cmd/set_mode", self.cmd_mode_cb, 10)
        self.create_subscription(Bool, "/flight/cmd/set_arm", self.cmd_arm_cb, 10)
        self.create_subscription(Float32, "/flight/cmd/takeoff", self.cmd_takeoff_cb, 10)
        self.create_subscription(Bool, "/flight/cmd/land", self.cmd_land_cb, 10)

        self.pub_connected = self.create_publisher(
            Bool, "/flight/telemetry/is_connected", 10
        )
        self.pub_armed = self.create_publisher(Bool, "/flight/telemetry/is_armed", 10)
        self.pub_mode = self.create_publisher(
            String, "/flight/telemetry/current_mode", 10
        )
        self.pub_alt = self.create_publisher(Float32, "/flight/telemetry/altitude", 10)
        self.pub_hover = self.create_publisher(
            Bool, "/flight/telemetry/is_hovering", 10
        )

        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

        self.create_timer(0.1, self.publish_telemetry)
        self.get_logger().info("Flight Manager aktif.")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def state_callback(self, msg: State) -> None:
        self.current_state = msg

    def pose_callback(self, msg: PoseStamped) -> None:
        self.current_alt = float(msg.pose.position.z)
        self.last_pose_time = self.now_sec()

    def velocity_callback(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self.current_speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def _service_ready(self, client, name: str) -> bool:
        if client.service_is_ready():
            return True
        if not client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning(f"Service {name} belum tersedia.")
            return False
        return True

    def cmd_mode_cb(self, msg: String) -> None:
        mode = msg.data.strip()
        if not mode or self.current_state.mode == mode or self.pending_mode:
            return
        if not self._service_ready(self.mode_client, "/mavros/set_mode"):
            return

        request = SetMode.Request()
        request.custom_mode = mode
        self.pending_mode = True
        future = self.mode_client.call_async(request)
        future.add_done_callback(lambda f: self._finish_service("mode", f))

    def cmd_arm_cb(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if self.current_state.armed == requested or self.pending_arm:
            return
        if not self._service_ready(self.arm_client, "/mavros/cmd/arming"):
            return

        request = CommandBool.Request()
        request.value = requested
        self.pending_arm = True
        future = self.arm_client.call_async(request)
        future.add_done_callback(lambda f: self._finish_service("arm", f))

    def cmd_takeoff_cb(self, msg: Float32) -> None:
        altitude = float(msg.data)
        if altitude <= 0.0:
            self.get_logger().error("Perintah takeoff ditolak: altitude harus positif.")
            return
        self.target_takeoff_alt = altitude
        if self.pending_takeoff:
            return
        if not self._service_ready(self.takeoff_client, "/mavros/cmd/takeoff"):
            return

        request = CommandTOL.Request()
        request.altitude = altitude
        self.pending_takeoff = True
        future = self.takeoff_client.call_async(request)
        future.add_done_callback(lambda f: self._finish_service("takeoff", f))

    def cmd_land_cb(self, msg: Bool) -> None:
        if not msg.data or self.pending_land:
            return
        if not self._service_ready(self.land_client, "/mavros/cmd/land"):
            return

        request = CommandTOL.Request()
        self.pending_land = True
        future = self.land_client.call_async(request)
        future.add_done_callback(lambda f: self._finish_service("land", f))

    def _finish_service(self, kind: str, future) -> None:
        setattr(self, f"pending_{kind}", False)
        try:
            response = future.result()
            success = bool(
                getattr(response, "success", getattr(response, "mode_sent", False))
            )
            if success:
                self.get_logger().info(f"Perintah {kind} diterima autopilot.")
            else:
                self.get_logger().warning(f"Perintah {kind} ditolak autopilot.")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Service {kind} gagal: {exc}")

    def _is_stable_hover(self) -> bool:
        if not self.current_state.armed or self.target_takeoff_alt <= 0.0:
            self.hover_candidate_since = None
            return False

        altitude_ok = (
            abs(self.current_alt - self.target_takeoff_alt) <= self.hover_alt_tolerance
        )
        speed_ok = self.current_speed <= self.hover_speed_tolerance
        now = self.now_sec()

        if altitude_ok and speed_ok:
            if self.hover_candidate_since is None:
                self.hover_candidate_since = now
            return (now - self.hover_candidate_since) >= self.hover_stable_sec

        self.hover_candidate_since = None
        return False

    def publish_telemetry(self) -> None:
        connected = Bool()
        connected.data = bool(self.current_state.connected)
        self.pub_connected.publish(connected)

        armed = Bool()
        armed.data = bool(self.current_state.armed)
        self.pub_armed.publish(armed)

        mode = String()
        mode.data = self.current_state.mode
        self.pub_mode.publish(mode)

        altitude = Float32()
        altitude.data = float(self.current_alt)
        self.pub_alt.publish(altitude)

        hovering = Bool()
        hovering.data = self._is_stable_hover()
        self.pub_hover.publish(hovering)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
