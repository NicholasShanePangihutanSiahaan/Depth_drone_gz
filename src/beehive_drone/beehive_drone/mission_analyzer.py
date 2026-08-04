#!/usr/bin/env python3

import csv
import math
import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import VfrHud
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from uav_interfaces.msg import TreeArray


class MissionAnalyzer(Node):
    """Low-overhead mission logger with optional live 2D visualization."""

    def __init__(self) -> None:
        super().__init__("mission_analyzer")

        self.declare_parameter("output_dir", "~/beehive_mission_results")
        self.declare_parameter("sample_period_sec", 0.2)
        self.declare_parameter("summary_period_sec", 5.0)
        self.declare_parameter("enable_live_plot", False)

        self.output_dir = Path(
            os.path.expanduser(str(self.get_parameter("output_dir").value))
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.sample_period_sec = float(self.get_parameter("sample_period_sec").value)
        self.summary_period_sec = float(
            self.get_parameter("summary_period_sec").value
        )
        self.enable_live_plot = bool(self.get_parameter("enable_live_plot").value)

        self.start_time = self.now_sec()
        self.last_sample_time = -1e9
        self.last_summary_time = -1e9
        self.current_state = "INIT"
        self.orbit_progress = 0.0
        self.current_pose: Optional[PoseStamped] = None
        self.current_velocity = (0.0, 0.0, 0.0)
        self.heading = 0.0
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self.throttle = 0.0
        self.trees = []
        self.rows = []
        self.state_events = []
        self.total_distance = 0.0
        self.previous_position = None
        self.saved = False
        self.figure = None
        self.axis = None

        if self.enable_live_plot:
            try:
                import matplotlib.pyplot as plt

                self.plt = plt
                plt.ion()
                self.figure, self.axis = plt.subplots(figsize=(10, 8))
            except Exception as exc:  # noqa: BLE001
                self.enable_live_plot = False
                self.get_logger().warning(f"Live plot dinonaktifkan: {exc}")

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
        self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self.velocity_callback,
            qos_sensor,
        )
        self.create_subscription(
            VfrHud, "/mavros/vfr_hud", self.vfr_callback, qos_sensor
        )
        self.create_subscription(TreeArray, "/map/trees", self.tree_callback, qos_map)
        self.create_subscription(
            String, "/mission/fsm_state", self.state_callback, 10
        )
        self.create_subscription(
            Float32, "/control/orbit_progress", self.orbit_progress_callback, 10
        )

        self.create_timer(0.2, self.update)
        self.get_logger().info(f"Mission Analyzer aktif; output={self.output_dir}.")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg
        now = self.now_sec()
        if now - self.last_sample_time < self.sample_period_sec:
            return
        self.last_sample_time = now

        p = msg.pose.position
        current = (float(p.x), float(p.y), float(p.z))
        if self.previous_position is not None:
            dx = current[0] - self.previous_position[0]
            dy = current[1] - self.previous_position[1]
            dz = current[2] - self.previous_position[2]
            step = math.sqrt(dx * dx + dy * dy + dz * dz)
            if step < 3.0:  # reject telemetry jumps
                self.total_distance += step
        self.previous_position = current

        vx, vy, vz = self.current_velocity
        self.rows.append(
            {
                "time": now - self.start_time,
                "x": current[0],
                "y": current[1],
                "z": current[2],
                "vx": vx,
                "vy": vy,
                "vz": vz,
                "heading": self.heading,
                "airspeed": self.airspeed,
                "groundspeed": self.groundspeed,
                "throttle": self.throttle,
                "state": self.current_state,
                "orbit_progress": self.orbit_progress,
            }
        )

    def velocity_callback(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self.current_velocity = (float(v.x), float(v.y), float(v.z))

    def vfr_callback(self, msg: VfrHud) -> None:
        self.heading = float(msg.heading)
        self.airspeed = float(msg.airspeed)
        self.groundspeed = float(msg.groundspeed)
        self.throttle = float(msg.throttle)

    def tree_callback(self, msg: TreeArray) -> None:
        self.trees = list(msg.trees)

    def state_callback(self, msg: String) -> None:
        if msg.data == self.current_state:
            return
        self.current_state = msg.data
        self.state_events.append((self.now_sec() - self.start_time, msg.data))
        if msg.data == "DONE" and not self.saved:
            self.save_results()

    def orbit_progress_callback(self, msg: Float32) -> None:
        self.orbit_progress = float(msg.data)

    def tree_counts(self):
        total = len(self.trees)
        inspected = sum(bool(tree.inspected) for tree in self.trees)
        validated = sum(
            bool(getattr(tree, "validated", tree.confidence >= 0.35))
            for tree in self.trees
        )
        return total, validated, inspected

    def update(self) -> None:
        now = self.now_sec()
        if now - self.last_summary_time >= self.summary_period_sec:
            self.last_summary_time = now
            total, validated, inspected = self.tree_counts()
            duration = max(1e-6, now - self.start_time)
            coverage = 100.0 * inspected / total if total else 0.0
            self.get_logger().info(
                f"State={self.current_state} | t={duration:.1f}s | "
                f"distance={self.total_distance:.1f}m | trees={inspected}/{total} | "
                f"validated={validated} | coverage={coverage:.1f}%"
            )
        if self.enable_live_plot:
            self.draw_map()

    def draw_map(self) -> None:
        if self.axis is None or self.figure is None:
            return
        self.axis.clear()
        self.axis.set_title("UAV Plantation Mission")
        self.axis.set_xlabel("X (m)")
        self.axis.set_ylabel("Y (m)")
        self.axis.grid(True)
        self.axis.set_aspect("equal", adjustable="box")

        if len(self.rows) > 1:
            self.axis.plot(
                [row["x"] for row in self.rows],
                [row["y"] for row in self.rows],
                linewidth=1.5,
                label="Trajectory",
            )
        if self.current_pose is not None:
            p = self.current_pose.pose.position
            self.axis.scatter(p.x, p.y, s=80, marker="o", label="UAV")
        self.axis.scatter(0.0, 0.0, marker="*", s=140, label="Home")

        for tree in self.trees:
            marker = "x" if tree.inspected else "o"
            self.axis.scatter(tree.x, tree.y, s=45, marker=marker)
            self.axis.text(tree.x, tree.y + 0.25, str(tree.id), fontsize=8)

        total, _, inspected = self.tree_counts()
        self.axis.text(
            0.02,
            0.98,
            f"State: {self.current_state}\nTrees: {inspected}/{total}\n"
            f"Distance: {self.total_distance:.1f} m\nOrbit: {100*self.orbit_progress:.0f}%",
            transform=self.axis.transAxes,
            verticalalignment="top",
        )
        self.axis.legend(loc="best")
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def save_results(self) -> None:
        if self.saved:
            return
        self.saved = True
        stamp = time.strftime("%Y%m%d_%H%M%S")
        trajectory_file = self.output_dir / f"mission_trajectory_{stamp}.csv"
        trees_file = self.output_dir / f"mission_trees_{stamp}.csv"
        states_file = self.output_dir / f"mission_states_{stamp}.csv"

        fields = [
            "time",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "heading",
            "airspeed",
            "groundspeed",
            "throttle",
            "state",
            "orbit_progress",
        ]
        with trajectory_file.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)

        with trees_file.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["id", "x", "y", "z", "confidence", "validated", "inspected"])
            for tree in self.trees:
                writer.writerow(
                    [
                        tree.id,
                        tree.x,
                        tree.y,
                        tree.z,
                        tree.confidence,
                        getattr(tree, "validated", tree.confidence >= 0.35),
                        tree.inspected,
                    ]
                )

        with states_file.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["time", "state"])
            writer.writerows(self.state_events)

        if self.enable_live_plot and self.figure is not None:
            self.draw_map()
            self.figure.savefig(
                self.output_dir / f"mission_map_{stamp}.png",
                dpi=220,
                bbox_inches="tight",
            )
        self.get_logger().info(f"Hasil misi tersimpan di {self.output_dir}.")

    def shutdown(self) -> None:
        self.save_results()
        if self.enable_live_plot and self.figure is not None:
            self.plt.close(self.figure)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
