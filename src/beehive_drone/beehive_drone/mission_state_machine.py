#!/usr/bin/env python3

import math
import statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from beehive_drone.mission_params import MissionConfig
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String, Float32
from uav_interfaces.msg import TreeArray, Tree

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class MissionStateMachine(Node):
    def __init__(self):
        super().__init__("mission_state_machine")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ==========================================
        # Parameter Strategi Kebun 
        # ==========================================
        self.explore_speed = MissionConfig.EXPLORE_SPEED  # Kecepatan menyusuri lorong (m/s)          
        self.crab_speed = MissionConfig.CRAB_SPEED
        self.end_of_row_dist = MissionConfig.END_OF_ROW_DIST
        self.end_of_farm_dist = MissionConfig.END_OF_FARM_DIST
        self.approach_safe_dist = MissionConfig.APPROACH_SAFE_DIST
        self.flight_altitude = MissionConfig.FLIGHT_ALTITUDE
        self.orbit_radius = MissionConfig.ORBIT_RADIUS
        self.verify_timeout = MissionConfig.TREE_VERIFY_TIMEOUT
        self.verify_samples_required = MissionConfig.TREE_VERIFY_SAMPLES
        self.verify_spread = MissionConfig.TREE_VERIFY_SPREAD
        self.verify_duration = MissionConfig.TREE_VERIFY_DURATION
        self.verify_drift = MissionConfig.TREE_VERIFY_DRIFT
        self.verify_min_confidence = MissionConfig.TREE_VERIFY_MIN_CONFIDENCE
        self.hover_position_tolerance = MissionConfig.TREE_HOVER_POSITION_TOL
        self.align_position_tolerance = MissionConfig.ORBIT_ALIGN_POSITION_TOL
        self.align_radius_tolerance = MissionConfig.ORBIT_ALIGN_RADIUS_TOL
        self.align_yaw_tolerance = MissionConfig.ORBIT_ALIGN_YAW_TOL
        self.align_hold_required = MissionConfig.ORBIT_ALIGN_HOLD

        # ==========================================
        # Variabel State & Navigasi
        # ==========================================
        self.state = "INIT"
        self.retry_counter = 0
        self.hover_timer = 0
        self.orbit_status = "IDLE"
        self.current_pose = None
        self.trees = []
        self.target_tree = None
        self.tree_observations = {}
        self.tree_observation_counts = {}
        self.verify_started_at = 0.0
        self.verify_start_count = 0
        self.align_stable_since = None
        self.verify_hover_x = 0.0
        self.verify_hover_y = 0.0
        self.pose_history = []

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
        self.observation_sub = self.create_subscription(
            Tree,
            "/perception/tree_observation",
            self.tree_observation_cb,
            10
        )

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
        self.fsm_status_pub = self.create_publisher(String, "/mission/fsm_state", 10)
        # Publisher untuk memperbarui status pohon ke Tree Mapper
        self.tree_update_pub = self.create_publisher(Tree, "/map/tree_update", 10)

        # Timer FSM berjalan pada 10 Hz
        self.timer = self.create_timer(0.1, self.fsm_loop)
        self.get_logger().info("Mission State Machine (The Brain) Siap!")

    # --- Callbacks Sensor & Status ---
    def pose_cb(self, msg):
        self.current_pose = msg
        now = self.get_clock().now().nanoseconds * 1e-9
        self.pose_history.append(
            (now, msg.pose.position.x, msg.pose.position.y)
        )
        self.pose_history = [
            sample for sample in self.pose_history if now - sample[0] <= 2.0
        ]
    def orbit_status_cb(self, msg): self.orbit_status = msg.data
    def tree_cb(self, msg):
        self.trees = msg.trees
        if self.target_tree is None:
            return
        # Lock by map ID, not by nearest coordinate. During approach and hover
        # the target center follows the mapper's latest robust PCL estimate.
        updated = next(
            (tree for tree in msg.trees if tree.id == self.target_tree.id),
            None,
        )
        if updated is not None and self.state in (
            "APPROACH_TREE", "VERIFY_TREE"
        ):
            self.target_tree = updated
    def tree_observation_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        sequence = self.tree_observation_counts.get(msg.id, 0) + 1
        self.tree_observation_counts[msg.id] = sequence
        history = self.tree_observations.setdefault(msg.id, [])
        history.append((sequence, now, msg.x, msg.y, msg.z, msg.confidence))
        del history[:-20]
    
    # --- Callbacks Telemetri ---
    def arm_cb(self, msg): self.is_armed = msg.data
    def mode_cb(self, msg): self.current_mode = msg.data
    def hover_cb(self, msg): self.is_hovering = msg.data

    # --- Helper Functions ---
    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def is_position_stable(self, now, duration=1.0):
        samples = [sample for sample in self.pose_history if now - sample[0] <= duration]
        if len(samples) < 5 or samples[-1][0] - samples[0][0] < duration * 0.8:
            return False
        center_x = statistics.median(sample[1] for sample in samples)
        center_y = statistics.median(sample[2] for sample in samples)
        return max(
            self.distance(sample[1], sample[2], center_x, center_y)
            for sample in samples
        ) <= self.hover_position_tolerance

    def find_uninspected_tree(self):
        if self.current_pose is None: return None
        cx, cy = self.current_pose.pose.position.x, self.current_pose.pose.position.y
        
        best_tree = None
        min_dist = float('inf')

        for tree in self.trees:
            if (
                not tree.inspected
                and tree.confidence >= self.verify_min_confidence
            ):
                dist = self.distance(cx, cy, tree.x, tree.y)
                is_ahead = (tree.x - cx) * self.explore_dir_x >= -1.0
                if is_ahead and dist < min_dist and dist < 15.0: 
                    min_dist = dist
                    best_tree = tree
        return best_tree

    def publish_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = self.flight_altitude
        
        qx, qy, qz, qw = euler_to_quaternion(0, 0, yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        
        self.local_goal_pub.publish(goal)

    # ==========================================
    # LOGIKA STATE MACHINE UTAMA
    # ==========================================
    def fsm_loop(self):
        if self.current_pose is None:
            return

        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y

        msg = String(); msg.data = self.state
        self.fsm_status_pub.publish(msg)

        # --- FASE PRE-FLIGHT ---
        if self.state == "INIT":
            mode_msg = String(); mode_msg.data = "GUIDED"
            self.cmd_mode_pub.publish(mode_msg)
            self.state = "WAIT_GUIDED"
            self.retry_counter = 0
            self.get_logger().info("Meminta transisi ke mode GUIDED...")

        elif self.state == "WAIT_GUIDED":
            if self.current_mode == "GUIDED":
                arm_msg = Bool(); arm_msg.data = True
                self.cmd_arm_pub.publish(arm_msg)
                self.state = "WAIT_ARM"
                self.retry_counter = 0
                self.get_logger().info("Mode GUIDED aktif. Meminta Arming Motor...")
            else:
                self.retry_counter += 1
                if self.retry_counter > 20:  # Ulangi perintah setiap 2 detik (20 x 0.1s)
                    mode_msg = String(); mode_msg.data = "GUIDED"
                    self.cmd_mode_pub.publish(mode_msg)
                    self.retry_counter = 0

        elif self.state == "WAIT_ARM":
            if self.is_armed:
                takeoff_msg = Float32(); takeoff_msg.data = self.flight_altitude
                self.cmd_takeoff_pub.publish(takeoff_msg)
                self.state = "WAIT_TAKEOFF"
                self.retry_counter = 0
                self.get_logger().info(f"Motor Bersenjata (Armed). Takeoff ke ketinggian {self.flight_altitude}m...")
            else:
                self.retry_counter += 1
                if self.retry_counter > 20:  # Ulangi perintah setiap 2 detik
                    arm_msg = Bool(); arm_msg.data = True
                    self.cmd_arm_pub.publish(arm_msg)
                    self.get_logger().info("Mencoba Arming ulang... (Menunggu Pre-arm good dari ArduPilot)")
                    self.retry_counter = 0

        elif self.state == "WAIT_TAKEOFF":
            if self.is_hovering:
                self.last_tree_x = cx
                self.last_tree_y = cy
                self.state = "EXPLORE_ROW"
                self.get_logger().info("Hover stabil tercapai. Mulai EXPLORE_ROW (Mencari Pohon).")

        # --- FASE MISI UTAMA ---
        elif self.state == "EXPLORE_ROW":
            self.target_tree = self.find_uninspected_tree()
            
            if self.target_tree is not None:
                self.state = "APPROACH_TREE"
                self.get_logger().info(f"Pohon ditemukan di ({self.target_tree.x:.1f}, {self.target_tree.y:.1f})")
            else:
                target_x = cx + (self.explore_speed * self.explore_dir_x)
                target_yaw = 0.0 if self.explore_dir_x > 0 else math.pi
                self.publish_goal(target_x, cy, target_yaw)

                dist_from_last = self.distance(cx, cy, self.last_tree_x, self.last_tree_y)
                if dist_from_last > self.end_of_row_dist:
                    self.state = "END_OF_ROW"
                    self.get_logger().info("Lorong Habis. Bersiap pindah lorong.")

        elif self.state == "APPROACH_TREE":
            # Hitung sudut arah (yaw) dari drone menuju pohon
            target_yaw = math.atan2(self.target_tree.y - cy, self.target_tree.x - cx)
            
            # 1. Kalkulasi TITIK PENGEREMAN (2 meter di depan pohon)
            stop_x = self.target_tree.x - (self.approach_safe_dist * math.cos(target_yaw))
            stop_y = self.target_tree.y - (self.approach_safe_dist * math.sin(target_yaw))
            
            # 2. Hitung jarak drone ke TITIK PENGEREMAN (bukan ke pohon)
            dist_to_stop = self.distance(cx, cy, stop_x, stop_y)
            
            # 3. Logika Bebas Deadlock
            # Karena velocity_controller akan mengerem di jarak 0.5m dari titik target,
            # FSM cukup menunggu sampai drone berada di jarak 0.6m dari titik pengereman.
            if dist_to_stop > 0.6:
                self.publish_goal(stop_x, stop_y, target_yaw)
            else:
                self.state = "VERIFY_TREE"
                self.verify_started_at = self.get_clock().now().nanoseconds * 1e-9
                self.verify_hover_x = cx
                self.verify_hover_y = cy
                self.verify_start_count = self.tree_observation_counts.get(
                    self.target_tree.id, 0
                )
                self.get_logger().info(
                    "Titik pendekatan tercapai. Menunggu observasi PCL baru "
                    "untuk memeriksa ulang koordinat pohon..."
                )
        
        elif self.state == "VERIFY_TREE":
            # Tahan posisi dan arahkan kamera ke kandidat selama PCL mengambil
            # beberapa observasi BARU. Isi /map/trees saja tidak cukup karena
            # dapat merupakan koordinat lama yang sudah stale.
            target_yaw = math.atan2(self.target_tree.y - cy, self.target_tree.x - cx)
            # Keep one fixed hover setpoint. Republishing the current position
            # made the hold point drift together with odometry/noise.
            self.publish_goal(
                self.verify_hover_x, self.verify_hover_y, target_yaw
            )
            now = self.get_clock().now().nanoseconds * 1e-9
            history = self.tree_observations.get(self.target_tree.id, [])
            fresh = [
                sample for sample in history
                if sample[0] > self.verify_start_count
            ]

            if len(fresh) >= self.verify_samples_required and self.is_position_stable(now):
                samples = fresh[-self.verify_samples_required:]
                mean_x = statistics.median(sample[2] for sample in samples)
                mean_y = statistics.median(sample[3] for sample in samples)
                mean_z = statistics.median(sample[4] for sample in samples)
                spread = max(
                    self.distance(sample[2], sample[3], mean_x, mean_y)
                    for sample in samples
                )
                half = len(samples) // 2
                first_x = statistics.median(sample[2] for sample in samples[:half])
                first_y = statistics.median(sample[3] for sample in samples[:half])
                last_x = statistics.median(sample[2] for sample in samples[half:])
                last_y = statistics.median(sample[3] for sample in samples[half:])
                drift = self.distance(first_x, first_y, last_x, last_y)
                sample_duration = samples[-1][1] - samples[0][1]
                min_confidence = min(sample[5] for sample in samples)

                stable_center = (
                    spread <= self.verify_spread
                    and drift <= self.verify_drift
                    and sample_duration >= self.verify_duration
                    and min_confidence >= self.verify_min_confidence
                )
                if stable_center:
                    old_x, old_y = self.target_tree.x, self.target_tree.y
                    self.target_tree.x = mean_x
                    self.target_tree.y = mean_y
                    self.target_tree.z = mean_z
                    self.target_tree.validated = True

                    # Sinkronkan pusat yang sudah dikoreksi ke mapper sebelum
                    # alignment. Orbit dan collision avoidance dengan demikian
                    # memakai koordinat pohon yang sama.
                    corrected_tree = Tree()
                    corrected_tree.id = self.target_tree.id
                    corrected_tree.x = mean_x
                    corrected_tree.y = mean_y
                    corrected_tree.z = mean_z
                    corrected_tree.confidence = self.target_tree.confidence
                    corrected_tree.inspected = False
                    corrected_tree.validated = True
                    self.tree_update_pub.publish(corrected_tree)

                    self.state = "ALIGN_ORBIT"
                    self.align_stable_since = None
                    self.get_logger().info(
                        f"Pohon ID:{self.target_tree.id} terverifikasi dari "
                        f"{len(samples)} observasi baru; koreksi koordinat "
                        f"({old_x:.2f},{old_y:.2f}) -> ({mean_x:.2f},{mean_y:.2f}), "
                        f"spread={spread:.2f}m, drift={drift:.2f}m. "
                        "Menyelaraskan titik masuk orbit."
                    )
                else:
                    self.get_logger().warning(
                        f"Observasi pohon ID:{self.target_tree.id} belum stabil "
                        f"(spread={spread:.2f}m, drift={drift:.2f}m, "
                        f"durasi={sample_duration:.1f}s, conf={min_confidence:.2f}); "
                        "menunggu sampel berikutnya."
                    )
                    self.verify_start_count = samples[0][0]

            elif now - self.verify_started_at > self.verify_timeout:
                self.get_logger().warning(
                    f"Pohon ID:{self.target_tree.id} tidak terlihat ulang oleh "
                    "PCL. Orbit dibatalkan dan kandidat dihapus."
                )
                update_msg = Tree()
                update_msg.id = self.target_tree.id
                update_msg.confidence = -1.0
                self.tree_update_pub.publish(update_msg)
                self.target_tree = None
                self.state = "EXPLORE_ROW"

        elif self.state == "ALIGN_ORBIT":
            # Masuk ke lingkaran orbit dari sisi drone saat ini, lalu hadapkan
            # kamera ke pusat pohon sebelum controller orbit diaktifkan.
            dx = cx - self.target_tree.x
            dy = cy - self.target_tree.y
            radius = math.hypot(dx, dy)
            if radius < 0.1:
                dx, dy, radius = -1.0, 0.0, 1.0

            entry_x = self.target_tree.x + self.orbit_radius * dx / radius
            entry_y = self.target_tree.y + self.orbit_radius * dy / radius
            target_yaw = math.atan2(
                self.target_tree.y - cy,
                self.target_tree.x - cx
            )
            self.publish_goal(entry_x, entry_y, target_yaw)

            position_error = self.distance(cx, cy, entry_x, entry_y)
            radius_error = abs(radius - self.orbit_radius)
            current_yaw = quaternion_to_yaw(self.current_pose.pose.orientation)
            yaw_error = abs(normalize_angle(target_yaw - current_yaw))
            now = self.get_clock().now().nanoseconds * 1e-9

            aligned = (
                position_error <= self.align_position_tolerance
                and radius_error <= self.align_radius_tolerance
                and yaw_error <= self.align_yaw_tolerance
            )
            if aligned:
                if self.align_stable_since is None:
                    self.align_stable_since = now
                elif now - self.align_stable_since >= self.align_hold_required:
                    self.state = "START_ORBIT"
                    self.get_logger().info(
                        f"Posisi pra-orbit stabil: radius={radius:.2f}m, "
                        f"yaw_error={math.degrees(yaw_error):.1f}deg. Memulai orbit."
                    )
            else:
                self.align_stable_since = None
                    
        elif self.state == "START_ORBIT":
            target_msg = Point()
            target_msg.x = self.target_tree.x
            target_msg.y = self.target_tree.y
            target_msg.z = self.target_tree.z
            self.orbit_target_pub.publish(target_msg)
            
            start_msg = Bool(); start_msg.data = True
            self.orbit_start_pub.publish(start_msg)
            self.state = "WAIT_ORBIT"

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
                    update_msg.validated = True
                    
                    self.tree_update_pub.publish(update_msg)
                    self.get_logger().info(f"Pohon ID:{self.target_tree.id} ditandai SELESAI (Inspected).")

                # 3. Update posisi terakhir untuk acuan lorong
                self.last_tree_x = self.target_tree.x
                self.last_tree_y = self.target_tree.y
                
                # 4. Kosongkan target dan kembali mencari
                self.target_tree = None
                self.state = "EXPLORE_ROW"
                self.get_logger().info("Orbit selesai. Kembali menyusuri lorong mencari pohon baru.")

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
            target_yaw = math.atan2(0.0 - cy, 0.0 - cx)
            self.publish_goal(0.0, 0.0, target_yaw)
            
            if self.distance(cx, cy, 0.0, 0.0) < 1.0:
                self.state = "FINAL_SPIN"
                qx = self.current_pose.pose.orientation.x
                qy = self.current_pose.pose.orientation.y
                qz = self.current_pose.pose.orientation.z
                qw = self.current_pose.pose.orientation.w
                self.last_yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
                self.spin_accumulated = 0.0
                self.get_logger().info("Tiba di Home. Memulai rotasi 360 derajat.")

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
                
            if self.spin_accumulated >= 2 * math.pi:
                self.state = "LANDING"
                self.get_logger().info("Area bersih. Memulai Pendaratan.")
            else:
                target_yaw = current_yaw + 0.2
                self.publish_goal(cx, cy, target_yaw)

        elif self.state == "LANDING":
            land_msg = Bool(); land_msg.data = True
            self.cmd_land_pub.publish(land_msg)
            self.state = "DONE"
            
        elif self.state == "DONE":
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MissionStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
