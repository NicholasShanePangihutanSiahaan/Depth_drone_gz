#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String, Float32
from uav_interfaces.msg import TreeArray

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class MissionStateMachine(Node):
    def __init__(self):
        super().__init__("mission_state_machine")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ==========================================
        # Parameter Strategi Kebun 
        # ==========================================
        self.explore_speed = 1.0          
        self.crab_speed = 0.5             
        self.end_of_row_dist = 10.0       
        self.end_of_farm_dist = 20.0      
        self.approach_safe_dist = 2.0     
        self.flight_altitude = 5.0        

        # ==========================================
        # Variabel State & Navigasi
        # ==========================================
        self.state = "INIT"
        self.retry_counter = 0
        self.orbit_status = "IDLE"
        self.current_pose = None
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
        self.fsm_status_pub = self.create_publisher(String, "/mission/fsm_state", 10)

        # Timer FSM berjalan pada 10 Hz
        self.timer = self.create_timer(0.1, self.fsm_loop)
        self.get_logger().info("Mission State Machine (The Brain) Siap!")

    # --- Callbacks Sensor & Status ---
    def pose_cb(self, msg): self.current_pose = msg
    def orbit_status_cb(self, msg): self.orbit_status = msg.data
    def tree_cb(self, msg): self.trees = msg.trees
    
    # --- Callbacks Telemetri ---
    def arm_cb(self, msg): self.is_armed = msg.data
    def mode_cb(self, msg): self.current_mode = msg.data
    def hover_cb(self, msg): self.is_hovering = msg.data

    # --- Helper Functions ---
    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def find_uninspected_tree(self):
        if self.current_pose is None: return None
        cx, cy = self.current_pose.pose.position.x, self.current_pose.pose.position.y
        
        best_tree = None
        min_dist = float('inf')

        for tree in self.trees:
            if not tree.inspected:
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
                self.state = "START_ORBIT"
                self.get_logger().info("Titik pengereman tercapai. Memulai orbit.")

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
                stop_msg = Bool(); stop_msg.data = False
                self.orbit_start_pub.publish(stop_msg)
                
                self.last_tree_x = self.target_tree.x
                self.last_tree_y = self.target_tree.y
                
                self.target_tree = None
                self.state = "EXPLORE_ROW"
                self.get_logger().info("Orbit selesai. Kembali menyusuri lorong.")

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
    rclpy.shutdown()

if __name__ == "__main__":
    main()