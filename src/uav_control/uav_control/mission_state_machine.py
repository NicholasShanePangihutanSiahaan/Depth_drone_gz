#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String
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
        # Parameter Strategi Kebun (Sesuai Flowchart)
        # ==========================================
        self.explore_speed = 1.0          # m/s target step
        self.crab_speed = 0.5             # m/s target step (menyamping)
        self.end_of_row_dist = 10.0       # Batas lorong habis (meter) dari pohon terakhir
        self.end_of_farm_dist = 20.0      # Batas lahan habis (meter) di sumbu Y
        self.approach_safe_dist = 2.0     # Jarak hover menstabilkan (2 meter sebelum pohon)
        self.flight_altitude = 5.0        # Ketinggian terbang konstan

        # ==========================================
        # Variabel State & Navigasi
        # ==========================================
        self.state = "INIT"
        self.flight_status = "IDLE"
        self.orbit_status = "IDLE"
        self.current_pose = None
        self.trees = []
        self.target_tree = None

        self.explore_dir_x = 1.0          # 1.0 (Maju/Utara), -1.0 (Mundur/Selatan)
        self.explore_dir_y = 1.0          # Arah geser Crab Scan
        
        self.last_tree_x = 0.0
        self.last_tree_y = 0.0
        self.crab_start_y = 0.0
        
        self.spin_accumulated = 0.0
        self.last_yaw = 0.0

        # ==========================================
        # Subscriber
        # ==========================================
        self.pose_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_cb, qos_sensor)
        self.flight_status_sub = self.create_subscription(String, "/flight/status", self.flight_status_cb, 10)
        self.orbit_status_sub = self.create_subscription(String, "/control/orbit_status", self.orbit_status_cb, 10)
        self.tree_sub = self.create_subscription(TreeArray, "/map/trees", self.tree_cb, qos_map)

        # ==========================================
        # Publisher
        # ==========================================
        # Publisher untuk Flight Manager
        self.takeoff_pub = self.create_publisher(Bool, "/mission/takeoff", 10)
        self.land_pub = self.create_publisher(Bool, "/mission/land", 10)
        
        # Publisher untuk Dynamic Orbit Controller
        self.orbit_start_pub = self.create_publisher(Bool, "/control/orbit_start", 10)
        self.orbit_target_pub = self.create_publisher(Point, "/control/orbit_target", 10)
        
        # Publisher tujuan utama ke Vortex Controller / Velocity Controller
        self.local_goal_pub = self.create_publisher(PoseStamped, "/navigation/local_goal", 10)
        self.fsm_status_pub = self.create_publisher(String, "/mission/fsm_state", 10)

        # Timer FSM berjalan pada 10 Hz
        self.timer = self.create_timer(0.1, self.fsm_loop)
        self.get_logger().info("Mission State Machine (The Brain) Siap!")

    # --- Callbacks ---
    def pose_cb(self, msg): 
        self.current_pose = msg

    def flight_status_cb(self, msg): 
        self.flight_status = msg.data

    def orbit_status_cb(self, msg): 
        self.orbit_status = msg.data

    def tree_cb(self, msg): 
        self.trees = msg.trees

    # --- Helper Functions ---
    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def find_uninspected_tree(self):
        """Mencari pohon terdekat yang belum diinspeksi (berada di depan drone)"""
        if self.current_pose is None: return None
        cx, cy = self.current_pose.pose.position.x, self.current_pose.pose.position.y
        
        best_tree = None
        min_dist = float('inf')

        for tree in self.trees:
            if not tree.inspected:
                dist = self.distance(cx, cy, tree.x, tree.y)
                # Pastikan pohon berada searah dengan arah pencarian saat ini (di depan drone)
                is_ahead = (tree.x - cx) * self.explore_dir_x >= -1.0
                if is_ahead and dist < min_dist and dist < 15.0: # Cari dalam radius 15m
                    min_dist = dist
                    best_tree = tree
        return best_tree

    def publish_goal(self, x, y, yaw):
        """Mengirim setpoint navigasi lokal"""
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

        # Log Status FSM
        msg = String(); msg.data = self.state
        self.fsm_status_pub.publish(msg)

        if self.state == "INIT":
            # Perintahkan Takeoff
            takeoff_msg = Bool(); takeoff_msg.data = True
            self.takeoff_pub.publish(takeoff_msg)
            self.state = "WAIT_TAKEOFF"
            self.get_logger().info("Mengirim perintah Take-Off...")

        elif self.state == "WAIT_TAKEOFF":
            if self.flight_status == "HOVER":
                self.last_tree_x = cx
                self.last_tree_y = cy
                self.state = "EXPLORE_ROW"
                self.get_logger().info("Hover tercapai. Mulai EXPLORE_ROW (Mencari Pohon).")

        elif self.state == "EXPLORE_ROW":
            # Flowchart: Cari Pohon Sawit
            self.target_tree = self.find_uninspected_tree()
            
            if self.target_tree is not None:
                self.state = "APPROACH_TREE"
                self.get_logger().info(f"Pohon ditemukan di ({self.target_tree.x:.1f}, {self.target_tree.y:.1f})")
            else:
                # Maju menyusuri lorong
                target_x = cx + (self.explore_speed * self.explore_dir_x)
                target_yaw = 0.0 if self.explore_dir_x > 0 else math.pi
                self.publish_goal(target_x, cy, target_yaw)

                # Flowchart: Pohon habis (tidak terdeteksi setelah 10 meter)
                dist_from_last = self.distance(cx, cy, self.last_tree_x, self.last_tree_y)
                if dist_from_last > self.end_of_row_dist:
                    self.state = "END_OF_ROW"
                    self.get_logger().info("Lorong Habis. Bersiap pindah lorong.")

        elif self.state == "APPROACH_TREE":
            # Flowchart: Approach (tidak nabrak, mungkin 2 meter sebelum pohon)
            dist_to_tree = self.distance(cx, cy, self.target_tree.x, self.target_tree.y)
            
            target_yaw = math.atan2(self.target_tree.y - cy, self.target_tree.x - cx)
            
            if dist_to_tree > self.approach_safe_dist:
                # Dorong drone perlahan menuju titik 2 meter sebelum pohon
                self.publish_goal(self.target_tree.x, self.target_tree.y, target_yaw)
            else:
                # Flowchart: Hovering untuk menstabilkan & Catat lokasi koordinat
                self.state = "START_ORBIT"
                self.get_logger().info("Jarak aman (2m) tercapai. Memulai orbit.")

        elif self.state == "START_ORBIT":
            # Flowchart: Mengelilingi pohon dengan arah kamera 45 derajat
            # Memicu Dynamic Orbit Controller (Node Terpisah)
            target_msg = Point()
            target_msg.x = self.target_tree.x
            target_msg.y = self.target_tree.y
            target_msg.z = self.target_tree.z
            self.orbit_target_pub.publish(target_msg)
            
            start_msg = Bool(); start_msg.data = True
            self.orbit_start_pub.publish(start_msg)
            
            self.state = "WAIT_ORBIT"

        elif self.state == "WAIT_ORBIT":
            # Dynamic Orbit Controller sedang bekerja. 
            # (Vortex Avoidance Controller secara otomatis mencegah ranting saat orbit berlangsung)
            if self.orbit_status == "ORBIT_COMPLETED":
                # Matikan perintah orbit
                stop_msg = Bool(); stop_msg.data = False
                self.orbit_start_pub.publish(stop_msg)
                
                # Update posisi terakhir untuk acuan lorong
                self.last_tree_x = self.target_tree.x
                self.last_tree_y = self.target_tree.y
                
                # Flowchart: Hovering menstabilkan -> Ke lorong (kembali mencari)
                self.target_tree = None
                self.state = "EXPLORE_ROW"
                self.get_logger().info("Orbit selesai. Kembali menyusuri lorong.")

        elif self.state == "END_OF_ROW":
            # Flowchart: Drone ke 2 meter dekat pohon terakhir
            retreat_x = self.last_tree_x - (self.approach_safe_dist * self.explore_dir_x)
            target_yaw = 0.0 if self.explore_dir_x > 0 else math.pi
            
            self.publish_goal(retreat_x, self.last_tree_y, target_yaw)
            
            if abs(cx - retreat_x) < 0.5:
                self.explore_dir_x *= -1.0 # Balik arah (U-Turn logis)
                self.crab_start_y = cy
                self.state = "CRAB_SCAN"
                self.get_logger().info("Mundur selesai. Memulai Crab Scan 90 derajat.")

        elif self.state == "CRAB_SCAN":
            # Flowchart: Crab searching 90 derajat dari pohon agar menemukan pohon di lorong berbeda
            target_y = cy + (self.crab_speed * self.explore_dir_y)
            target_yaw = 0.0 if self.explore_dir_x > 0 else math.pi
            
            self.publish_goal(cx, target_y, target_yaw)
            
            self.target_tree = self.find_uninspected_tree()
            if self.target_tree is not None:
                # Flowchart: Iya, menemukan pohon
                self.last_tree_y = self.target_tree.y
                self.state = "APPROACH_TREE"
                self.get_logger().info("Lorong baru ditemukan!")
            else:
                # Flowchart: Tidak menemukan pohon. Semua lorong tersusuri
                if abs(cy - self.crab_start_y) > self.end_of_farm_dist:
                    self.state = "RETURN_TO_HOME"
                    self.get_logger().info("Lahan habis. Cari jalur untuk pulang (RTH).")

        elif self.state == "RETURN_TO_HOME":
            # Flowchart: Sampai di lokasi awal (sambil menghindari obstacle by Vortex)
            target_yaw = math.atan2(0.0 - cy, 0.0 - cx)
            self.publish_goal(0.0, 0.0, target_yaw)
            
            if self.distance(cx, cy, 0.0, 0.0) < 1.0:
                self.state = "FINAL_SPIN"
                
                # Inisialisasi parameter Yaw untuk 360 Spin
                qx = self.current_pose.pose.orientation.x
                qy = self.current_pose.pose.orientation.y
                qz = self.current_pose.pose.orientation.z
                qw = self.current_pose.pose.orientation.w
                self.last_yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
                self.spin_accumulated = 0.0
                
                self.get_logger().info("Tiba di Home. Memulai rotasi 360 derajat.")

        elif self.state == "FINAL_SPIN":
            # Flowchart: Berputar sekalian mencari pohon yang belum terdeteksi
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
                # Flowchart: Ada pohon yang belum pernah di-screening -> Lanjut inspeksi
                self.state = "APPROACH_TREE"
                self.get_logger().info("Pohon terlewat ditemukan saat Final Spin!")
                return
                
            if self.spin_accumulated >= 2 * math.pi:
                # Flowchart: Tidak ada -> Landing
                self.state = "LANDING"
                self.get_logger().info("Area bersih. Memulai Pendaratan.")
            else:
                # Perintah memutar di tempat (Yaw bertambah)
                target_yaw = current_yaw + 0.2
                self.publish_goal(cx, cy, target_yaw)

        elif self.state == "LANDING":
            # Flowchart: Landing, turun sampai ke tanah & Disarm
            land_msg = Bool(); land_msg.data = True
            self.land_pub.publish(land_msg)
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