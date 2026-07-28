#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class OdomTester(Node):
    def __init__(self):
        super().__init__("odom_tester")

        # ==========================================
        # Konfigurasi Misi Uji
        # ==========================================
        self.alt_target = 3.0
        self.hover_duration = 4.0 # Waktu tunggu setiap titik (detik)
        self.dist_tolerance = 0.3 # Toleransi jarak sampai ke waypoint (meter)
        
        # Variabel Odometri
        self.current_state = State()
        self.current_pose = None
        self.start_x = 0.0
        self.start_y = 0.0
        
        # FSM Internal Tester
        self.step = "WAIT_CONN"
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "odom"
        
        self.hover_start_time = 0.0
        self.orbit_start_time = 0.0

        # ==========================================
        # ROS 2 Interfaces
        # ==========================================
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_cb, qos_sensor)
        self.setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        

        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

        # Loop Kontrol Utama (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Odom Tester Aktif. Menunggu koneksi FCU...")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def distance_to_target(self):
        if not self.current_pose: return float('inf')
        dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
        dz = self.target_pose.pose.position.z - self.current_pose.pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def set_target(self, x, y, z, yaw=0.0):
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.pose.position.x = float(x)
        self.target_pose.pose.position.y = float(y)
        self.target_pose.pose.position.z = float(z)
        
        qx, qy, qz, qw = euler_to_quaternion(0, 0, yaw)
        self.target_pose.pose.orientation.x = qx
        self.target_pose.pose.orientation.y = qy
        self.target_pose.pose.orientation.z = qz
        self.target_pose.pose.orientation.w = qw

    def trigger_hover(self, next_step_name):
        self.get_logger().info(f"Target tercapai. Hovering {self.hover_duration} detik...")
        self.hover_start_time = time.time()
        self.next_step_after_hover = next_step_name
        self.step = "HOVERING"

    def control_loop(self):
        if self.current_pose is None:
            return

        # 1. Publikasikan setpoint terus-menerus (Syarat wajib ArduPilot GUIDED/OFFBOARD)
        if self.step not in ["WAIT_CONN", "ARMING", "TAKEOFF", "LAND", "DONE"]:
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.target_pose)

        # 2. Logic Berdasarkan Sekuens
        if self.step == "WAIT_CONN":
            if self.current_state.connected:
                self.get_logger().info("FCU Terkoneksi. Memulai Setup Mode & Arming...")
                self.step = "ARMING"

        elif self.step == "ARMING":
            if self.current_state.mode != "GUIDED":
                req = SetMode.Request(custom_mode="GUIDED")
                self.mode_client.call_async(req)
            elif not self.current_state.armed:
                req = CommandBool.Request(value=True)
                self.arm_client.call_async(req)
            else:
                self.get_logger().info("Armed dan GUIDED. Bersiap Takeoff...")
                self.start_x = self.current_pose.pose.position.x
                self.start_y = self.current_pose.pose.position.y
                self.step = "TAKEOFF"

        elif self.step == "TAKEOFF":
            self.set_target(self.start_x, self.start_y, self.alt_target)
            req = CommandTOL.Request(altitude=self.alt_target)
            self.takeoff_client.call_async(req)
            self.get_logger().info(f"Takeoff ke {self.alt_target}m...")
            self.step = "WAIT_TAKEOFF"

        elif self.step == "WAIT_TAKEOFF":
            if abs(self.current_pose.pose.position.z - self.alt_target) < 0.5:
                # Kunci target pose ke titik saat ini setelah takeoff
                self.set_target(self.start_x, self.start_y, self.alt_target)
                self.trigger_hover("MAJU_3M")

        elif self.step == "HOVERING":
            if time.time() - self.hover_start_time > self.hover_duration:
                self.step = self.next_step_after_hover
                self.get_logger().info(f"Eksekusi manuver: {self.step}")

        elif self.step == "MAJU_3M":
            self.set_target(self.start_x + 3.0, self.start_y, self.alt_target)
            if self.distance_to_target() < self.dist_tolerance:
                self.trigger_hover("KIRI_3M")

        elif self.step == "KIRI_3M":
            self.set_target(self.start_x + 3.0, self.start_y + 3.0, self.alt_target)
            if self.distance_to_target() < self.dist_tolerance:
                self.trigger_hover("KANAN_3M")

        elif self.step == "KANAN_3M":
            # Kembali ke tengah (y = start_y)
            self.set_target(self.start_x + 3.0, self.start_y, self.alt_target)
            if self.distance_to_target() < self.dist_tolerance:
                self.trigger_hover("MUNDUR_3M")

        elif self.step == "MUNDUR_3M":
            self.set_target(self.start_x, self.start_y, self.alt_target)
            if self.distance_to_target() < self.dist_tolerance:
                self.trigger_hover("MAJU_LAGI_3M")

        elif self.step == "MAJU_LAGI_3M":
            self.set_target(self.start_x + 3.0, self.start_y, self.alt_target)
            if self.distance_to_target() < self.dist_tolerance:
                self.get_logger().info("Target tercapai. Bersiap simulasi Orbit Odom...")
                self.orbit_start_time = time.time()
                self.step = "ORBIT_TEST"

        elif self.step == "ORBIT_TEST":
            # Orbit buatan murni secara geometris
            # Pusat Lingkaran (Center) berada 2m di kiri titik saat ini agar transisinya mulus
            radius = 2.0
            cx = self.start_x + 3.0
            cy = self.start_y + radius 
            
            # Waktu berjalan menentukan sudut orbit (Kecepatan 0.5 rad/s)
            elapsed = time.time() - self.orbit_start_time
            angular_speed = 0.5 
            theta = -math.pi/2 + (elapsed * angular_speed) # Mulai dari dasar lingkaran
            
            # Kalkulasi posisi di lintasan lingkaran
            orbit_x = cx + radius * math.cos(theta)
            orbit_y = cy + radius * math.sin(theta)
            
            # Arahkan (Yaw) kamera selalu ke pusat lingkaran
            orbit_yaw = math.atan2(cy - orbit_y, cx - orbit_x)
            
            self.set_target(orbit_x, orbit_y, self.alt_target, yaw=orbit_yaw)
            
            # Selesai jika sudah berputar 360 derajat (2 * PI)
            if elapsed * angular_speed >= 2 * math.pi:
                self.get_logger().info("Orbit 360 derajat selesai. Meminta Pendaratan...")
                self.step = "LAND"

        elif self.step == "LAND":
            req = CommandTOL.Request()
            self.land_client.call_async(req)
            self.step = "DONE"
            self.get_logger().info("Mode pendaratan dieksekusi. Uji coba odometri SELESAI.")

def main(args=None):
    rclpy.init(args=args)
    node = OdomTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
