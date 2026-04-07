#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range, LaserScan
import time
import math

class RovNavNode(Node):
    def __init__(self):
        super().__init__('rovnav_node')
        
        # ==========================================
        # 1. TUNABLE PARAMETERS & OFFSETS
        # ==========================================
        self.US_THRESHOLD = 0.5
        self.US_OFFSET_1 = 0.0
        self.US_OFFSET_2 = 0.2
        self.FLAT_WALL_TOL = 0.1
        
        # SLAM specific parameters
        self.SLAM_SCALE_OFFSET = 5.0   # Multiply SLAM ranges by this to get actual meters
        self.SLAM_TIMEOUT = 1.0        # If no SLAM data for 1 sec, fallback to US only
        self.SLAM_VFH_THRESHOLD = 1.0  # Distance at which SLAM starts "leaning" the rover
        
        # Timing parameters
        self.TIME_BACKUP = 1.5
        self.TIME_TURN = 2.0
        
        # ==========================================
        # 2. MOVEMENT SETTINGS (EASY TO EDIT)
        # ==========================================
        self.SPEED_FORWARD = {'lin': 0.3, 'ang': 0.0}
        self.SPEED_BACKUP  = {'lin': -0.2, 'ang': 0.0}
        
        # Diagonal/Sweeping base settings (used for parallel turns)
        self.SWEEP_LEFT    = {'lin': 0.2, 'ang': 0.4}
        self.SWEEP_RIGHT   = {'lin': 0.2, 'ang': -0.4}
        
        # ==========================================
        # 3. ROS 2 SETUP
        # ==========================================
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_d1 = self.create_subscription(Range, '/distance1', self.d1_callback, 10)
        self.sub_d2 = self.create_subscription(Range, '/distance2', self.d2_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/orb_slam3/pose', self.pose_callback, 10)
        
        # Internal Variables
        self.dist_left = 10.0
        self.dist_right = 10.0
        self.slam_active = False
        self.last_slam_time = 0.0
        self.current_yaw = 0.0         # Robot's current heading from /pose
        
        self.vfh_lean_angle = 0.0      # Angular velocity calculated by SLAM
        
        self.state = "FORWARD"
        self.next_state_after_backup = "SWEEP_RIGHT"
        self.state_timer_end = 0.0
        
        self.create_timer(0.1, self.logic_loop)
        self.get_logger().info("ROVNAV Node Started: Hybrid Mode Active.")

    # --- Sensor Callbacks ---
    def d1_callback(self, msg): self.dist_left = msg.range + self.US_OFFSET_1
    def d2_callback(self, msg): self.dist_right = msg.range + self.US_OFFSET_2

    def pose_callback(self, msg):
        # Convert quaternion to yaw (Z-axis rotation) to keep track of direction
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.last_slam_time = time.time()
        self.slam_active = True
        
        # Simple Vector Field Histogram (VFH) Logic
        # Divide the scan into Left and Right halves to calculate "Lean"
        mid_idx = len(msg.ranges) // 2
        left_slice = msg.ranges[:mid_idx]
        right_slice = msg.ranges[mid_idx:]
        
        # Find closest point in each slice, apply the SLAM_SCALE_OFFSET
        closest_left = min(left_slice) * self.SLAM_SCALE_OFFSET if left_slice else float('inf')
        closest_right = min(right_slice) * self.SLAM_SCALE_OFFSET if right_slice else float('inf')
        
        # Calculate lean. If left is blocked, lean right (negative angular.z)
        self.vfh_lean_angle = 0.0
        if closest_left < self.SLAM_VFH_THRESHOLD:
            self.vfh_lean_angle = -0.3 # Lean Right
        elif closest_right < self.SLAM_VFH_THRESHOLD:
            self.vfh_lean_angle = 0.3  # Lean Left

    # --- Movement Helper ---
    def move(self, speed_dict, override_angular=None):
        t = Twist()
        t.linear.x = float(speed_dict['lin'])
        # If SLAM wants us to lean, override the straight angular velocity
        if override_angular is not None:
            t.angular.z = float(override_angular)
        else:
            t.angular.z = float(speed_dict['ang'])
        self.pub_cmd.publish(t)

    # --- Main State Machine ---
    def logic_loop(self):
        current_time = time.time()
        
        # Timeout Check: Did SLAM stop sending data?
        if (current_time - self.last_slam_time) > self.SLAM_TIMEOUT:
            if self.slam_active:
                self.get_logger().warn("SLAM TIMEOUT! Falling back to Ultrasonic Only.")
            self.slam_active = False
            self.vfh_lean_angle = 0.0 # Stop leaning if SLAM is dead
            
        # ==================================
        # STATE: FORWARD (Driving & Leaning)
        # ==================================
        if self.state == "FORWARD":
            # 1. Base Layer: Drive forward, apply SLAM lean if active
            self.move(self.SPEED_FORWARD, override_angular=self.vfh_lean_angle)
            
            # 2. Emergency Layer: Ultrasonics override everything
            if self.dist_left < self.US_THRESHOLD or self.dist_right < self.US_THRESHOLD:
                self.get_logger().info("US OVERRIDE: Obstacle detected! Backing up.")
                self.state = "BACKUP"
                self.state_timer_end = current_time + self.TIME_BACKUP
                
                # Determine slant
                diff = abs(self.dist_left - self.dist_right)
                if diff <= self.FLAT_WALL_TOL:
                    self.next_state_after_backup = "SWEEP_RIGHT"
                elif self.dist_left < self.dist_right:
                    self.next_state_after_backup = "SWEEP_RIGHT" # Left blocked, sweep right
                else:
                    self.next_state_after_backup = "SWEEP_LEFT"  # Right blocked, sweep left
                    
        # ==================================
        # STATE: BACKUP
        # ==================================
        elif self.state == "BACKUP":
            self.move(self.SPEED_BACKUP)
            if current_time >= self.state_timer_end:
                self.state = self.next_state_after_backup
                self.state_timer_end = current_time + self.TIME_TURN
                
        # ==================================
        # STATES: SWEEP (Parallel Turns)
        # ==================================
        elif self.state == "SWEEP_LEFT":
            self.move(self.SWEEP_LEFT)
            if current_time >= self.state_timer_end:
                self.state = "FORWARD"
                
        elif self.state == "SWEEP_RIGHT":
            self.move(self.SWEEP_RIGHT)
            if current_time >= self.state_timer_end:
                self.state = "FORWARD"

def main(args=None):
    rclpy.init(args=args)
    node = RovNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()