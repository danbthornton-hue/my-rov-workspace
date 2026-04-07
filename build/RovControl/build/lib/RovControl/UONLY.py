#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import time

class UsOnlyNode(Node):
    def __init__(self):
        super().__init__('uonly_node')
        
        # ==========================================
        # 1. TUNABLE PARAMETERS
        # ==========================================
        self.US_THRESHOLD = 0.3   # Meters to trigger obstacle avoidance
        self.US_OFFSET_1 = 0.0    # Left sensor calibration
        self.US_OFFSET_2 = 0.1    # Right sensor calibration
        
        # State Timing
        self.TIME_BACKUP = 1.2    # How long to reverse
        self.TIME_TURN   = 0.75   # How long to spin (adjust for a 90-degree turn)
        
        # ==========================================
        # 2. MOVEMENT COMMANDS (STRAIGHT SPINS)
        # ==========================================
        # lin: 0.0 ensures it spins in place (Tank Turn)
        self.CMD_FORWARD  = {'lin': 0.3,  'ang': 0.0}
        self.CMD_BACKUP   = {'lin': -0.2, 'ang': 0.0}
        self.CMD_HARD_L   = {'lin': 0.0,  'ang': 0.6} # Spin left in place
        self.CMD_HARD_R   = {'lin': 0.0,  'ang': -0.6} # Spin right in place
        self.CMD_STOP     = {'lin': 0.0,  'ang': 0.0}

        # ==========================================
        # 3. ROS 2 SETUP
        # ==========================================
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_d1 = self.create_subscription(Range, '/distance1', self.d1_callback, 10)
        self.sub_d2 = self.create_subscription(Range, '/distance2', self.d2_callback, 10)
        
        self.dist_left = 10.0
        self.dist_right = 10.0
        self.state = "FORWARD"
        self.next_turn_state = "TURN_RIGHT"
        self.state_timer_end = 0.0
        
        self.create_timer(0.1, self.logic_loop)
        self.get_logger().info("UONLY Node Started: Hard Turn Mode Active.")

    def d1_callback(self, msg):
        self.dist_left = msg.range + self.US_OFFSET_1

    def d2_callback(self, msg):
        self.dist_right = msg.range + self.US_OFFSET_2

    def move(self, speed_dict):
        t = Twist()
        t.linear.x = float(speed_dict['lin'])
        t.angular.z = float(speed_dict['ang'])
        self.pub_cmd.publish(t)

    def logic_loop(self):
        current_time = time.time()
        
        if self.state == "FORWARD":
            self.move(self.CMD_FORWARD)
            
            # Obstacle Check
            if self.dist_left < self.US_THRESHOLD or self.dist_right < self.US_THRESHOLD:
                self.get_logger().info("Obstacle! Choosing turn direction...")
                
                # Decision: Spin away from the closest object
                if self.dist_left < self.dist_right:
                    self.next_turn_state = "TURN_RIGHT" # Left is blocked, spin right
                else:
                    self.next_turn_state = "TURN_LEFT"  # Right is blocked, spin left
                
                self.state = "BACKUP"
                self.state_timer_end = current_time + self.TIME_BACKUP
                    
        elif self.state == "BACKUP":
            self.move(self.CMD_BACKUP)
            if current_time >= self.state_timer_end:
                self.state = self.next_turn_state
                self.state_timer_end = current_time + self.TIME_TURN
                self.get_logger().info(f"Backing up done. Spinning: {self.state}")
                
        elif self.state == "TURN_LEFT":
            self.move(self.CMD_HARD_L)
            if current_time >= self.state_timer_end:
                self.state = "FORWARD"
                
        elif self.state == "TURN_RIGHT":
            self.move(self.CMD_HARD_R)
            if current_time >= self.state_timer_end:
                self.state = "FORWARD"

def main(args=None):
    rclpy.init(args=args)
    node = UsOnlyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()