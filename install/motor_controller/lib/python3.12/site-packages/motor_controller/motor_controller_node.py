import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import board
from adafruit_motorkit import MotorKit
from gpiozero import Motor, PWMOutputDevice

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ---------------- MOTOR SETUP ----------------
        self.kit = MotorKit(i2c=board.I2C())

        # L298N motors (Middle Section)
        self.motorA = Motor(forward=17, backward=27)
        self.motorB = Motor(forward=24, backward=23)

        self.enableA = PWMOutputDevice(22)
        self.enableB = PWMOutputDevice(25)

        # ==========================================
        # 1. HARDWARE MAPPING & INVERSIONS
        # ==========================================
        # Based on your previous code, some motors need negative values to go forward.
        # User note: Middle section must be fully reversed for skid-steer mechanics.
        self.HW_INV = {
            'FL': -1.0,  # Front Left (Motor 1)
            'FR':  1.0,  # Front Right (Motor 2)
            'ML': -1.0,  # Middle Left (Motor A - L298N) - Reversed per instructions
            'MR': -1.0,  # Middle Right (Motor B - L298N) - Reversed per instructions
            'BL': -1.0,  # Back Left (Motor 3)
            'BR':  1.0   # Back Right (Motor 4)
        }

        # Base multiplier to easily speed up/slow down the entire rover
        self.MASTER_SPEED = 0.8 

        # ==========================================
        # 2. MOVEMENT PROFILES (THE TUNING ZONE)
        # ==========================================
        # Positive = Forward, Negative = Backward. 
        # Range: -1.0 to 1.0
        # Physics Fix: Front pulls hardest (prevents dip), Rear drags slightly (prevents jackknife)
        
        self.V_FORWARD = {
            'FL': 1.0,  'FR': 1.0,
            'ML': 0.9,  'MR': 0.9,
            'BL': 0.8,  'BR': 0.8
        }

        self.V_BACKWARD = {
            'FL': -0.8, 'FR': -0.8, # Front drags
            'ML': -0.9, 'MR': -0.9,
            'BL': -1.0, 'BR': -1.0  # Rear pulls hardest in reverse
        }

        self.V_TURN_LEFT_IN_PLACE = {
            'FL': -0.8, 'FR': 0.8,
            'ML': -1.0, 'MR': 1.0,
            'BL': -0.8, 'BR': 0.8
        }

        self.V_TURN_RIGHT_IN_PLACE = {
            'FL': 0.8,  'FR': -0.8,
            'ML': 1.0,  'MR': -1.0,
            'BL': 0.8,  'BR': -0.8
        }

        # SWEEPING TURNS (Moving Forward while turning)
        # Outside wheels spin fast, inside wheels spin slow
        self.V_SWEEP_FWD_LEFT = {
            'FL': 0.3,  'FR': 1.0,
            'ML': 0.2,  'MR': 0.9,
            'BL': 0.1,  'BR': 0.8
        }

        self.V_SWEEP_FWD_RIGHT = {
            'FL': 1.0,  'FR': 0.3,
            'ML': 0.9,  'MR': 0.2,
            'BL': 0.8,  'BR': 0.1
        }

        # SWEEPING TURNS (Moving Backward while turning)
        self.V_SWEEP_BWD_LEFT = {
            'FL': -0.1, 'FR': -0.8,
            'ML': -0.2, 'MR': -0.9,
            'BL': -0.3, 'BR': -1.0
        }

        self.V_SWEEP_BWD_RIGHT = {
            'FL': -0.8, 'FR': -0.1,
            'ML': -0.9, 'MR': -0.2,
            'BL': -1.0, 'BR': -0.3
        }

        self.V_STOP = {
            'FL': 0.0, 'FR': 0.0,
            'ML': 0.0, 'MR': 0.0,
            'BL': 0.0, 'BR': 0.0
        }

    # ---------------- LOW-LEVEL MOTOR EXECUTION ----------------
    def apply_speeds(self, profile):
        """ Translates high-level percentages to hardware-specific commands. """
        
        # Calculate raw speeds with Master multiplier and Hardware Inversions
        fl_raw = profile['FL'] * self.MASTER_SPEED * self.HW_INV['FL']
        fr_raw = profile['FR'] * self.MASTER_SPEED * self.HW_INV['FR']
        bl_raw = profile['BL'] * self.MASTER_SPEED * self.HW_INV['BL']
        br_raw = profile['BR'] * self.MASTER_SPEED * self.HW_INV['BR']
        
        ml_raw = profile['ML'] * self.MASTER_SPEED * self.HW_INV['ML']
        mr_raw = profile['MR'] * self.MASTER_SPEED * self.HW_INV['MR']

        # 1. Apply to I2C Motors (Expects -1.0 to 1.0)
        # Using max/min to clamp values safely
        self.kit.motor1.throttle = max(min(fl_raw, 1.0), -1.0)
        self.kit.motor2.throttle = max(min(fr_raw, 1.0), -1.0)
        self.kit.motor3.throttle = max(min(bl_raw, 1.0), -1.0)
        self.kit.motor4.throttle = max(min(br_raw, 1.0), -1.0)

        # 2. Apply to L298N Motors (gpiozero Motor expects 0.0 to 1.0)
        self._set_l298n(self.motorA, self.enableA, ml_raw)
        self._set_l298n(self.motorB, self.enableB, mr_raw)

    def _set_l298n(self, motor, enable, speed):
        """ Helper to handle the L298N directional PWM logic """
        clamped_speed = max(min(speed, 1.0), -1.0)
        
        if clamped_speed > 0:
            motor.forward(clamped_speed)
            enable.value = 1
        elif clamped_speed < 0:
            motor.backward(abs(clamped_speed)) # backward() requires a positive number
            enable.value = 1
        else:
            motor.stop()
            enable.value = 0

    # ---------------- CMD_VEL CALLBACK ----------------
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Decode Twist message into specific states
        if linear > 0 and angular == 0:
            self.apply_speeds(self.V_FORWARD)
            
        elif linear < 0 and angular == 0:
            self.apply_speeds(self.V_BACKWARD)
            
        elif linear == 0 and angular > 0:
            self.apply_speeds(self.V_TURN_LEFT_IN_PLACE)
            
        elif linear == 0 and angular < 0:
            self.apply_speeds(self.V_TURN_RIGHT_IN_PLACE)
            
        elif linear > 0 and angular > 0:
            self.apply_speeds(self.V_SWEEP_FWD_LEFT)
            
        elif linear > 0 and angular < 0:
            self.apply_speeds(self.V_SWEEP_FWD_RIGHT)
            
        elif linear < 0 and angular > 0:
            # Reversing while turning left means the rear swings to the right
            self.apply_speeds(self.V_SWEEP_BWD_LEFT)
            
        elif linear < 0 and angular < 0:
            self.apply_speeds(self.V_SWEEP_BWD_RIGHT)
            
        else:
            self.apply_speeds(self.V_STOP)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()