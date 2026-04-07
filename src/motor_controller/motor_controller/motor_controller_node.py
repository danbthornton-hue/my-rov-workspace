import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import board
from adafruit_motorkit import MotorKit
from gpiozero import Motor, PWMOutputDevice


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ---------------- MOTOR SETUP ----------------
        self.kit = MotorKit(i2c=board.I2C())

        # L298N motors (Back Section: MA=BL, MB=BR)
        self.motorA = Motor(forward=17, backward=27) # BL
        self.motorB = Motor(forward=24, backward=23) # BR
        self.enableA = PWMOutputDevice(22)
        self.enableB = PWMOutputDevice(25)

        # ---------------- TENSION PARAMETERS ----------------
        self.pull_power = 1.0   # The leading section
        self.mid_power = 0.6    # The support section
        self.drag_power = 0.3   # The trailing section
        
        # Turn parameters
        self.turn_lead_power = 0.9      # Front power (The "Tug")
        self.mid_counter_steer = 0.15   # Mid counter-push
        self.back_turn_power = 0.20     # Back follow power (Reduced to prevent jackknife)

    # ---------------- DIRECTION LOGIC (M1=FR, M2=FL, M4=ML, M3=MR) ----------------

    def set_front(self, speed):
        # M2=FL (+ fwd), M1=FR (- fwd)
        self.kit.motor2.throttle = speed
        self.kit.motor1.throttle = -speed

    def set_mid(self, speed):
        # M4=ML (+ fwd), M3=MR (- fwd)
        self.kit.motor4.throttle = speed
        self.kit.motor3.throttle = -speed

    def set_back(self, speed):
        # MA=BL, MB=BR (L298N)
        if speed > 0:
            self.motorA.forward()
            self.motorB.forward()
        elif speed < 0:
            self.motorA.backward()
            self.motorB.backward()
        else:
            self.motorA.stop()
            self.motorB.stop()
        
        self.enableA.value = abs(speed)
        self.enableB.value = abs(speed)

    def stop_all(self):
        self.kit.motor1.throttle = 0
        self.kit.motor2.throttle = 0
        self.kit.motor3.throttle = 0
        self.kit.motor4.throttle = 0
        self.enableA.value = 0
        self.enableB.value = 0
        self.motorA.stop()
        self.motorB.stop()

    # ---------------- MOVEMENT STATES ----------------

    def move_forward(self):
        self.set_front(self.pull_power)
        self.set_mid(self.mid_power)
        self.set_back(self.drag_power)

    def move_backward(self):
        self.set_back(-self.pull_power)
        self.set_mid(-self.mid_power)
        self.set_front(-self.drag_power)

    def turn_left(self):
        """
        ROBOT PIVOTS LEFT: 
        Front: Leads with high power (FL Back, FR Fwd)
        Back:  Follows with low power (BL Back, BR Fwd)
        Mid:   Counter-steers (ML Fwd, MR Back)
        """
        # Front (High Power)
        self.kit.motor2.throttle = -self.turn_lead_power # FL Back
        self.kit.motor1.throttle = -self.turn_lead_power # FR Fwd
        
        # Back (Low Follow Power)
        self.motorA.backward() # BL Back
        self.motorB.forward()  # BR Fwd
        self.enableA.value = self.back_turn_power
        self.enableB.value = self.back_turn_power

        # Middle Counter-Steer
        self.kit.motor4.throttle = self.mid_counter_steer  # ML Fwd
        self.kit.motor3.throttle = self.mid_counter_steer  # MR Back

    def turn_right(self):
        """
        ROBOT PIVOTS RIGHT:
        Front: Leads with high power (FL Fwd, FR Back)
        Back:  Follows with low power (BL Fwd, BR Back)
        Mid:   Counter-steers (ML Back, MR Fwd)
        """
        # Front (High Power)
        self.kit.motor2.throttle = self.turn_lead_power  # FL Fwd
        self.kit.motor1.throttle = self.turn_lead_power  # FR Back
        
        # Back (Low Follow Power)
        self.motorA.forward()  # BL Fwd
        self.motorB.backward() # BR Back
        self.enableA.value = self.back_turn_power
        self.enableB.value = self.back_turn_power

        # Middle Counter-Steer
        self.kit.motor4.throttle = -self.mid_counter_steer # ML Back
        self.kit.motor3.throttle = -self.mid_counter_steer # MR Fwd

    # ---------------- CALLBACK ----------------

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if linear > 0:
            self.move_forward()
        elif linear < 0:
            self.move_backward()
        elif angular > 0:
            self.turn_left()
        elif angular < 0:
            self.turn_right()
        else:
            self.stop_all()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()