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

        # L298N motors (example: adjust pins if needed)
        self.motorA = Motor(forward=17,backward=27)
        self.motorB = Motor(forward=24, backward=23)

        self.enableA = PWMOutputDevice(22)
        self.enableB = PWMOutputDevice(25)

        self.speed = 0.7

    # ---------------- LOW-LEVEL MOTOR FUNCTIONS ----------------
    # These are what you will tweak when testing

    def motor1_forward(self):
        self.kit.motor1.throttle = -self.speed

    def motor1_backward(self):
        self.kit.motor1.throttle = self.speed

    def motor1_stop(self):
        self.kit.motor1.throttle = 0

    def motor2_forward(self):
        self.kit.motor2.throttle = self.speed

    def motor2_backward(self):
        self.kit.motor2.throttle = -self.speed

    def motor2_stop(self):
        self.kit.motor2.throttle = 0

    def motor3_forward(self):
        self.kit.motor3.throttle = self.speed

    def motor3_backward(self):
        self.kit.motor3.throttle = -self.speed

    def motor3_stop(self):
        self.kit.motor3.throttle = 0

    def motor4_forward(self):
        self.kit.motor4.throttle = -self.speed

    def motor4_backward(self):
        self.kit.motor4.throttle = self.speed

    def motor4_stop(self):
        self.kit.motor4.throttle = 0

    # L298N motors
    def motor5_forward(self):
        self.motorA.forward(self.speed)
        self.enableA.value = 1

    def motor5_backward(self):
        self.motorA.backward(self.speed)
        self.enableA.value = 1

    def motor5_stop(self):
        self.motorA.stop()
        self.enableA.value = 0

    def motor6_forward(self):
        self.motorB.forward(self.speed)
        self.enableB.value = 1

    def motor6_backward(self):
        self.motorB.backward(self.speed)
        self.enableB.value = 1

    def motor6_stop(self):
        self.motorB.stop()
        self.enableB.value = 0

    # ---------------- HIGH-LEVEL MOVEMENT ----------------
    # These call the individual motor functions

    def move_forward(self):
        self.motor1_forward()
        self.motor2_forward()
        self.motor3_forward()
        self.motor4_forward()
        self.motor5_forward()
        self.motor6_forward()

    def move_backward(self):
        self.motor1_backward()
        self.motor2_backward()
        self.motor3_backward()
        self.motor4_backward()
        self.motor5_backward()
        self.motor6_backward()

    def turn_left(self):
        self.motor1_backward()
        self.motor2_forward()
        self.motor3_forward()
        self.motor4_backward()
        self.motor5_forward()
        self.motor6_backward()

    def turn_right(self):
        self.motor1_forward()
        self.motor2_backward()
        self.motor3_backward()
        self.motor4_forward()
        self.motor5_backward()
        self.motor6_forward()

    def stop_all(self):
        self.motor1_stop()
        self.motor2_stop()
        self.motor3_stop()
        self.motor4_stop()
        self.motor5_stop()
        self.motor6_stop()

    # ---------------- CMD_VEL CALLBACK ----------------

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
