import rclpy
from rclpy.node import Node
import board
from adafruit_motorkit import MotorKit
from gpiozero import Motor, PWMOutputDevice

class MotorsForward(Node):
    def __init__(self):
        super().__init__('motors_forward')
        
        # 1. Initialize Hardware
        self.get_logger().info('Initializing hardware for forward test...')
        try:
            self.kit = MotorKit(i2c=board.I2C())
            self.motorA = Motor(forward=17, backward=27)
            self.motorB = Motor(forward=23, backward=24)
            self.enableA = PWMOutputDevice(22)
            self.enableB = PWMOutputDevice(25)
            
            # 2. Start Moving
            self.start_moving_forward()
        except Exception as e:
            self.get_logger().error(f'Failed to start motors: {e}')

    def start_moving_forward(self):
        speed = 0.7  # 70% power
        self.get_logger().info(f'🚀 All 6 motors driving FORWARD at {speed*100}% power.')
        self.get_logger().warn('Press Ctrl+C to stop the robot!')

        # Adafruit MotorKit (4 motors)
        # Note: If some motors spin backward, change the sign to -speed
        self.kit.motor1.throttle = speed
        self.kit.motor2.throttle = speed
        self.kit.motor3.throttle = speed
        self.kit.motor4.throttle = speed

        # L298N (2 motors)
        self.enableA.value = speed
        self.motorA.forward()
        self.enableB.value = speed
        self.motorB.forward()

    def stop_all_motors(self):
        self.get_logger().info('🛑 Stopping all motors...')
        self.kit.motor1.throttle = 0.0
        self.kit.motor2.throttle = 0.0
        self.kit.motor3.throttle = 0.0
        self.kit.motor4.throttle = 0.0
        self.motorA.stop()
        self.motorB.stop()
        self.enableA.value = 0.0
        self.enableB.value = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = MotorsForward()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This catches the Ctrl+C and shuts things down safely
        node.stop_all_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
