import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from gpiozero import DistanceSensor

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')
        
        # Initialize the GPIO Zero sensor
        # Trig = 16, Echo = 5
        self.sensor = DistanceSensor(echo=5, trigger=16)
        
        # Create publisher for /distance1
        self.publisher_ = self.create_publisher(Range, '/distance1', 10)
        
        # Create a timer that runs every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Distance Sensor Node has started (Trig: 16, Echo: 5)')

    def timer_callback(self):
        msg = Range()
        
        # Standard ROS 2 header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_sensor_link'
        
        # Range message specs
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.5  # Approx 30 degrees in radians
        msg.min_range = 0.02     # 2cm
        msg.max_range = 4.0      # 4m
        
        # Get distance from gpiozero (returns value in meters)
        try:
            msg.range = float(self.sensor.distance)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {msg.range:.2f} m')
        except Exception as e:
            self.get_logger().error(f'Sensor read failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()