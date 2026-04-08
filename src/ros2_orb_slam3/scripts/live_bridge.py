import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
import time

class LiveBridge(Node):
    def __init__(self):
        super().__init__('live_bridge')
        
        # 1. Subscribes to YOUR working camera node
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        
        # 2. Publishes to the SLAM "Doors" we found earlier
        self.img_pub = self.create_publisher(Image, '/mono_py_driver/img_msg', 10)
        self.time_pub = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', 10)
        self.handshake_pub = self.create_publisher(String, '/mono_py_driver/experiment_settings', 10)

        # 3. Automatic Handshake after 2 seconds
        self.timer = self.create_timer(2.0, self.send_handshake)
        self.handshake_sent = False
        self.get_logger().info("Bridge Ready. Waiting to send handshake...")

    def send_handshake(self):
        if not self.handshake_sent:
            msg = String()
            msg.data = 'EuRoC'
            self.handshake_pub.publish(msg)
            self.get_logger().info("Handshake Sent to SLAM engine!")
            self.handshake_sent = True

    def callback(self, msg):
        # Forward the image
        self.img_pub.publish(msg)
        
        # Forward the timestamp in the specific Float64 format it wants
        t_msg = Float64()
        t_msg.data = float(msg.header.stamp.sec) + (float(msg.header.stamp.nanosec) / 1e9)
        self.time_pub.publish(t_msg)

def main():
    rclpy.init()
    rclpy.spin(LiveBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
