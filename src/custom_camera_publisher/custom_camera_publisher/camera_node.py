import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading

class RpiCamPublisher(Node):
    def __init__(self):
        super().__init__('rpicam_publisher')
        
        # Publishers
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.time_pub = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', 10)
        
        self.bridge = CvBridge()
        
        # --- CAMERA SETUP ---
        # 0 is the default index for the RPi Camera
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        # Set resolution to match ORB-SLAM3 expectations (EuRoC size)
        self.width = 752
        self.height = 480
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open video device. Is another process using it?")
            return

        self.get_logger().info(f"RPi Camera started at {self.width}x{self.height}")
        
        # Start background thread for capturing
        self.running = True
        self.thread = threading.Thread(target=self.capture_loop)
        self.thread.start()

    def capture_loop(self):
        while self.running and rclpy.ok():
            ret, frame = self.cap.read()
            
            if not ret:
                continue

            # Get current ROS time for the header
            now = self.get_clock().now()
            
            # ORB-SLAM3 and the Calibrator process Grayscale images
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # --- PUBLISH IMAGE ---
            try:
                msg = self.bridge.cv2_to_imgmsg(gray, encoding="mono8")
                msg.header.stamp = now.to_msg()
                msg.header.frame_id = "camera_frame"
                self.publisher_.publish(msg)

                # --- PUBLISH TIMESTAMP (for ORB-SLAM3 Wrapper) ---
                time_msg = Float64()
                time_msg.data = now.nanoseconds / 1e9
                self.time_pub.publish(time_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish: {e}")

    def stop(self):
        self.running = False
        if self.cap.isOpened():
            self.cap.release()
        self.thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = RpiCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
