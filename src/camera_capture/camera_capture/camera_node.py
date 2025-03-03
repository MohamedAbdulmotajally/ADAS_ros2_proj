import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraCapture(Node):
    def __init__(self):
        super().__init__('camera_capture_node')
        self.publisher = self.create_publisher(Image, '/camera_frame', 10)
        self.cap = cv2.VideoCapture(0)  # USB camera
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.capture_frame)

    def capture_frame(self):
        ret, frame = self.cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(image_msg)
            self.get_logger().info("Publishing camera frame")

def main():
    rclpy.init()
    node = CameraCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
