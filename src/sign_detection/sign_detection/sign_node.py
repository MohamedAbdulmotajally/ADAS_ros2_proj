
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import requests

class SignDetection(Node):
    def __init__(self):
        super().__init__('sign_detection_node')

        # Subscribe to the lane-detected frames
        self.subscription = self.create_subscription(
            Image,
            '/lane_output',
            self.process_frame,
            10
        )

        # Publisher for the processed frames
        self.publisher = self.create_publisher(Image, '/sign_output', 10)

        # Initialize CVBridge
        self.bridge = CvBridge()

        # Load YOLOv5 model for sign detection
        self.get_logger().info("Loading YOLOv5 model for sign detection...")
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='sign_detection_model.pt')

        # Live stream server endpoint (Running on Ubuntu Desktop)
        self.live_stream_url = "http://192.168.1.100:5000/upload"  # Change to your server IP

    def process_frame(self, msg):
        """ Process the incoming frame, detect traffic signs, and send it to the live stream. """
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLOv5 model on the frame
            results = self.model(frame)

            # Render detections on the frame
            output_frame = results.render()[0]

            # Convert processed frame back to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(output_frame, encoding='bgr8')

            # Publish the processed frame
            self.publisher.publish(image_msg)

            # Send the frame to the live stream server
            self.send_to_live_stream(output_frame)

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {str(e)}")

    def send_to_live_stream(self, frame):
        """ Send the processed frame to the live-streaming server. """
        try:
            _, buffer = cv2.imencode('.jpg', frame)
            response = requests.post(self.live_stream_url, files={"file": buffer.tobytes()})
            if response.status_code == 200:
                self.get_logger().info("Frame sent to live stream successfully.")
            else:
                self.get_logger().error(f"Failed to send frame: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error sending frame to live stream: {str(e)}")

def main():
    rclpy.init()
    node = SignDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
