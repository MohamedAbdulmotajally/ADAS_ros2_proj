import rclpy
from rclpy.node import Node
import torch
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.subscription = self.create_subscription(Image, '/camera_frame', self.process_frame, 10)
        self.publisher = self.create_publisher(Image, '/lane_output', 10)
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='lane_detection_model.pt', force_reload=True)


    def process_frame(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)
        output = results.render()[0]  # Processed frame
        image_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        self.publisher.publish(image_msg)

def main():
    rclpy.init()
    node = LaneDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
