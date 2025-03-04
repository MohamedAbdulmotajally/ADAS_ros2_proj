
import rclpy
from rclpy.node import Node
import pygame
from std_msgs.msg import Float32MultiArray

class SteeringPublisher(Node):
    def __init__(self):
        super().__init__('steering_publisher')

        # Initialize pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No steering wheel detected!")
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected to {self.joystick.get_name()}")

        self.publisher = self.create_publisher(Float32MultiArray, '/steering_data', 10)
        self.timer = self.create_timer(0.02, self.read_steering_data)  # 50Hz

    def read_steering_data(self):
        pygame.event.pump()

        direction = self.joystick.get_axis(0) * 100
        angle = self.joystick.get_axis(1) * 100
        paddle_left = self.joystick.get_button(4)
        paddle_right = self.joystick.get_button(5)

        msg = Float32MultiArray()
        msg.data = [direction, angle, paddle_left, paddle_right]
        self.publisher.publish(msg)

        self.get_logger().info(f"Direction: {direction}, Angle: {angle}, Paddle L: {paddle_left}, Paddle R: {paddle_right}")

def main():
    rclpy.init()
    node = SteeringPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
