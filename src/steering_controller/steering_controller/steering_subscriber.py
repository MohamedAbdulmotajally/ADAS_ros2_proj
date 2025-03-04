
import rclpy
from rclpy.node import Node
import spidev
from std_msgs.msg import Float32MultiArray

class SteeringSubscriber(Node):
    def __init__(self):
        super().__init__('steering_subscriber')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/steering_data',
            self.send_to_tiva,
            10)
        
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 500000

    def send_to_tiva(self, msg):
        direction, angle, paddle_left, paddle_right = msg.data
        spi_data = [int(direction), int(angle), int(paddle_left), int(paddle_right)]
        self.spi.xfer(spi_data)

        self.get_logger().info(f"Sent to Tiva: {spi_data}")

def main():
    rclpy.init()
    node = SteeringSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
