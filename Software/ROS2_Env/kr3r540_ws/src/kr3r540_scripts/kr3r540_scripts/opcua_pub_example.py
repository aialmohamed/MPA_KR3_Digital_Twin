# random_number_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import time

class RandomNumberPublisher(Node):
    def __init__(self):
        super().__init__('random_number_publisher')
        self.publisher_ = self.create_publisher(Float32, 'random_number', 10)
        self.timer = self.create_timer(10.0, self.publish_random_number)

    def publish_random_number(self):
        random_number = Float32()
        random_number.data = random.uniform(0, 100)  # Random number between 0 and 100
        self.publisher_.publish(random_number)
        self.get_logger().info(f"Published: {random_number.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomNumberPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
