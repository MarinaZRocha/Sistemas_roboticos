#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String
from geometry_msgs.msg import Twist

class Publisher(Node):
      def __init__(self):
            super().__init__("publisher")

            self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.publish_hello)
            self.get_logger().info("Publisher started")

      def publish_hello(self):
            msg = Twist()
            msg.linear.x = 0.2
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=None)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
      main()