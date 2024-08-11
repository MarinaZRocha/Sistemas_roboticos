#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

from example_interfaces.msg import String

class TurtlebotCtrl(Node):
	def __init__(self):
		super().__init__("TurtlebotCtrl")

		self.laser = LaserScan()
		self.odom = Odometry()

		self.map = np.array([	[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
								[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
								[0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
								[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
						])

		self.publish_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
		self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
		self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)
		self.timer = self.create_timer(0.5, self.cmd_vel_pub)

	def cmd_vel_pub(self):

		map_resolution = 4

		index_x = -int(self.odom.pose.pose.position.x*map_resolution)
		index_y = -int(self.odom.pose.pose.position.y*map_resolution)

		index_x += int(self.map.shape[0]/2)
		index_y += int(self.map.shape[0]/2)

		if (index_x < 1): index_x = 1
		if (index_x > self.map.shape[0]-1): index_x = self.map.shape[0]-1
		if (index_y < 1): index_y = 1
		if (index_y > self.map.shape[0]-1): index_y = self.map.shape[0]-1

		if (self.map[index_x][index_y] == 1):
			self.map[index_x][index_y] = 2

			self.get_logger().info("Another part reached ... percentage total reached...." + str(100*float(np.count_nonzero(self.map == 2))/(np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2))) )
			self.get_logger().info("Discrete Map")
			self.get_logger().info("\n"+str(self.map))

        # Desenvlva seu codigo aqui

		msg = Twist()
		msg.linear.x = 0.1
		msg.angular.z = 0.1
		self.publish_cmd_vel.publish(msg)

	def callback_laser(self, msg):
		self.laser = msg

	def callback_odom(self, msg):
		self.odom = msg

def main(args=None):
	rclpy.init(args=args)
	node = TurtlebotCtrl()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
