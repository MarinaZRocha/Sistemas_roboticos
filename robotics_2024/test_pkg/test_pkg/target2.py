#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tf_transformations
import math
import time

class MoveTurtle(Node):

    def __init__(self):
        super().__init__('move_turtle_forward')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # publisher to send data to the cmd_vel topic
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)  # subscriber to receive current position values of the robot
        self.current_pose = PoseStamped()  
        self.goal = (-2.0, -5.0)  # You can set the goal here or pass it as a parameter to the class

        # Start navigation in a separate thread
        self.timer = self.create_timer(0.2, self.navigate_to_goal)

    def odom_callback(self, odom):  # function to update odometry values
        self.current_pose.header = odom.header
        self.current_pose.pose = odom.pose.pose

    def quaternion_to_yaw(self, quaternion):  # function to convert from euler to quaternion to obtain the yaw rotation
        euler = tf_transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler[2]  # yaw

    def velocity_control(self, vel_x, vel_z):  # function to make the robot move
        velocity_msg = Twist()
        velocity_msg.linear.x = vel_x
        velocity_msg.angular.z = vel_z
        self.velocity_publisher.publish(velocity_msg)

    def navigate_to_goal(self):  # main function to calculate current and desired distances and rotation
        x_target, y_target = self.goal
        x_pose = self.current_pose.pose.position.x
        y_pose = self.current_pose.pose.position.y

        x_distance = x_target - x_pose
        y_distance = y_target - y_pose

        theta = math.atan2(y_distance, x_distance)
        point_distance = math.sqrt(x_distance ** 2 + y_distance ** 2)
        angle_distance = theta - self.quaternion_to_yaw(self.current_pose.pose.orientation)

        if point_distance <= 0.3:  # if the distance is acceptable, print a message saying that it reached the target
            self.get_logger().info("Chegueei!")
            self.velocity_control(0.0, 0.0)  # stop the robot until moving to the next target
            rclpy.shutdown()  # send a signal to stop the script
        else:
            self.velocity_control(0.1 * point_distance, 0.2 * angle_distance)  # while the distance is still large, the speed is given by the multiplication of a constant

def main(args=None):
    rclpy.init(args=args)
    move_turtle = MoveTurtle()
    rclpy.spin(move_turtle)
    move_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
