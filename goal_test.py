#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math

class MoveTurtle():

    def __init__(self):
        rospy.init_node('move_turtle_forward', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.move2goal, queue_size=10)
        self.velocity_msg = Twist()
        self.rate = rospy.Rate(5)

        self.move_foward(0.2, 0.0)

    def adiciona_perturbacao_vel(self, vel):
        """Add disturbance to velocity"""
        vel_com_perturbacao = Twist()
        vel_com_perturbacao.linear.x = vel.linear.x + np.random.normal(loc=0.0, scale=0.1)
        vel_com_perturbacao.angular.z = vel.angular.z + np.random.normal(loc=0.0, scale=0.1)
        return vel_com_perturbacao

    def move_foward(self, vel_x, vel_z):
        """Move the robot"""
        self.velocity_msg.linear.x = vel_x
        self.velocity_msg.angular.z = vel_z

        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.velocity_msg)
            self.rate.sleep()

    def move2goal(self, odom):
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        x_pose = position.x
        y_pose = position.y
        i = 0

        (_, __, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        targets_positions = [(2.0, 1.5),(0.0, 6.5),(-4.0, 0.0),(-0.3, -0.7),(0.1, 0.1)]

        x_distance = targets_positions[i][0] - x_pose
        y_distance = targets_positions[i][1] - y_pose
        theta = math.atan2(y_distance, x_distance)
        point_distance = math.sqrt(x_distance ** 2 + y_distance ** 2)
        angle_distance = theta - yaw
        print("DISTANCE:", point_distance)

        if point_distance <= 0.8:
            print("REACHED THE TARGET")
            self.velocity_msg.linear.x = 0.0
            self.velocity_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.velocity_msg)
            i += 1
            if i == 4:
                rospy.signal_shutdown("Reached the target position")
        else:
            print("NOT YET AT TARGET")
            print("Current Position: x =", x_pose, "y =", y_pose)
            print("Current Orientation: yaw =", yaw)
            self.velocity_msg.linear.x = 0.02*point_distance
            self.velocity_msg.linear.y = 0
            self.velocity_msg.linear.z = 0
            self.velocity_msg.angular.x = 0
            self.velocity_msg.angular.y = 0
            self.velocity_msg.angular.z = 0.02*angle_distance
            

            self.velocity_publisher.publish(self.velocity_msg)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        MoveTurtle()

    rospy.spin()
