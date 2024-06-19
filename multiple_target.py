#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry, Pose
import numpy as np
from tf.transformations import euler_from_quaternion
import math

class MoveTurtle():

    def __init__(self):

        rospy.init_node('move_turtle_foward', anonymous = True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.move2goal, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/pose', Pose, self.update_pose)
        self.velocity_msg = Twist()
        self.rate = rospy.Rate(5)

        #self.move_foward(0.2, 0.0)
        self.velocity_msg.linear.x = 0.2
        self.velocity_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity_msg)
        

    def adiciona_perturbacao_vel(self, vel):
        """ Function that adds disturbance
            loc = media, 
            scale = desvio padrao """

        vel_com_perturbacao = Twist()
        vel_com_perturbacao.linear.x  = vel.linear.x  + np.random.normal(loc = 0.0, scale = 0.1)     
        vel_com_perturbacao.angular.z = vel.angular.z + np.random.normal(loc = 0.0, scale = 0.1) 
        
        return vel_com_perturbacao
    
    def move_foward(self, vel_x, vel_z):
        """ Move the robot """
        
        self.velocity_msg.linear.x  = vel_x
        self.velocity_msg.angular.z = vel_z

        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.velocity_msg)
            self.rate.sleep()
            print("Preso aquii")
            #self.velocity_publisher.publish(self.adiciona_perturbacao_vel(self.velocity_msg))
            
    def move2goal(self, odom):
        """ Move the robot to the target position """

        targets_positions = [(4.0, 2.5),(0.0, 6.5),(-4.0, 0.0),(-0.3, -0.7),(0.1, 0.1)]
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        count = 0

        x_pose = position.x
        y_pose = position.y
        x_target = targets_positions[count][0]
        y_target = targets_positions[count][1]

        x_distance = x_target - x_pose
        y_distance = y_target - y_pose

        (_, __, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        theta = math.atan2(y_distance, x_distance)
        point_distance = math.sqrt(x_distance ** 2 + y_distance ** 2)

        angle_distance = theta - yaw


        # print("-------------------Target: ", count, "X: ", x_target, "Y: ", y_target)

        if point_distance < 0.8:
            count += 1
            print("-------------------Target: ", count, "X: ", x_target, "Y: ", y_target)
            if count > 4:
                while not rospy.is_shutdown():
                    self.velocity_msg.linear.x = 0.0
                    self.velocity_msg.angular.z = 0.0
                    self.velocity_publisher.publish(self.velocity_msg)
                    print("AQUIIII 2")
        

        else:
            self.velocity_msg.angular.z = angle_distance
            self.velocity_msg.linear.x = 0.0
            self.velocity_publisher.publish(self.velocity_msg)
            # if yaw == (theta):
                # self.velocity_msg.linear.x = 0.2
                # self.velocity_msg.angular.z = 0.0
                # self.velocity_publisher.publish(self.velocity_msg)

                        


            #self.move_foward(0.2, 0.0)
            # self.rate.sleep()

         
            
       
if __name__ == "__main__":
    while not rospy.is_shutdown():
        MoveTurtle()

    rospy.spin()