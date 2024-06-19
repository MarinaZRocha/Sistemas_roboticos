#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
import math

class MoveTurtle():

    def __init__(self):

        rospy.init_node('move_turtle_foward', anonymous = True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.move2goal, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/pose', Pose, self.update_pose)
        #self.velocity_msg = Twist()
        

        #self.move_foward(0.2, 0.0)
        # self.velocity_msg.linear.x = 0.2
        # self.velocity_msg.angular.z = 0.0
        # self.velocity_publisher.publish(self.velocity_msg)
        self.pose = Pose()
        self.rate = rospy.Rate(5)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data.pose
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def adiciona_perturbacao_vel(self, vel):
        """ loc = media, scale = desvio padrao """

        vel_com_perturbacao = Twist()
        vel_com_perturbacao.linear.x  = vel.linear.x  + np.random.normal(loc = 0.0, scale = 0.1)     
        vel_com_perturbacao.angular.z = vel.angular.z + np.random.normal(loc = 0.0, scale = 0.1) 
        
        return vel_com_perturbacao
    
    def move_foward(self, vel_x, vel_z):
        """ Move the robot to the target position """

        self.velocity_msg.linear.x  = vel_x
        self.velocity_msg.angular.z = vel_z

        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.velocity_msg)
            self.rate.sleep()
            #self.velocity_publisher.publish(self.adiciona_perturbacao_vel(self.velocity_msg))
            
    def move2goal(self, odom):
        """ Move the robot to the target position """
        goal_pose = Pose()
        goal_pose.position.x = 4.0
        goal_pose.position.y = 2.5
        #targets_positions = [(4.0, 2.5),(0.0, 6.5),(-4.0, 0.0),(-0.3, -0.7),(0.1, 0.1)]
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        count = 0
        
        x_pose = position.x
        y_pose = position.y
        x_target = goal_pose.position.x
        y_target = goal_pose.position.y
        print("goal_pose: ", goal_pose)

        x_distance = x_target - x_pose
        y_distance = y_target - y_pose

        (_, __, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        theta = math.atan2(y_distance, x_distance)
        point_distance = math.sqrt(x_distance ** 2 + y_distance ** 2)

        angle_distance = theta - yaw
        vel_msg = Twist()
        while point_distance >= 0.8:
            # self.move_foward(0.0 , 0.0)
            print("x_pose: ", x_pose, "y_pose: ", y_pose)
        #    print("distance: ", point_distance)
            #while not rospy.is_shutdown():
            vel_msg.linear.x = 0.02*point_distance
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
                # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0.02*angle_distance
    
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.signal_shutdown("All goals reached")

       
if __name__ == "__main__":
    while not rospy.is_shutdown():
        MoveTurtle()

    rospy.spin()