#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
import math

class MoveTurtle():

    def __init__(self):

        rospy.init_node('move_turtle_foward', anonymous = True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.current_pose = PoseStamped() 
        
        self.velocity_msg = Twist()
        self.rate = rospy.Rate(5)

        self.move_foward(0.2, 0.0)


    def odom_callback(self, odom):              # funcao q atualiza valores da odometria
        self.current_pose.header = odom.header
        self.current_pose.pose = odom.pose.pose

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
        # while not rospy.is_shutdown():
            #odom = Odometry()
            
        position = odom.pose.pose.position
        #print(position)
        x_pose = position.x
        y_pose = position.y
            #orientation = odom.pose.pose.orientation

            #(_, __, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        print(x_pose, y_pose)
        target_position = (2.0, 5.0)

        distance_x = target_position[0] - x_pose
        distance_y = target_position[1] - y_pose

        print("DISTANCE:", distance_x)

        # self.move_foward(0.2 , 0.0)

            #print(distance_x, distance_y)
            #ifference = tuple(a - b for a, b in zip((x_pose, y_pose), self.target_position))
                        
            #theta = math.atan2(distance_y, distance_x)

        if distance_x <= 0.0:
            # self.move_foward(0.0 , 0.0)
            print("CHEGUEEEEEEEEEEEI")
            #while not rospy.is_shutdown():
            self.velocity_msg.linear.x  = 0.0
            self.velocity_msg.angular.z = 0.0
        # else:
        #     self.move_foward(0.2 , 0.0)

        #self.rate.sleep()

       
if __name__ == "__main__":
    while not rospy.is_shutdown():
        MoveTurtle()

    rospy.spin()

    

    

    
