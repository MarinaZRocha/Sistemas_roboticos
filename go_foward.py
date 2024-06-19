#! /usr/bin/env python3

# Trabalho 4 da disciplina de Sistemas Robóticos
#  A tarefa proposta é fazer com que um robô diferencial ande para frente 
#  Para isso fez-se esse nodo publisher para enviar dados de velocidade
#  Marina Zanotta Rocha - 140592

import rospy
from geometry_msgs.msg import *

class MoveTurtle():

    def __init__(self):

        rospy.init_node('move_turtle_foward', anonymous = True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10) #publisher para enviar dados para o tópico cmd_vel

        self.velocity_msg = Twist()
        self.rate = rospy.Rate(5)
        self.move_foward(0.2, 0.0)      #define apenas velocidade linear

    def move_foward(self, vel_x, vel_z):    #funcao para fazer o robo andar
        
        self.velocity_msg.linear.x  = vel_x
        self.velocity_msg.angular.z = vel_z
        
        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.velocity_msg)
            self.rate.sleep()

if __name__ == "__main__":
   while not rospy.is_shutdown():
        MoveTurtle()
   rospy.spin()