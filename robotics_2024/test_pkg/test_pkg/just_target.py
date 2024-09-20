#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf_transformations
import math

class moveturtle():

    def __init__(self):
        rclpy.init_node('move_turtle_foward', anonymous=True)
        self.velocity_publisher = self.create_publisher('/cmd_vel', Twist, queue_size = 10) #publisher para enviar dados para o tópico cmd_vel
        self.odom_subscriber = rclpy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)    # define subscriber, recebe a os valores da posição atual do robô
        self.current_pose = PoseStamped()  
        goal = (2.0, 5.0)

        self.navigate_to_goal(goal)
        self.rate = rclpy.Rate(5)

    def odom_callback(self, odom):              # funcao q atualiza valores da odometria
        self.current_pose.header = odom.header
        self.current_pose.pose = odom.pose.pose

    def quaternion_to_yaw(self, quaternion):    # funcao que converte de euler para quaternio para obter a rotacao "yaw"
        _, _, yaw = tf_transformations.quaternion_from_euler([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
        return yaw        
    
    def velocity_control(self, vel_x, vel_z):    #funcao para fazer o robo andar
        velocity_msg = Twist()
        velocity_msg.linear.x  = vel_x
        velocity_msg.angular.z = vel_z
        self.velocity_publisher.publish(velocity_msg)
    def navigate_to_goal(self, goal):                       # principal funcao, em que sao calculadas as distancias atual e desejada e a rotacao 
       while not rclpy.is_shutdown():                      # a partir disso compara-se se a distancia ate o alvo esta dentro da tolerancia (0.1)
            x_target, y_target = goal
            x_pose = self.current_pose.pose.position.x
            y_pose = self.current_pose.pose.position.y

            x_distance = x_target - x_pose
            y_distance = y_target - y_pose

            theta = math.atan2(y_distance, x_distance)
            point_distance = math.sqrt(x_distance ** 2 + y_distance ** 2)
            angle_distance = theta - self.quaternion_to_yaw(self.current_pose.pose.orientation)

            if point_distance <= 0.1:                       # se a distancia for aceitavel, printa uma mensagem dizendo q atingiu o alvo
                print("Chegueei!")
                self.velocity_control(0.0, 0.0)        # faz o robo parar ate ir para o próximo alvo 
                rclpy.signal_shutdown("Bye bye")          # envia um sinal para parar o script
                break
            else:
                self.velocity_control(0.1 * point_distance, 0.2 * angle_distance)  # enquanto a distancia ainda for grande, a velocidade eh dada pela multiplicacao de uma constante
 
if __name__ == "__main__":
  # while not rospy.is_shutdown():
    moveturtle()
    rclpy.spin()