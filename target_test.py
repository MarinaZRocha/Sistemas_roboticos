#!/usr/bin/env python3

# Trabalho final da disciplina de Sistemas Robóticos
# O robô diferencial utilizado foi o Turtlebot3 burger simulado no Gazebo Classic em ROS 1
# O seguinte código faz com que o robô percorra os pontos pré-definidos
# Andressa Cavalcante da silva - 140594 & Marina Zanotta Rocha - 140592


# importa bibliotecas necessarias

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class TurtleBotNavigator:
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)     # define publisher, nesse caso optou-se por enviar valores de velocidade 
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)    # define subscriber, recebe a os valores da posição atual do robô
        self.current_pose = PoseStamped()                                  # utilizado para atualizar a pose do robô

        # define alvos

        self.goals = [
            (1.0, 0.0),
            (1.5, 1.0),
            (2.0, 2.0),
            (3.0, 5.0)]

        self.current_goal_index = 0
        self.next_goal()

    def odom_callback(self, odom):              # funcao q atualiza valores da odometria
        self.current_pose.header = odom.header
        self.current_pose.pose = odom.pose.pose

    def quaternion_to_yaw(self, quaternion):    # funcao que converte de euler para quaternio para obter a rotacao "yaw"
        _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw

    def send_velocity_command(self, linear_x, angular_z):   # funao que define o controle de velocidade em x e em z
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        self.velocity_publisher.publish(vel_msg)

    def next_goal(self):                                    # funcao para atualizar os alvos
        current_goal = self.goals[self.current_goal_index]
        self.navigate_to_goal(current_goal)

    def navigate_to_goal(self, goal):                       # principal funcao, em que sao calculadas as distancias atual e desejada e a rotacao 
        while not rospy.is_shutdown():                      # a partir disso compara-se se a distancia ate o alvo esta dentro da tolerancia (0.1)
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
                self.send_velocity_command(0.0, 0.0)        # faz o robo parar ate ir para o próximo alvo 
                self.current_goal_index += 1                # incrementa contagem dos pontos percorridos
                if self.current_goal_index < len(self.goals):   # enquanto ainda houver alvos para atingir
                    self.next_goal()
                else:
                    rospy.loginfo("Cheguei em todos os alvos! Tchaau")  # depois que o robo passou por todos os pontos, printa uma mensagem dizendo que cumpriu a meta
                    rospy.signal_shutdown("All goals reached")          # envia um sinal para parar o script
                break
            else:
                self.send_velocity_command(0.2 * point_distance, 0.4 * angle_distance)  # enquanto a distancia ainda for grande, a velocidade eh dada pela multiplicacao de uma constante
                                                                                        # pela distancia remanescente e outra constante pela diferenca do angulo
if __name__ == "__main__":
    TurtleBotNavigator()
    rospy.spin()