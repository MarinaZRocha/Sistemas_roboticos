#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from time import sleep
from tf_transformations import euler_from_quaternion
import copy

class MoveRobot(Node):
    def __init__(self):
        super().__init__('go_targets')
        
        # Estados do robô
        self.state = 0  # 0: Go to point, 1: Avoid obstacle
        self.state_wall = 0
        self.state_go_to_point = 0
        self.error_angle = 0.0
        self.obstacle_threshold = 0.3  # Distância mínima para evitar obstáculos
        self.yaw_precision = math.pi / 180  # +/- 2 degrees allowed
        self.dist_precision = 0.3  # Tolerância para atingir o objetivo
        self.initial_position = Point()  # Inicialização com objeto Point
        self.position = Point()  # Inicializa posição para evitar erro de atributo
        self.aux_initial = True
        self.avoid_time = 0.0
        self.done = False
        self.current_goal_index = 0  # Índice do objetivo atual
        self.goals = [(5.0, 4.0), (4.0, -4.0)]  # Lista de objetivos
        self.avoid_time = None  # Tempo de início do estado avoid_obstacle (None enquanto não iniciado)
        self.max_avoid_time = 20.0

        # Publicadores e assinantes
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)
        self.create_subscription(Odometry, '/odom', self.position_callback, 10)
        
        self.set_goal(self.goals[self.current_goal_index])  # Inicializa com o primeiro objetivo
        rclpy.get_default_context().on_shutdown(self.shutdown)

    def set_goal(self, goal):
        # Calcula a posição do objetivo em coordenadas corrigidas
        self.goal_position = Point()
        origin_map_x, origin_map_y = -7, -7
        target_x, target_y = goal
        
        target_x_correct = target_x - origin_map_x
        target_y_correct = target_y - origin_map_y
        
        angle_origin_correct = -45 * math.pi / 180
        self.get_logger().info(f"New goal set: {target_x_correct}, {target_y_correct}")
        
        # Calcula a posição corrigida do objetivo
        self.goal_position.x = target_x_correct * math.cos(angle_origin_correct) - target_y_correct * math.sin(angle_origin_correct)
        self.goal_position.y = target_x_correct * math.sin(angle_origin_correct) + target_y_correct * math.cos(angle_origin_correct)
        
        self.get_logger().info(f"New goal set: {self.goal_position.x}, {self.goal_position.y}")

    def go_to_point(self):
        msg = Twist()
        # Checa se o robô atingiu o objetivo
        if self.state_go_to_point == 0:
            if self.distance_goal() <= self.dist_precision:
                self.get_logger().info("Cheguei ao objetivo!")
                self.done = True
                self.state_go_to_point = 2  # Estado de finalização
            else:
                self.fix_yaw()
        elif self.state_go_to_point == 1:
            if self.distance_goal() <= self.dist_precision:
                self.get_logger().info("Cheguei ao objetivo!")
                self.done = True
                self.state_go_to_point = 2
            else: 
                self.go_forward()
        elif self.state_go_to_point == 2:
            # Parar o robô e verificar o próximo objetivo
            if self.done:
                self.get_logger().info("Reached goal, stopping robot.")
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.velocity_publisher.publish(msg)
            
            # Checa se há mais objetivos na lista
                if self.current_goal_index < len(self.goals) - 1:
                    self.done = False
                    self.get_logger().info("Moving to next goal.")
                    self.current_goal_index += 1
                    self.set_goal(self.goals[self.current_goal_index])
                    self.state_go_to_point = 0
                    
                else:
                    self.get_logger().info("All goals reached. Stopping robot.")
                    self.done = True
                    self.shutdown()

    def fix_yaw(self):
        msg = Twist()
        # Alinha o robô com o objetivo
        if math.fabs(self.error_angle) > self.yaw_precision:
            msg.angular.z = 0.1 if self.error_angle > 0 else -0.1
            self.velocity_publisher.publish(msg)
        else:
            self.state_go_to_point = 1

    def go_forward(self):
        msg = Twist()
        # Move o robô para frente em direção ao objetivo
        if self.distance_goal() > self.dist_precision:
            msg.linear.x = 0.2
            self.velocity_publisher.publish(msg) 
        else:
            self.done = True
            self.state_go_to_point = 2

        # Revisa se há necessidade de corrigir o ângulo
        if math.fabs(self.error_angle) > self.yaw_precision:
            self.state_go_to_point = 0
    
    def distance_goal(self):
        # Calcula a distância até o objetivo
        return round(math.hypot(self.goal_position.x - self.position.x, self.goal_position.y - self.position.y), 2)

    def shutdown(self):
        # Para o robô e finaliza o nó
        self.get_logger().info('Shutting down!')
        linear_velocity = Twist()
        linear_velocity.linear.x = 0
        self.velocity_publisher.publish(linear_velocity)

    def laser_callback(self, data):
        # Callback para lidar com os dados do LaserScan
        self.laser_reading = data
        # Verifica se os dados do laser são válidos antes de processar
        if data.ranges:
            self.regions = {
                'right': min(min(data.ranges[30:90]), 5),
                'front': min(min(data.ranges[100:160]), 5),
                'left': min(min(data.ranges[170:230]), 5)
            }
            self.action_wall()

    def action_wall(self):
        # Atualiza o estado com base nos dados do laser
        d = 0.6
        if self.regions['front'] > d:
            self.state_wall = 0
        elif self.regions['front'] < d:
            self.state_wall = 1

    def turn_left(self):
        # Gira o robô para a esquerda
        msg = Twist()
        msg.angular.z = 0.2
        self.velocity_publisher.publish(msg)
            
    def avoid_obstacle(self):
        # Função para desviar de obstáculos
        msg = Twist()
        if self.state_wall == 0:
            if self.distance_goal() <= self.dist_precision:
                self.get_logger().info("got stuck.")
                self.done = True
                self.go_to_point()
                self.state_go_to_point = 2
            else:
                msg.linear.x = 0.2
                msg.angular.z = -0.1
                self.get_logger().info("Obstacle detected, stopping.")
                self.velocity_publisher.publish(msg)
        elif self.state_wall == 1:
            self.turn_left()

    def normalize(self, angle):
        # Normaliza o ângulo para o intervalo [-pi, pi]
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def distance_to_line(self):
        # Calcula a distância perpendicular até a linha do objetivo
        if not self.position:
            self.get_logger().info("Position not yet initialized.")
            return float('inf')  # Retorna infinito se a posição não estiver inicializada

        up_eq = math.fabs((self.goal_position.y - self.initial_position.y) * self.position.x - 
                          (self.goal_position.x - self.initial_position.x) * self.position.y + 
                          (self.goal_position.x * self.initial_position.y) - 
                          (self.goal_position.y * self.initial_position.x))
        lo_eq = math.sqrt(pow(self.goal_position.y - self.initial_position.y, 2) + 
                          pow(self.goal_position.x - self.initial_position.x, 2))
        distance = up_eq / lo_eq
        return distance

    def position_callback(self, data):
        # Callback para lidar com os dados de odometria e atualizar a posição e orientação
        print('position_callback', data.pose.pose.position)
        self.position = data.pose.pose.position

        # Inicializa a posição do robô na primeira execução
        if self.aux_initial:
            self.initial_position = copy.deepcopy(self.position)
            self.aux_initial = False

        # Converte a orientação de quaternion para ângulos de Euler
        orientation = data.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

        # Calcula o ângulo do objetivo e o erro de orientação
        self.goal_angle = math.atan2(self.goal_position.y - self.position.y, self.goal_position.x - self.position.x)
        self.heading = self.goal_angle - self.yaw
        self.error_angle = self.normalize(self.heading)

def main(args=None):
    rclpy.init(args=args)

    env = MoveRobot()
    # Obter o tempo atual
    current_time = env.get_clock().now()

    sleep(1)

    while rclpy.ok():
        rclpy.spin_once(env)
        distance_position_to_line = env.distance_to_line()
        env.get_logger().info(f'Distance to Goal: {env.distance_goal()}')
        # Máquina de estados principal
        if env.state == 0:
            env.go_to_point()
            if 0.01 < env.regions['front'] < 0.45:
                env.state = 1

        elif env.state == 1:
            env.avoid_time = current_time
            env.avoid_obstacle()
            
            if ((current_time - env.avoid_time).nanoseconds / 1e9) > 20.0 and distance_position_to_line < 0.25:
                env.state = 0
                current_time = 0.0
                env.go_to_point()


if __name__ == '__main__':
    main()
