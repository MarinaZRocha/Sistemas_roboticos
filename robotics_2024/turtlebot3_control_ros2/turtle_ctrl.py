#!/usr/bin/env python3

# Trabalho de roboticos Marina Zanotta Rocha 
# Robo deve percorrer maior área possível sem colidir
# Primeiro teste: threshold muito alto
# Segundo teste: threshold corrigido, ainda nao desvia bem de obstaculos
# Terceiro teste: robo não ficou girando infinitamente, mas nao seguia em frente
# Estado atual: robo consegue percorrer o ambiente aleatoriamente e desviar de obstaculos

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import random

from example_interfaces.msg import String

class TurtlebotCtrl(Node):
    def __init__(self):
        super().__init__("TurtlebotCtrl")

        self.laser = LaserScan()
        self.odom = Odometry()

        self.map = np.array([   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
        self.obstacle_distance_threshold = 0.4  # Threshold distance to consider an obstacle
        self.state = "MOVING_FORWARD"
        self.turn_direction = 0.5
        self.turn_start_time = 0.0
        self.move_back_start_time = 0.0
        self.initial_turn_direction = 0.5
        self.turn_total = 0.0

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

        msg = Twist()
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if self.state == "MOVING_FORWARD":             # default state
            if self.detect_obstacle():
                self.state = "MOVING_BACKWARD"         # goes back before turn
                self.get_logger().info("going backwards")
                self.move_back_start_time = current_time    # start time for going back
                msg.linear.x = -0.1
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.1
                msg.angular.z = 0.0

        elif self.state == "MOVING_BACKWARD":          # backwards state
            if current_time - self.move_back_start_time < 0.5:
                msg.linear.x = -0.1                    # goes backwards for 0.5s
                msg.angular.z = 0.0
            else:
                self.state = "TURNING"                 # after going backwards, turn
                self.turn_start_time = current_time    # start time for turning side 1
                self.turn_total = current_time         # start total time turning
                self.turn_direction = self.initial_turn_direction
                msg.linear.x = 0.0

        elif self.state == "TURNING":                  # turning state
            
            if current_time - self.turn_start_time < 2.0:   # turns for 2s
                self.get_logger().info("turning")
                msg.angular.z = self.turn_direction
            else:
                if self.detect_obstacle():                  # if it still detects obstacle
                    if current_time - self.turn_total < 8.0:
                        self.get_logger().info("changing direction")
                        self.turn_direction *= -1.2          # Change turn direction
                        self.turn_start_time = current_time  # Reset turn start time for side 2
                    else:                                    # if total turn time > 8s
                        msg.angular.z = self.turn_direction  # keeps turning only one direction
                else:
                    self.state = "MOVING_FORWARD"            # not detecting obstacle
                    self.get_logger().info("no obstacle, going forward")
                    msg.linear.x = 0.1
                    msg.angular.z = 0.0

        self.publish_cmd_vel.publish(msg)

    def callback_laser(self, scan):
        self.laser = scan
        #self.get_logger().info(f"Laser Scan Data: {self.laser.ranges[:10]}")  # Print first 10 scan ranges for verification

    def callback_odom(self, odom):
        self.odom = odom

    def detect_obstacle(self):
        if self.laser is None:
            return False
        # Only considers 30° in front of the robot
        front_angles = list(self.laser.ranges[0:15]) + list(self.laser.ranges[-15:])
        min_distance = min(front_angles)
        #self.get_logger().info(f"Min distance in front: {min_distance}")

        return min_distance < self.obstacle_distance_threshold

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotCtrl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
