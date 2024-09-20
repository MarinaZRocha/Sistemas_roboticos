#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from time import sleep
import tf_transformations as transform
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import copy

class MoveRobot(Node):
    def __init__(self):
        super().__init__('go_targets')
        
        self.state = 0  # 0 go to point, 1 wall following
        self.state_wall = 0
        self.state_go_to_point = 0
        self.error_angle =0.0
        self.obstacle_threshold = 0.3
        self.yaw_precision = math.pi / 180  # +/- 2 degree allowed
        self.dist_precision = 0.4
        self.initial_position = Point()  # Initialize with a proper Point object
        self.position = Point()  # Initialize position to avoid the attribute error
        self.aux_initial = True
        self.done = False
        self.current_goal_index = 0  # Track the current goal index
        self.goals = [(5.0, 4.0), (4.0, -4.0)]  # List of goals (x, y) - second goal added

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)
        self.create_subscription(Odometry, '/odom', self.position_callback, 10)
        
        self.set_goal(self.goals[self.current_goal_index])  # Initialize first goal
        rclpy.get_default_context().on_shutdown(self.shutdown)

    def set_goal(self, goal):
        #data = Odometry()
        #self.goal_position = data.pose.pose.position
        self.goal_position = Point()
        origin_map_x, origin_map_y = -7, -7
        target_x, target_y = goal
        
        target_x_correct = target_x - origin_map_x
        target_y_correct = target_y - origin_map_y
        
        angle_origin_correct = -45 * math.pi / 180
        self.get_logger().info(f"New goal set: {target_x_correct}, {target_y_correct}")
        
        self.goal_position.x = target_x_correct * math.cos(angle_origin_correct) - target_y_correct * math.sin(angle_origin_correct)
        self.goal_position.y = target_x_correct * math.sin(angle_origin_correct) + target_y_correct * math.cos(angle_origin_correct)
        
        self.get_logger().info(f"New goal set: {self.goal_position.x}, {self.goal_position.y}")

    def go_to_point(self):
        msg = Twist()
        if self.state_go_to_point == 0:
            if self.distance_goal() <= self.dist_precision:
                self.get_logger().info("Chegueeei 1")
                self.done = True
                self.state_go_to_point = 2
            else:
                self.fix_yaw()
        elif self.state_go_to_point == 1:
            if self.distance_goal() <= self.dist_precision:
                self.get_logger().info("Chegueeei 2")
                self.done = True
                self.state_go_to_point = 2
            else: 
                self.go_foward()
        elif self.state_go_to_point == 2:
            if self.done:
                self.get_logger().info("Reached goal, stopping robot.")
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.velocity_publisher.publish(msg)
            
            # Check if there are more goals
            if self.current_goal_index < len(self.goals) - 1:
                self.get_logger().info("Moving to next goal.")
                self.current_goal_index += 1
                self.set_goal(self.goals[self.current_goal_index])
                self.state_go_to_point = 0
                self.done = False
            else:
                self.get_logger().info("All goals reached. Stopping robot.")
                # Stop robot and exit
                self.done = True
                self.shutdown()
         #   if self.distance_goal() > self.dist_precision:
          #      self.state_go_to_point = 0
           #     self.done = False

    def fix_yaw(self):
        msg = Twist()
        if math.fabs(self.error_angle) > self.yaw_precision:
            msg.angular.z = 0.1 if self.error_angle > 0 else -0.1
        self.velocity_publisher.publish(msg)
        if math.fabs(self.error_angle) <= self.yaw_precision:
            self.state_go_to_point = 1

    def go_foward(self):
        msg = Twist()
        if self.distance_goal() > self.dist_precision:
            msg.linear.x = 0.2
            self.velocity_publisher.publish(msg) 
        else:
            self.done = True
            self.state_go_to_point = 2
            

        if math.fabs(self.error_angle) > self.yaw_precision:
            self.state_go_to_point = 0

    def follow_wall(self):
        if self.state_wall == 0:
            self.find_wall()
        elif self.state_wall == 1:
            self.turn_left()

    def normalize(self, angle):
        if math.fabs(angle) > math.pi:
            angle -= 2 * math.pi
        elif math.fabs(angle) < -math.pi:
            angle += 2 * math.pi
        return angle

    def find_wall(self):
        msg = Twist()
        if self.distance_goal() <= self.dist_precision:
            self.get_logger().info("Chegueeei 1")
            self.done = True
            self.state_go_to_point = 2
            
        else:
            msg.linear.x = 0.2
            msg.angular.z = -0.1
            self.velocity_publisher.publish(msg)

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.2
        self.velocity_publisher.publish(msg)


    def distance_goal(self):
        return round(math.hypot(self.goal_position.x - self.position.x, self.goal_position.y - self.position.y), 2)

    def shutdown(self):
        self.get_logger().info('Shutting down!')
        linear_velocity = Twist()
        linear_velocity.linear.x = 0
        self.velocity_publisher.publish(linear_velocity)

    def laser_callback(self, data):
        self.laser_reading = data
        self.regions = {
            'right': min(min(data.ranges[30:90]), 5),
            'front': min(min(data.ranges[100:160]), 5),
            'left': min(min(data.ranges[170:230]), 5)
        }
        self.action_wall()

    def action_wall(self):
        d = 0.5
        if self.regions['front'] > d:
            self.state_wall = 0
        elif self.regions['front'] < d:
            self.state_wall = 1
        else:
            self.state_wall = 0

    def distance_to_line(self):
        if not self.position:
            self.get_logger().info("Position not yet initialized.")
            return float('inf')  # Return an infinite distance if the position is not set

        up_eq = math.fabs((self.goal_position.y - self.initial_position.y) * self.position.x - 
                          (self.goal_position.y - self.initial_position.x) * self.position.y + 
                          (self.goal_position.y * self.initial_position.y) - 
                          (self.goal_position.y * self.initial_position.x))
        lo_eq = math.sqrt(pow(self.goal_position.y - self.initial_position.y, 2) + 
                          pow(self.goal_position.x - self.initial_position.x, 2))
        distance = up_eq / lo_eq
        return distance

    def position_callback(self, data):
        #data = Odometry() 
        print('position_callback', data.pose.pose.position)
        self.position = data.pose.pose.position

        if self.aux_initial:
            self.initial_position = copy.deepcopy(self.position)
            self.aux_initial = False

        orientation = data.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

        self.goal_angle = math.atan2(self.goal_position.y - self.position.y, self.goal_position.x - self.position.x)
        self.heading = self.goal_angle - self.yaw
        self.error_angle = self.normalize(self.heading)
      #  self.get_logger().info(f'Error Angle: {self.error_angle}')

def main(args=None):
    rclpy.init(args=args)

    env = MoveRobot()

    count_time = 0
    count_loop = 0

    sleep(1)

    while rclpy.ok():
        rclpy.spin_once(env)
        distance_position_to_line = env.distance_to_line()

     #   env.get_logger().info(f'State Wall: {env.state_wall}, State Go To Point: {env.state_go_to_point}, State: {env.state}')
     #   env.get_logger().info(f'Regions: {env.regions}')
     #   env.get_logger().info(f'Distance to Line: {distance_position_to_line}')
        env.get_logger().info(f'Distance to Goal: {env.distance_goal()}')

        if env.state == 0:
            env.go_to_point()
            if 0.01 < env.regions['front'] < 0.45:
                env.state = 1

        elif env.state == 1:
            env.follow_wall()
            if count_time > 10 and distance_position_to_line < 0.15:
                env.state = 0
                count_time = 0

        count_loop += 1
        if count_loop == 20:
            count_time += 1
            count_loop = 0

if __name__ == '__main__':
    main()