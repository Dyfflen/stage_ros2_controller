#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node, Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

class RobotNavigationNode(Node):
    def __init__(self):
        super().__init__('robot_navigation_node')
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for main control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.current_x = -7.0
        self.current_y = -7.0
        self.current_theta = math.radians(45)
        self.initial_x = -7.0
        self.initial_y = -7.0
        self.initial_theta = math.radians(45)
        
        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0
        
        self.oddometry_is = True # change here if odometry is turned off
        if self.oddometry_is is True: # if oddometry is on
            self.estimated_error_target_1a = 0.69
            self.estimated_error_target_1b = 0.17
            self.estimated_error_target_2a = 0.84
            self.estimated_error_target_2b = 0.66
        else:
            self.estimated_error_target_1a = 0.0
            self.estimated_error_target_1b = 0.0
            self.estimated_error_target_2a = 0.0
            self.estimated_error_target_2b = 0.0
        self.set_target_1a = 7.0
        self.set_target_1b = -3.0
        self.set_target_2a = 7.0
        self.set_target_2b = 7.0

        self.goals = [(self.set_target_1a + self.estimated_error_target_1a, self.set_target_1b + self.estimated_error_target_1b), (self.set_target_2a + self.estimated_error_target_2a, self.set_target_2b + self.estimated_error_target_2b)]
        self.current_goal_index = 0.0
        self.goal_tolerance = 0.02
        self.robot_speed = 0.3
        self.turn_speed = 0.4
        
        self.state = 'start'
        self.prev_state = ''
        self.goal_reached_time = None
        self.safe_distance = 1.2
        self.critical_distance = 0.8
        self.wall_follow_distance = 1.0
        self.wall_follow_start_time = None
        self.emergency_distance = 0.1
        self.front_sector_angles = [-30, -15, 0, 15, 30]
        self.hit_points = []
        self.leave_points = []
        self.target_theta = 0.0
        self.stuck_counter = 0.0
        
        self.get_logger().info('Robot Navigation Node initialized')
        self.get_logger().info(f'Starting position: ({self.current_x}, {self.current_y})')
        self.get_logger().info(f'Goals: {(self.set_target_1a, self.set_target_1b), (self.set_target_2a, self.set_target_2b)}')

    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        rotated_odom_x = odom_x * math.cos(self.initial_theta) - odom_y * math.sin(self.initial_theta)
        rotated_odom_y = odom_x * math.sin(self.initial_theta) + odom_y * math.cos(self.initial_theta)
        self.current_x = self.initial_x + rotated_odom_x
        self.current_y = self.initial_y + rotated_odom_y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_theta = self.normalize_angle(yaw + self.initial_theta)

    def laser_callback(self, msg):
        self.laser_ranges = list(msg.ranges)
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment

    def get_laser_reading(self, angle_deg):
        if not self.laser_ranges:
            return float('inf')
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - self.laser_angle_min) / self.laser_angle_increment)
        index = max(0, min(index, len(self.laser_ranges) - 1))
        reading = self.laser_ranges[index]
        return reading if not math.isinf(reading) and not math.isnan(reading) else 10.0

    def get_front_readings(self):
        front_left = self.get_laser_reading(10)
        front_right = self.get_laser_reading(-10)
        front_center = self.get_laser_reading(0)
        return front_left, front_right, front_center

    def is_path_clear(self, target_angle, check_distance=2.0):
        target_angle_deg = math.degrees(target_angle)
        check_angles = [target_angle_deg - 10, target_angle_deg, target_angle_deg + 10]
        for angle in check_angles:
            distance = self.get_laser_reading(angle)
            if distance < check_distance:
                return False
        return True

    def get_side_readings(self):
        left_side = self.get_laser_reading(90)
        right_side = self.get_laser_reading(-90)
        return left_side, right_side

    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def move_towards_goal(self):
        if self.current_goal_index >= len(self.goals):
            return
            
        goal_x, goal_y = self.goals[self.current_goal_index]
        distance_to_goal = self.calculate_distance(self.current_x, self.current_y, goal_x, goal_y)
        angle_to_goal = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
        
        path_blocked = not self.is_path_clear(angle_to_goal, self.safe_distance)
        front_blocked = self.get_laser_reading(0) < self.critical_distance
        
        if distance_to_goal > self.safe_distance and (path_blocked or front_blocked):
            best_direction = self.choose_best_escape_direction()
            self.get_logger().info(f'Path blocked to goal, switching to wall following (preferring {best_direction} side).')
            self.hit_points.append([self.current_x, self.current_y])
            self.state = 'wall_following'
            self.wall_follow_start_time = self.get_clock().now()
            return
        
        cmd = Twist()
        
        front_distance = self.get_laser_reading(0)
        if front_distance > self.critical_distance:
            if distance_to_goal < 1.0:
                cmd.linear.x = self.robot_speed * 0.5 # for safety
            else:
                cmd.linear.x = self.robot_speed
        else:
            cmd.linear.x = 0.0
        
        # Angular control, aligning with the target
        angle_diff = self.normalize_angle(angle_to_goal - self.current_theta)
        cmd.angular.z = max(-1.0, min(1.0, 2.0 * angle_diff))
        self.cmd_vel_pub.publish(cmd)

    def is_dead_end(self):
        front_readings = [self.get_laser_reading(angle) for angle in [-45, -30, -15, 0, 15, 30, 45]]
        left_readings = [self.get_laser_reading(angle) for angle in [60, 75, 90, 105, 120]]
        right_readings = [self.get_laser_reading(angle) for angle in [-60, -75, -90, -105, -120]]
        
        # If the front and one side are blocked, it is a dead end
        front_blocked = all(reading < self.safe_distance for reading in front_readings)
        right_blocked = all(reading < self.safe_distance for reading in right_readings)
        left_blocked = all(reading < self.safe_distance for reading in left_readings)
        return front_blocked and (right_blocked or left_blocked)

    def choose_best_escape_direction(self):
        goal_x, goal_y = self.goals[self.current_goal_index]
        angle_to_goal = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
        angle_to_goal_deg = math.degrees(self.normalize_angle(angle_to_goal - self.current_theta))
        
        # Looking for space
        left_space = sum([self.get_laser_reading(angle) for angle in [45, 60, 90, 120, 135]]) / 5
        right_space = sum([self.get_laser_reading(angle) for angle in [-45, -60, -90, -120, -135]]) / 5
        self.get_logger().info(f'Goal angle: {angle_to_goal_deg:.1f}°, Left space: {left_space:.2f}m, Right space: {right_space:.2f}m')
        
        # Target and free space are together on the left
        if angle_to_goal_deg > 30 and left_space > right_space + 0.5:
            return 'left'
        # Target and free space are together on the right
        elif angle_to_goal_deg < -30 and right_space > left_space + 0.5:
            return 'right'
        # Look for space
        else:
            return 'left' if left_space > right_space else 'right' # probably caused a bug that made me add a stuck counter...

    def wall_follow(self):
        front_left, front_right, front_center = self.get_front_readings()
        left_side, right_side = self.get_side_readings()
        cmd = Twist()
        
        # Security check
        min_distance = min([front_left, front_right, front_center, left_side, right_side])
        if min_distance < self.emergency_distance:
            self.emergency_stop()
            self.get_logger().error(f'Obstacle at {min_distance:.2f}m. Stopping!')
            return
        
        if self.is_dead_end():
            self.get_logger().warn('Dead end detected! Backing up and turning around...')
            cmd.linear.x = -self.robot_speed * 0.5
            cmd.angular.z = self.turn_speed * 1.5
            self.stuck_counter += 2
            self.cmd_vel_pub.publish(cmd)
            return
        
        if front_center < self.critical_distance or front_left < self.critical_distance or front_right < self.critical_distance:
            best_direction = self.choose_best_escape_direction()
            self.get_logger().info(f'Obstacle ahead, turning {best_direction}...')
            cmd.linear.x = 0.0
            if best_direction == 'left':
                cmd.angular.z = self.turn_speed
            else:
                cmd.angular.z = -self.turn_speed
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
            goal_x, goal_y = self.goals[self.current_goal_index]
            angle_to_goal = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
            angle_to_goal_deg = math.degrees(self.normalize_angle(angle_to_goal - self.current_theta))
            
            # Choose which wall to follow depending on the target direction
            if right_side < 5.0 and (angle_to_goal_deg < 0 or right_side < left_side):
                # Follow the wall on the right
                if right_side > self.wall_follow_distance + 0.2:
                    cmd.linear.x = self.robot_speed * 0.5
                    cmd.angular.z = -0.3
                elif right_side < self.wall_follow_distance - 0.2:
                    cmd.linear.x = self.robot_speed * 0.3
                    cmd.angular.z = 0.4
                else:
                    cmd.linear.x = self.robot_speed * 0.6
                    cmd.angular.z = 0.0
            elif left_side < 5.0 and (angle_to_goal_deg > 0 or left_side < right_side):
                # Follow the wall on the left
                if left_side > self.wall_follow_distance + 0.2:
                    cmd.linear.x = self.robot_speed * 0.5
                    cmd.angular.z = 0.3
                elif left_side < self.wall_follow_distance - 0.2:
                    cmd.linear.x = self.robot_speed * 0.3
                    cmd.angular.z = -0.4
                else:
                    cmd.linear.x = self.robot_speed * 0.6
                    cmd.angular.z = 0.0
            else:
                # If there are no walls nearby, go to the objective
                angle_diff = self.normalize_angle(angle_to_goal - self.current_theta)
                cmd.linear.x = self.robot_speed * 0.5
                cmd.angular.z = max(-0.5, min(0.5, angle_diff))
        
        if self.stuck_counter > 15:
            self.get_logger().warn('Robot seems stuck, trying to escape...')
            cmd.linear.x = -self.robot_speed * 0.4
            cmd.angular.z = self.turn_speed * 1.2
            self.stuck_counter = 0
        self.cmd_vel_pub.publish(cmd)
        
        goal_x, goal_y = self.goals[self.current_goal_index]
        current_dist = self.calculate_distance(self.current_x, self.current_y, goal_x, goal_y)
        
        if len(self.hit_points) > 0:
            hit_x, hit_y = self.hit_points[-1]
            hit_dist = self.calculate_distance(hit_x, hit_y, goal_x, goal_y)
            angle_to_goal = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
            path_clear = self.is_path_clear(angle_to_goal, self.safe_distance)
            progress_made = current_dist < hit_dist - 0.3
            timeout = (self.get_clock().now() - self.wall_follow_start_time).nanoseconds / 1e9 > 15.0  # wall follow timeout, necessary to avoid generating infinite navigation loops
            
            if (progress_made and path_clear) or timeout:
                self.state = 'move_to_goal'
                self.wall_follow_start_time = None
                self.stuck_counter = 0
                if timeout:
                    self.get_logger().warn('Wall following timeout, returning to direct navigation')
                else:
                    self.get_logger().info('Progress made, returning to direct navigation')

    def control_loop(self):
        if not self.laser_ranges: 
            return
        
        if self.check_collision_risk() and self.state != 'goal_reached':
            self.emergency_stop()
            return
            
        current_goal = self.goals[self.current_goal_index] if self.current_goal_index < len(self.goals) else None

        if current_goal is None:
            if self.state != 'end':
                self.get_logger().info('All targets reached!')
                self.state = 'end'
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        goal_x, goal_y = current_goal
        distance_to_goal = self.calculate_distance(self.current_x, self.current_y, goal_x, goal_y)
        
        if self.state == 'start':
            self.get_logger().info(f'Starting navigation to goal {self.current_goal_index + 1}')
            self.state = 'move_to_goal'
            
        elif self.state == 'move_to_goal':
            if distance_to_goal < self.goal_tolerance:
                self.get_logger().info(f'Goal {self.current_goal_index + 1} reached! Pausing for 1 second.')
                self.state = 'goal_reached'
                self.goal_reached_time = self.get_clock().now()
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                return
            
            self.move_towards_goal()
        
        elif self.state == 'goal_reached':
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)

            if self.get_clock().now() - self.goal_reached_time > rclpy.duration.Duration(seconds=1):
                self.current_goal_index += 1
                self.goal_reached_time = None
                
                if self.current_goal_index >= len(self.goals):
                    self.state = 'end'
                else:
                    self.state = 'start'
            
        elif self.state == 'wall_following':
            self.wall_follow()
                
        elif self.state == 'end':
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)

    def check_collision_risk(self):
        if not self.laser_ranges: 
            return True
        
        min_front_distance = float('inf')
        for angle in self.front_sector_angles:
            min_front_distance = min(min_front_distance, self.get_laser_reading(angle))
        
        if min_front_distance < self.emergency_distance:
            return True
        return False

    def emergency_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().warn('Obstacle too close!')

def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()