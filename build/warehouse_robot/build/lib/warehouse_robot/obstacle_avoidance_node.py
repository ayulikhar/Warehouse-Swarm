#!/usr/bin/env python3
"""
Autonomous Obstacle Avoidance Node for Warehouse Robot
Uses LiDAR scan data to detect obstacles and navigate autonomously.
Behavior:
  - Move forward when path is clear
  - Turn away from obstacles when detected
  - Use a simple reactive controller (VFH-inspired)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('obstacle_distance_threshold', 0.8)
        self.declare_parameter('front_angle_range', 30.0)  # degrees each side
        self.declare_parameter('side_angle_range', 60.0)   # degrees each side

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.front_angle = math.radians(self.get_parameter('front_angle_range').value)
        self.side_angle = math.radians(self.get_parameter('side_angle_range').value)

        # State
        self.scan_data = None
        self.state = 'FORWARD'  # FORWARD, TURN_LEFT, TURN_RIGHT
        self.turn_direction = 1  # 1 = left, -1 = right

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Node started!')
        self.get_logger().info(f'  Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'  Angular speed: {self.angular_speed} rad/s')
        self.get_logger().info(f'  Obstacle threshold: {self.obstacle_threshold} m')

    def scan_callback(self, msg: LaserScan):
        """Store the latest laser scan data."""
        self.scan_data = msg

    def get_sector_min_distance(self, scan: LaserScan, angle_min: float, angle_max: float) -> float:
        """
        Get the minimum distance reading within an angular sector.
        Angles are in radians, measured from the robot's forward direction.
        """
        min_dist = float('inf')
        num_readings = len(scan.ranges)

        for i, dist in enumerate(scan.ranges):
            # Calculate the angle of this reading
            angle = scan.angle_min + i * scan.angle_increment

            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Check if this reading is within our sector
            if angle_min <= angle <= angle_max:
                # Filter out invalid readings
                if scan.range_min < dist < scan.range_max and not math.isnan(dist) and not math.isinf(dist):
                    min_dist = min(min_dist, dist)

        return min_dist

    def analyze_scan(self, scan: LaserScan):
        """
        Analyze the laser scan and return distances in key sectors.
        Returns: (front_dist, left_dist, right_dist)
        """
        # Front sector: -front_angle to +front_angle
        front_dist = self.get_sector_min_distance(scan, -self.front_angle, self.front_angle)

        # Left sector: front_angle to front_angle + side_angle
        left_dist = self.get_sector_min_distance(
            scan, self.front_angle, self.front_angle + self.side_angle
        )

        # Right sector: -(front_angle + side_angle) to -front_angle
        right_dist = self.get_sector_min_distance(
            scan, -(self.front_angle + self.side_angle), -self.front_angle
        )

        return front_dist, left_dist, right_dist

    def control_loop(self):
        """Main control loop - runs at 10 Hz."""
        cmd = Twist()

        if self.scan_data is None:
            # No scan data yet - stay still
            self.cmd_vel_pub.publish(cmd)
            return

        front_dist, left_dist, right_dist = self.analyze_scan(self.scan_data)

        # Log distances periodically (every ~2 seconds = 20 iterations)
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        if self._log_counter >= 20:
            self._log_counter = 0
            self.get_logger().info(
                f'State: {self.state} | Front: {front_dist:.2f}m | '
                f'Left: {left_dist:.2f}m | Right: {right_dist:.2f}m'
            )

        # State machine for obstacle avoidance
        if self.state == 'FORWARD':
            if front_dist < self.obstacle_threshold:
                # Obstacle ahead - decide which way to turn
                if left_dist >= right_dist:
                    self.turn_direction = 1   # Turn left
                    self.state = 'TURN_LEFT'
                    self.get_logger().info(f'Obstacle at {front_dist:.2f}m - Turning LEFT')
                else:
                    self.turn_direction = -1  # Turn right
                    self.state = 'TURN_RIGHT'
                    self.get_logger().info(f'Obstacle at {front_dist:.2f}m - Turning RIGHT')
            else:
                # Path is clear - move forward
                cmd.linear.x = self.linear_speed
                # Slight steering to avoid side obstacles
                if left_dist < self.obstacle_threshold * 1.5:
                    cmd.angular.z = -self.angular_speed * 0.3  # Steer right
                elif right_dist < self.obstacle_threshold * 1.5:
                    cmd.angular.z = self.angular_speed * 0.3   # Steer left

        elif self.state == 'TURN_LEFT':
            if front_dist > self.obstacle_threshold * 1.2:
                # Path is clear again
                self.state = 'FORWARD'
                self.get_logger().info('Path clear - resuming FORWARD')
            else:
                # Keep turning left
                cmd.angular.z = self.angular_speed

        elif self.state == 'TURN_RIGHT':
            if front_dist > self.obstacle_threshold * 1.2:
                # Path is clear again
                self.state = 'FORWARD'
                self.get_logger().info('Path clear - resuming FORWARD')
            else:
                # Keep turning right
                cmd.angular.z = -self.angular_speed

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Robot stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
