#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


ZONES = {
    'A': (0.0, 0.0),
    'B': (-4.0, 5.0),
    'C': (1.0, 5.0),
    'D': (5.0, -5.0),
    'E': (6.0, 2.0),
}


class GoalNavigationNode(Node):

    def __init__(self):
        super().__init__('goal_navigation_node')

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.ready = False
        self.goal_reached = False

        # Get target zone from user
        zone = input(
            "Enter target zone (A/B/C/D/E): "
        ).strip().upper()

        if zone not in ZONES:
            raise ValueError(
                f"Unknown zone '{zone}'"
            )

        self.target_x, self.target_y = ZONES[zone]

        self.get_logger().info(
            f"Target Zone {zone}: "
            f"({self.target_x}, {self.target_y})"
        )

        # Subscriber
        self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_cb,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/robot1/cmd_vel',
            10
        )

        # Control loop
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

    def odom_cb(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.yaw = math.atan2(
            siny,
            cosy
        )

        self.ready = True

    def normalize(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def control_loop(self):

        if self.goal_reached:
            return

        if not self.ready:
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y

        distance = math.hypot(dx, dy)

        cmd = Twist()

        # Goal reached
        if distance < 0.30:

            self.stop_robot()

            self.goal_reached = True

            self.get_logger().info(
                f"Goal reached at "
                f"({self.x:.2f}, {self.y:.2f})"
            )

            return

        desired_heading = math.atan2(
            dy,
            dx
        )

        heading_error = self.normalize(
            desired_heading - self.yaw
        )

        # Rotate first
        if abs(heading_error) > 0.30:

            cmd.angular.z = (
                0.8
                if heading_error > 0.0
                else -0.8
            )

        else:

            cmd.linear.x = 0.3

            cmd.angular.z = (
                0.8 * heading_error
            )

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = GoalNavigationNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
