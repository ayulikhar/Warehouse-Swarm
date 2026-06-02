#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

NUM_ROBOTS = 3
ROBOT_NAMES = ['robot1', 'robot2', 'robot3']

MAX_LINEAR_SPEED  = 0.3
MAX_ANGULAR_SPEED = 0.8
MIN_LINEAR_SPEED  = 0.05

W_SEPARATION = 2.5
W_ALIGNMENT  = 1.0
W_COHESION   = 0.8

SEPARATION_RADIUS = 1.5
NEIGHBOR_RADIUS   = 4.0


class RobotState:
    def __init__(self, name):
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scan = None
        self.ready = False


class SwarmControllerNode(Node):

    def __init__(self):
        super().__init__('swarm_controller_node')

        self.robots = {name: RobotState(name) for name in ROBOT_NAMES}

        for name in ROBOT_NAMES:
            self.create_subscription(Odometry, f'/{name}/odom',
                                     lambda msg, n=name: self.odom_cb(msg, n), 10)

            self.create_subscription(LaserScan, f'/{name}/scan',
                                     lambda msg, n=name: self.scan_cb(msg, n), 10)

            self.robots[name].pub = self.create_publisher(
                Twist, f'/{name}/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(" Swarm Controller (Override Mode) Started")

    # Callbacks 

    def odom_cb(self, msg, name):
        r = self.robots[name]
        r.x = msg.pose.pose.position.x
        r.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        r.yaw = math.atan2(siny, cosy)

        r.ready = True

    def scan_cb(self, msg, name):
        self.robots[name].scan = msg

    # Helpers 

    def dist(self, r1, r2):
        return math.hypot(r1.x - r2.x, r1.y - r2.y)

    def angle_to(self, r1, r2):
        return math.atan2(r2.y - r1.y, r2.x - r1.x)

    def normalize(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    # Boids 

    def separation(self, robot, others):
        dx, dy = 0, 0
        for o in others:
            d = self.dist(robot, o)
            if 0.01 < d < SEPARATION_RADIUS:
                dx -= (o.x - robot.x) / d
                dy -= (o.y - robot.y) / d
        return dx, dy

    def alignment(self, robot, others):
        if not others:
            return math.cos(robot.yaw), math.sin(robot.yaw)
        avg_x = sum(math.cos(o.yaw) for o in others)/len(others)
        avg_y = sum(math.sin(o.yaw) for o in others)/len(others)
        return avg_x, avg_y

    def cohesion(self, robot, others):
        if not others:
            return 0, 0
        cx = sum(o.x for o in others)/len(others)
        cy = sum(o.y for o in others)/len(others)
        return cx - robot.x, cy - robot.y

    # Control Loop 

    def control_loop(self):

        ready = [r for r in self.robots.values() if r.ready]
        if not ready:
            return

        for robot in ready:

            # OBSTACLE OVERRIDE 
            if robot.scan:
                valid = [d for d in robot.scan.ranges if not math.isinf(d)]
                if valid:
                    front = min(valid)

                    if front < 0.6:
                        cmd = Twist()
                        cmd.linear.x = 0.0
                        cmd.angular.z = MAX_ANGULAR_SPEED
                        robot.pub.publish(cmd)
                        continue

            # Neighbor detection
            neighbors = [o for o in ready
                         if o.name != robot.name
                         and self.dist(robot, o) < NEIGHBOR_RADIUS]

            # AGGREGATION
            if len(neighbors) == 0:
                closest = None
                min_d = float('inf')

                for o in ready:
                    if o.name == robot.name:
                        continue
                    d = self.dist(robot, o)
                    if d < min_d:
                        min_d = d
                        closest = o

                if closest:
                    angle = self.angle_to(robot, closest)
                    error = self.normalize(angle - robot.yaw)

                    cmd = Twist()
                    cmd.angular.z = max(min(error, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
                    cmd.linear.x = MAX_LINEAR_SPEED * 0.6

                    robot.pub.publish(cmd)
                    continue

            # BOIDS
            sep_x, sep_y = self.separation(robot, ready)
            ali_x, ali_y = self.alignment(robot, neighbors)
            coh_x, coh_y = self.cohesion(robot, neighbors)

            dx = W_SEPARATION*sep_x + W_ALIGNMENT*ali_x + W_COHESION*coh_x
            dy = W_SEPARATION*sep_y + W_ALIGNMENT*ali_y + W_COHESION*coh_y

            cmd = Twist()

            if abs(dx) + abs(dy) > 0:
                desired = math.atan2(dy, dx)
                error = self.normalize(desired - robot.yaw)

                cmd.angular.z = max(min(error, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)

                turn_factor = 1 - abs(cmd.angular.z)/MAX_ANGULAR_SPEED
                cmd.linear.x = max(MIN_LINEAR_SPEED,
                                   MAX_LINEAR_SPEED * turn_factor)

            robot.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

