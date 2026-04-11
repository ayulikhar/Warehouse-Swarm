#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

NUM_ROBOTS = 3
ROBOT_NAMES = ['robot1', 'robot2', 'robot3']

MAX_LINEAR_SPEED  = 0.3
MAX_ANGULAR_SPEED = 0.8
MIN_LINEAR_SPEED  = 0.05

W_SEPARATION = 2.5
W_ALIGNMENT  = 1.0
W_COHESION   = 0.8
W_OBSTACLE   = 3.0

SEPARATION_RADIUS = 1.5
NEIGHBOR_RADIUS   = 4.0

OBSTACLE_THRESHOLD = 0.8
FRONT_ANGLE_DEG    = 40.0
SIDE_ANGLE_DEG     = 70.0


class RobotState:
    def __init__(self, name: str):
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.scan = None
        self.odom_received = False
        self.scan_received = False

    @property
    def ready(self):
        return self.odom_received and self.scan_received


class SwarmControllerNode(Node):

    def __init__(self):
        super().__init__('swarm_controller_node')

        self.sep_radius = SEPARATION_RADIUS
        self.nbr_radius = NEIGHBOR_RADIUS
        self.w_sep = W_SEPARATION
        self.w_ali = W_ALIGNMENT
        self.w_coh = W_COHESION
        self.w_obs = W_OBSTACLE
        self.max_lin = MAX_LINEAR_SPEED
        self.max_ang = MAX_ANGULAR_SPEED
        self.obs_thresh = OBSTACLE_THRESHOLD

        self.robots = {name: RobotState(name) for name in ROBOT_NAMES}

        sensor_qos = QoSProfile(depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        reliable_qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        for name in ROBOT_NAMES:
            self.create_subscription(Odometry, f'/{name}/odom',
                lambda msg, n=name: self._odom_callback(msg, n), reliable_qos)

            self.create_subscription(LaserScan, f'/{name}/scan',
                lambda msg, n=name: self._scan_callback(msg, n), sensor_qos)

            self.robots[name].pub = self.create_publisher(
                Twist, f'/{name}/cmd_vel', reliable_qos)

        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info("Swarm Controller Node started!")

    # ------------------ Callbacks ------------------

    def _odom_callback(self, msg, name):
        r = self.robots[name]
        r.x = msg.pose.pose.position.x
        r.y = msg.pose.pose.position.y
        r.vx = msg.twist.twist.linear.x

        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        r.yaw = math.atan2(siny, cosy)

        r.odom_received = True

    def _scan_callback(self, msg, name):
        r = self.robots[name]
        r.scan = msg
        r.scan_received = True

    # ------------------ Helpers ------------------

    def _distance(self, r1, r2):
        return math.hypot(r1.x - r2.x, r1.y - r2.y)

    def _angle_to(self, r1, r2):
        return math.atan2(r2.y - r1.y, r2.x - r1.x)

    def _normalize(self, angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

    # ------------------ Boids ------------------

    def _rule_separation(self, robot, others):
        dx, dy = 0, 0
        for o in others:
            d = self._distance(robot, o)
            if 0.01 < d < self.sep_radius:
                dx -= (o.x - robot.x) / d
                dy -= (o.y - robot.y) / d
        return dx, dy

    def _rule_alignment(self, robot, others):
        if not others:
            return math.cos(robot.yaw), math.sin(robot.yaw)
        avg_x = sum(math.cos(o.yaw) for o in others)/len(others)
        avg_y = sum(math.sin(o.yaw) for o in others)/len(others)
        return avg_x, avg_y

    def _rule_cohesion(self, robot, others):
        if not others:
            return 0, 0
        cx = sum(o.x for o in others)/len(others)
        cy = sum(o.y for o in others)/len(others)
        return cx-robot.x, cy-robot.y

    # ------------------ Control Loop ------------------

    def _control_loop(self):

        ready = [r for r in self.robots.values() if r.ready]
        if not ready:
            return

        for robot in ready:

            neighbors = [o for o in ready
                         if o.name != robot.name
                         and self._distance(robot, o) < self.nbr_radius]

            # 🔥 AGGREGATION (NEW)
            if len(neighbors) == 0:
                closest = None
                min_d = float('inf')

                for o in ready:
                    if o.name == robot.name:
                        continue
                    d = self._distance(robot, o)
                    if d < min_d:
                        min_d = d
                        closest = o

                if closest:
                    angle = self._angle_to(robot, closest)
                    error = self._normalize(angle - robot.yaw)

                    cmd = Twist()
                    cmd.angular.z = max(min(error, self.max_ang), -self.max_ang)
                    cmd.linear.x = self.max_lin * 0.6

                    robot.pub.publish(cmd)
                    continue

            # --- BOIDS ---
            sep_x, sep_y = self._rule_separation(robot, ready)
            ali_x, ali_y = self._rule_alignment(robot, neighbors)
            coh_x, coh_y = self._rule_cohesion(robot, neighbors)

            dx = self.w_sep*sep_x + self.w_ali*ali_x + self.w_coh*coh_x
            dy = self.w_sep*sep_y + self.w_ali*ali_y + self.w_coh*coh_y

            cmd = Twist()

            if abs(dx) + abs(dy) > 0:
                desired = math.atan2(dy, dx)
                error = self._normalize(desired - robot.yaw)

                cmd.angular.z = max(min(error, self.max_ang), -self.max_ang)

                turn_factor = 1 - abs(cmd.angular.z)/self.max_ang
                cmd.linear.x = max(MIN_LINEAR_SPEED,
                                   self.max_lin * turn_factor)

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
