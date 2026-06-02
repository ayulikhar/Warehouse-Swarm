#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


A = (0.0,  0.0)
B = (-4.0, 5.0)
C = (1.0,  5.0)
D = (5.0, -5.0)
E = (6.0,  2.0)

ZONES = {'A': A, 'B': B, 'C': C, 'D': D, 'E': E}


FOLLOW_DISTANCE   = 1.2
LEADER_MAX_LINEAR = 0.5
LEADER_MAX_ANGULAR= 1.2
LEADER_KP_LINEAR  = 0.4
LEADER_KP_ANGULAR = 1.0
LEADER_GOAL_TOL   = 0.15

FOLLOW_KP_LINEAR  = 0.35
FOLLOW_KP_ANGULAR = 0.8
FOLLOW_MAX_LINEAR = 0.6
FOLLOW_MAX_ANGULAR= 1.4
FOLLOW_MIN_LINEAR = 0.05

CONTROL_RATE_HZ   = 20.0


def get_yaw(o):
    return math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y**2 + o.z**2))

def normalize_angle(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PlatooningNode(Node):
    def __init__(self):
        super().__init__('platooning_node')

        self._pose = {'robot1': None, 'robot2': None, 'robot3': None}
        self._target_x, self._target_y = ZONES['A']
        self._current_zone = 'A'

        self._scan = None

        qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(Odometry, '/robot1/odom', lambda m: self._odom_cb(m,'robot1'), qos)
        self.create_subscription(Odometry, '/robot2/odom', lambda m: self._odom_cb(m,'robot2'), qos)
        self.create_subscription(Odometry, '/robot3/odom', lambda m: self._odom_cb(m,'robot3'), qos)
        self.create_subscription(String, '/zone_command', self._zone_cb, 10)

        self.create_subscription(LaserScan, '/robot1/scan', self._scan_cb, 10)

        self._pub_r1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self._pub_r2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self._pub_r3 = self.create_publisher(Twist, '/robot3/cmd_vel', 10)

        self.create_timer(1.0/CONTROL_RATE_HZ, self._loop)

    def _odom_cb(self, msg, rid):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._pose[rid] = {'x': p.x, 'y': p.y, 'yaw': get_yaw(o)}

    def _zone_cb(self, msg):
        z = msg.data.strip().upper()
        if z in ZONES:
            self._target_x, self._target_y = ZONES[z]
            self._current_zone = z

    def _scan_cb(self, msg):
        self._scan = msg

    def _obstacle_ahead(self):
        if self._scan is None:
            return False
        ranges = self._scan.ranges
        center = len(ranges)//2
        front = ranges[center-10:center+10]
        return min(front) < 0.6

    def _loop(self):
        self._leader()
        self._follower('robot2','robot1')
        self._follower('robot3','robot2')

    def _leader(self):
        p = self._pose['robot1']
        if not p: return

        dx = self._target_x - p['x']
        dy = self._target_y - p['y']
        dist = math.hypot(dx,dy)

        cmd = Twist()

        # 🚧 obstacle override
        if self._obstacle_ahead():
            cmd.linear.x = 0.05
            cmd.angular.z = 0.5
            self._pub_r1.publish(cmd)
            return

        if dist < LEADER_GOAL_TOL:
            self._pub_r1.publish(cmd)
            return

        desired = math.atan2(dy,dx)
        err = normalize_angle(desired - p['yaw'])

        cmd.angular.z = clamp(LEADER_KP_ANGULAR*err, -0.8, 0.8)

        cmd.linear.x = clamp(
            LEADER_KP_LINEAR*dist*max(0.1,math.cos(err)),
            FOLLOW_MIN_LINEAR,
            LEADER_MAX_LINEAR)

        self._pub_r1.publish(cmd)

    def _follower(self, fid, lid):
        fp = self._pose[fid]
        lp = self._pose[lid]
        if not fp or not lp: return

        target_x = lp['x'] - FOLLOW_DISTANCE*math.cos(lp['yaw'])
        target_y = lp['y'] - FOLLOW_DISTANCE*math.sin(lp['yaw'])

        dx = target_x - fp['x']
        dy = target_y - fp['y']
        dist = math.hypot(dx,dy)

        gap = math.hypot(lp['x']-fp['x'], lp['y']-fp['y'])
        gap_error = gap - FOLLOW_DISTANCE

        cmd = Twist()

        # HARD STOP
        if gap < 0.6:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            pub = self._pub_r2 if fid=='robot2' else self._pub_r3
            pub.publish(cmd)
            return

        desired = math.atan2(dy,dx)
        err = normalize_angle(desired - fp['yaw'])

        cmd.angular.z = clamp(FOLLOW_KP_ANGULAR*err, -0.8, 0.8)

        # ANTI-REVERSE FIX
        if abs(err) > 1.5:
            cmd.linear.x = 0.0
        else:
            raw = FOLLOW_KP_LINEAR * dist * max(0.2, math.cos(err))

            if gap_error < 0:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = clamp(raw, 0.05, FOLLOW_MAX_LINEAR)

        pub = self._pub_r2 if fid=='robot2' else self._pub_r3
        pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PlatooningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

