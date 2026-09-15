#!/usr/bin/env python3
"""Parameterized waypoint controller: follows a square course using the lab's
dead-reckoning estimate as feedback, publishing velocity setpoints to /cmd_vel.

Different gains/laws make different controllers to RANK in the benchmark. The
lab's ANFIS-DDPG/fuzzy/PPO controllers plug in identically (subscribe
/deadreckon/odom, publish TwistStamped /cmd_vel).

Usage: python3 waypoint_controller.py [--name N] [--kp-yaw K] [--kd-yaw K]
                                       [--cruise M] [--law p|pd|pursuit]
"""
import argparse
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

WAYPOINTS = [(3.0, 0.0), (3.0, 3.0), (0.0, 3.0), (0.0, 0.0)]
REACH = 0.6
DEPTH_TARGET = -0.6


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Ctrl:
    def __init__(self, node, a):
        self.node, self.a = node, a
        self.pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)
        node.create_subscription(Odometry, '/deadreckon/odom', self.on_odom, 10)
        self.wp = 0
        self.p0 = None
        self.prev_err = 0.0

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.p0 is None:
            self.p0 = (p.x, p.y)
        x, y, z = p.x - self.p0[0], p.y - self.p0[1], p.z
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        gx, gy = WAYPOINTS[self.wp]
        if math.hypot(gx - x, gy - y) < REACH:
            self.wp = (self.wp + 1) % len(WAYPOINTS)
            gx, gy = WAYPOINTS[self.wp]

        # pure-pursuit: aim a look-ahead point along the leg; else aim the waypoint
        if self.a.law == 'pursuit':
            px, py = WAYPOINTS[(self.wp - 1) % len(WAYPOINTS)]
            seg = np.array([gx - px, gy - py]); L = np.linalg.norm(seg) + 1e-9
            t = np.clip(np.dot([x - px, y - py], seg) / L**2, 0, 1)
            look = np.array([px, py]) + min(t + 0.4, 1.0) * seg
            gx, gy = look

        err = math.atan2(gy - y, gx - x) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        d_err = err - self.prev_err
        self.prev_err = err

        yaw_cmd = self.a.kp_yaw * err
        if self.a.law == 'pd':
            yaw_cmd += self.a.kd_yaw * d_err

        cmd = TwistStamped()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.twist.linear.x = self.a.cruise if abs(err) < 1.0 else 0.1
        cmd.twist.linear.z = float(np.clip(0.6 * (DEPTH_TARGET - z), -0.3, 0.3))
        cmd.twist.angular.z = float(np.clip(yaw_cmd, -0.8, 0.8))
        self.pub.publish(cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='ctrl')
    ap.add_argument('--kp-yaw', type=float, default=1.5)
    ap.add_argument('--kd-yaw', type=float, default=0.0)
    ap.add_argument('--cruise', type=float, default=0.3)
    ap.add_argument('--law', choices=['p', 'pd', 'pursuit'], default='p')
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('waypoint_controller')
    Ctrl(node, a)
    node.get_logger().info(f'controller {a.name}: law={a.law} kp={a.kp_yaw} cruise={a.cruise}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
