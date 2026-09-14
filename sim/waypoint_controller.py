#!/usr/bin/env python3
"""Demo closed-loop controller: follow a square waypoint course using the lab's
dead-reckoning estimate as feedback, publishing velocity setpoints the sim's
autopilot consumes. This is the reference "controller" that proves the
closed-loop path; the lab's ANFIS-DDPG/fuzzy/PPO controllers plug in the same way
(subscribe to /deadreckon/odom, publish TwistStamped to /cmd_vel).

Usage: python3 waypoint_controller.py [cruise_mps]
"""
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

WAYPOINTS = [(3.0, 0.0), (3.0, 3.0), (0.0, 3.0), (0.0, 0.0)]
REACH = 0.6            # m, waypoint switch radius
CRUISE = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3   # m/s surge
KP_YAW = 1.5          # rad/s per rad heading error
DEPTH_TARGET = -0.6


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class WaypointController:
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)
        node.create_subscription(Odometry, '/deadreckon/odom', self.on_odom, 10)
        self.wp = 0
        self.p0 = None

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.p0 is None:
            self.p0 = (p.x, p.y)     # zero the course at the DR origin
        x, y, z = p.x - self.p0[0], p.y - self.p0[1], p.z
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        gx, gy = WAYPOINTS[self.wp]
        if math.hypot(gx - x, gy - y) < REACH:
            self.wp = (self.wp + 1) % len(WAYPOINTS)
            gx, gy = WAYPOINTS[self.wp]

        heading_err = math.atan2(gy - y, gx - x) - yaw
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        cmd = TwistStamped()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.twist.linear.x = CRUISE if abs(heading_err) < 1.0 else 0.1  # slow in sharp turns
        cmd.twist.linear.z = float(np.clip(0.6 * (DEPTH_TARGET - z), -0.3, 0.3))
        cmd.twist.angular.z = float(np.clip(KP_YAW * heading_err, -0.8, 0.8))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = rclpy.create_node('waypoint_controller')
    WaypointController(node)
    node.get_logger().info('waypoint controller: following square course via /deadreckon/odom')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
