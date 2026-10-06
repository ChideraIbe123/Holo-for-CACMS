#!/usr/bin/env python3
"""Run one path-following law around a course, on the real vehicle or the simulator.

Reads the estimator's pose on /deadreckon/odom, publishes a velocity setpoint on
/cmd_vel (geometry_msgs/TwistStamped), the same interface as
sim/waypoint_controller.py. Differences from that controller:
  - the course is rotated to the vehicle's starting heading (tracker.py explains why)
  - no derivative terms, so the estimator's bursty timing does not matter
  - the run ends by itself after --laps laps (or --duration seconds), then sends zeros
  - every decision is logged with cross-track error and lap count

/cmd_vel convention (unchanged from the benchmark): linear.x forward m/s, linear.z
up m/s, angular.z yaw rate with POSITIVE = CLOCKWISE (the vehicle's stick
convention). Laws compute counter-clockwise-positive rates; --yaw-sign -1 converts.

Usage:
  python3 course_runner.py --law los --name los_r1 --log runs/los_r1.csv
  python3 course_runner.py --law p --course line
--depth-target Z holds depth with a proportional law on the throttle (sim: -0.5, pool: -0.4).
"""
import argparse
import math
import os
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tracker import LAWS, CourseTracker  # noqa: E402


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Runner:
    def __init__(self, node, a):
        self.node, self.a = node, a
        self.trk = CourseTracker(LAWS[a.law](), course=a.course, cruise=a.cruise,
                                 reach=a.reach, yaw_cap=a.yaw_cap, laps=a.laps,
                                 rotate=not a.no_rotate, line_length=a.line_length,
                                 line_offset=a.line_offset)
        self.pub = node.create_publisher(TwistStamped, a.cmd_topic, 10)
        node.create_subscription(Odometry, '/deadreckon/odom', self.on_odom, 50)
        self.t0 = None
        self.finished = False
        self.stop_sent = 0
        self.logf = None
        if a.log:
            os.makedirs(os.path.dirname(os.path.abspath(a.log)), exist_ok=True)
            self.logf = open(a.log, 'w')
            self.logf.write('t,stamp,lap,wp,x,y,yaw,e,s,dist_target,fwd,yaw_ccw,z\n')

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def publish(self, u, r_ccw, w=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(u)
        cmd.twist.linear.z = float(w)
        cmd.twist.angular.z = float(self.a.yaw_sign * r_ccw)
        self.pub.publish(cmd)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        t = self.now()
        if self.t0 is None:
            self.t0 = t
            self.node.get_logger().info(
                f'start: estimator heading {math.degrees(yaw_from_quat(msg.pose.pose.orientation)):+.0f} deg '
                f'-> course frame {"rotated to it" if not self.a.no_rotate else "NOT rotated"}')
        t -= self.t0
        if self.finished:
            return
        u, r, done = self.trk.update(t, p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        if done or (self.a.duration and t > self.a.duration):
            self.finished = True
            why = 'course complete' if done else 'time limit'
            self.node.get_logger().info(f'{why} at t={t:.1f} s, laps={self.trk.laps}')
            return
        w = 0.0
        if self.a.depth_target is not None:
            w = max(-0.3, min(0.3, 0.6 * (self.a.depth_target - p.z)))
        self.publish(u, r, w)
        o = self.trk.last_obs
        if self.logf and o is not None:
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.logf.write(f'{t:.3f},{st:.3f},{self.trk.laps},{self.trk.i},{o.x:.4f},{o.y:.4f},{o.yaw:.4f},'
                            f'{o.e:.4f},{o.s:.4f},{o.dist_target:.4f},{u:.3f},{r:.4f},{p.z:.3f}\n')

    def tick(self):
        """After the run: send explicit zeros for 1 s, then exit."""
        if self.finished:
            self.publish(0.0, 0.0, 0.0)
            self.stop_sent += 1
            if self.stop_sent > 20:
                raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--law', choices=sorted(LAWS), required=True)
    ap.add_argument('--name', default=None)
    ap.add_argument('--course', choices=['rect', 'line'], default='rect')
    ap.add_argument('--line-length', type=float, default=2.4, help='line course: metres of pipe to follow')
    ap.add_argument('--line-offset', type=float, default=0.3, help='line course: the line is this far to the LEFT of the start')
    ap.add_argument('--cruise', type=float, default=0.3)
    ap.add_argument('--reach', type=float, default=0.3)
    ap.add_argument('--yaw-cap', type=float, default=0.8)
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--duration', type=float, default=75.0, help='hard time limit, s (0 = none)')
    ap.add_argument('--yaw-sign', type=float, default=-1.0)
    ap.add_argument('--no-rotate', action='store_true', help='do not rotate the course to the start heading')
    ap.add_argument('--depth-target', type=float, default=None, help='hold this depth (m, negative down) with a P law on the up-velocity')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--log', default=None)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('course_runner')
    r = Runner(node, a)
    node.create_timer(0.05, r.tick)
    node.get_logger().info(f'{a.name or a.law}: law={a.law} course={a.course} cruise={a.cruise} laps={a.laps}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        for _ in range(5):
            r.publish(0.0, 0.0, 0.0)
        if r.logf:
            r.logf.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
