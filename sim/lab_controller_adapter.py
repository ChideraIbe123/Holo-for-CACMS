#!/usr/bin/env python3
"""Drop-in ROS 2 adapter for the lab's real controllers (ANFIS-DDPG / fuzzy / PPO).

The lab's controllers (e.g. AUVSL/anfis_rl) are planar path-trackers: they read
(x, y, yaw) from an Odometry topic, hold a constant cruise speed, and output a
yaw-rate control law. This node owns everything around that: it subscribes the
dead-reckoning estimate, walks the benchmark course, computes the path-relative
state the lab's laws consume, applies the control-path yaw-sign convention and
depth hold, and publishes /cmd_vel — so porting a lab controller to the benchmark
means implementing ONE class, with no ROS code at all:

    class MyLaw:
        cruise = 0.3                      # optional, m/s (else --cruise)
        def reset(self): ...              # optional, called once at start
        def compute(self, obs) -> float:  # yaw-rate (rad/s), + = turn left
            ...

`obs` is a LawObs with the anfis_rl state signature plus raw pose:
    dist_error  signed cross-track distance to the current path segment (m);
                positive = vehicle is LEFT of the segment direction
    theta_near  heading error to the path tangent (rad, wrapped)
    theta_far   heading error to the current target waypoint (rad, wrapped)
    x, y, yaw, t   raw planar pose (m, rad, s since first odom)

Run it exactly where waypoint_controller.py runs in run_benchmark.sh:
    python3 lab_controller_adapter.py --law mymodule:MyLaw [--cruise 0.3]
The default law (builtin PassthroughP) exercises the same plugin path and exists
to prove the adapter end-to-end; it is NOT one of the benchmarked controllers.

Yaw sign: defaults to -1, matching every benchmark controller (the real
vehicle's yaw command/response is inverted — teleop-data corr -0.73, confirmed
in closed loop). Lab laws should output the sign convention they trained with;
the adapter applies the flip.
"""
import argparse
import importlib
import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from waypoint_controller import DEPTH_TARGET, REACH, WAYPOINTS, yaw_from_quat


@dataclass
class LawObs:
    dist_error: float
    theta_near: float
    theta_far: float
    x: float
    y: float
    yaw: float
    t: float


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PassthroughP:
    """Trivial P heading law THROUGH the plugin path (adapter self-test only)."""
    cruise = 0.3
    kp = 1.4

    def compute(self, obs):
        return self.kp * obs.theta_far


def load_law(spec):
    """'module:Class' -> instance. 'self:X' loads from this file."""
    mod_name, _, cls_name = spec.partition(":")
    if not cls_name:
        raise SystemExit(f"--law must be module:Class, got {spec!r}")
    mod = globals() if mod_name == "self" else vars(importlib.import_module(mod_name))
    return mod[cls_name]()


class Adapter:
    def __init__(self, node, law, a):
        self.node, self.law, self.a = node, law, a
        self.cruise = a.cruise if a.cruise is not None else getattr(law, "cruise", 0.3)
        self.pub = node.create_publisher(TwistStamped, "/cmd_vel", 10)
        node.create_subscription(Odometry, "/deadreckon/odom", self.on_odom, 10)
        self.wp = 0
        self.p0 = None
        self.t0 = None
        if hasattr(law, "reset"):
            law.reset()

    def path_state(self, x, y, yaw):
        """Signed cross-track + heading errors vs the current course segment."""
        gx, gy = WAYPOINTS[self.wp]
        if math.hypot(gx - x, gy - y) < REACH:
            self.wp = (self.wp + 1) % len(WAYPOINTS)
            gx, gy = WAYPOINTS[self.wp]
        px, py = WAYPOINTS[(self.wp - 1) % len(WAYPOINTS)]
        seg = np.array([gx - px, gy - py])
        L = np.linalg.norm(seg) + 1e-9
        tang = seg / L
        off = np.array([x - px, y - py])
        # z of tang x off: positive when the vehicle is left of the segment
        dist_error = float(tang[0] * off[1] - tang[1] * off[0])
        theta_near = wrap(math.atan2(tang[1], tang[0]) - yaw)
        theta_far = wrap(math.atan2(gy - y, gx - x) - yaw)
        return dist_error, theta_near, theta_far

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.p0 is None:
            self.p0 = (p.x, p.y)
            self.t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y, z = p.x - self.p0[0], p.y - self.p0[1], p.z
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.t0

        dist_error, theta_near, theta_far = self.path_state(x, y, yaw)
        obs = LawObs(dist_error, theta_near, theta_far, x, y, yaw, t)
        yaw_rate = float(self.law.compute(obs)) * self.a.yaw_sign

        cmd = TwistStamped()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.twist.linear.x = self.cruise if abs(theta_far) < 1.0 else 0.1
        cmd.twist.linear.z = float(np.clip(0.6 * (DEPTH_TARGET - z), -0.3, 0.3))
        cmd.twist.angular.z = float(np.clip(yaw_rate, -0.8, 0.8))
        self.pub.publish(cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="lab_ctrl")
    ap.add_argument("--law", default="self:PassthroughP",
                    help="module:Class implementing compute(obs)->yaw_rate")
    ap.add_argument("--cruise", type=float, default=None,
                    help="override the law's cruise speed (m/s)")
    ap.add_argument("--yaw-sign", type=float, default=-1.0,
                    help="control-path heading convention (benchmark default -1)")
    a = ap.parse_args()
    law = load_law(a.law)
    rclpy.init()
    node = rclpy.create_node("lab_controller_adapter")
    Adapter(node, law, a)
    node.get_logger().info(
        f"{a.name}: law={a.law} cruise={a.cruise or getattr(law, 'cruise', 0.3)} "
        f"yaw_sign={a.yaw_sign}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
