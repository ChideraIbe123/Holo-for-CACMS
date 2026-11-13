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
--depth-target Z holds depth with a proportional law on the throttle (sim and pool: -0.5).

--dive-first (real vehicle): the simulator starts every run already at depth and still,
so the real run must too. The estimator zeroes depth at /deadreckon/reset, which
run_one.sh sends while the vehicle floats, so depth is metres below the floating
position. The runner then (1) refuses to start unless depth reads ~0 (vehicle floating,
reset done), (2) sinks with zero surge/yaw until the depth sensor sits within
--settle-tol of the target for --settle-hold seconds, (3) lays out the course from the
pose at that moment, and (4) records depth and DVL altitude before and during the run
in depth_gate.json next to the log. Nothing changes without the flag.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tracker import LAWS, CourseTracker  # noqa: E402


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def depth_cmd(target, z, gain=0.6, cap=0.3):
    """Up-velocity command (m/s) holding depth with a P law; unchanged from the original."""
    return max(-cap, min(cap, gain * (target - z)))


class DepthGate:
    """Sensor-verified dive before the course starts. Pure python, no ROS.

    feed(t, z) -> (phase, w): phase is 'check', 'dive', 'course' or 'abort';
    w is the up-velocity to publish while diving. record(z, alt) tracks depth
    during the course; summary() is what gets written to depth_gate.json.
    """

    def __init__(self, target, start_tol=0.10, settle_tol=0.05, settle_hold=1.0,
                 timeout=12.0):
        self.target = target
        self.start_tol, self.settle_tol = start_tol, settle_tol
        self.settle_hold, self.timeout = settle_hold, timeout
        self.phase = 'check'
        self.t_start = None
        self.z_start = None
        self.settled_since = None
        self.dive_s = None
        self.timed_out = False
        self.z_course_start = None
        self.alt_course_start = None
        self.reason = None
        self._zs = []
        self._alt = []

    def feed(self, t, z):
        if self.phase in ('course', 'abort'):
            return self.phase, 0.0
        if self.phase == 'check':
            self.t_start, self.z_start = t, z
            if abs(z) > self.start_tol:
                self.phase = 'abort'
                self.reason = (f'depth reads {z:+.2f} m at start (limit +/-{self.start_tol:.2f}). '
                               'The vehicle must be floating freely when the run starts and '
                               '/deadreckon/reset must have been sent there. Re-float, re-run.')
                return self.phase, 0.0
            self.phase = 'dive'
        inside = abs(z - self.target) < self.settle_tol
        if inside:
            if self.settled_since is None:
                self.settled_since = t
            if t - self.settled_since >= self.settle_hold:
                return self._start(t, z)
        else:
            self.settled_since = None
        if t - self.t_start >= self.timeout:
            self.timed_out = True
            return self._start(t, z)
        return 'dive', depth_cmd(self.target, z)

    def _start(self, t, z):
        self.phase = 'course'
        self.dive_s = t - self.t_start
        self.z_course_start = z
        return 'course', 0.0

    def record(self, z, alt=None):
        self._zs.append(z)
        if alt is not None:
            self._alt.append(alt)

    def summary(self):
        s = dict(target=self.target, z_at_start=self.z_start, dive_s=self.dive_s,
                 settled=(self.dive_s is not None and not self.timed_out),
                 timed_out=self.timed_out, z_course_start=self.z_course_start,
                 alt_course_start=self.alt_course_start, phase=self.phase, reason=self.reason)
        if self._zs:
            n = len(self._zs)
            mean = sum(self._zs) / n
            s.update(course_z_mean=mean, course_z_std=(sum((v - mean) ** 2 for v in self._zs) / n) ** 0.5,
                     course_z_max_dev=max(abs(v - self.target) for v in self._zs), course_samples=n)
        if self._alt:
            s.update(course_alt_mean=sum(self._alt) / len(self._alt), course_alt_min=min(self._alt))
        return s


class Runner:
    def __init__(self, node, a):
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32
        self.TwistStamped = TwistStamped
        self.node, self.a = node, a
        self.trk = CourseTracker(LAWS[a.law](), course=a.course, cruise=a.cruise,
                                 reach=a.reach, yaw_cap=a.yaw_cap, laps=a.laps,
                                 rotate=not a.no_rotate, line_length=a.line_length,
                                 line_offset=a.line_offset)
        self.pub = node.create_publisher(TwistStamped, a.cmd_topic, 10)
        node.create_subscription(Odometry, '/deadreckon/odom', self.on_odom, 50)
        node.create_subscription(Float32, '/dvl/altitude', self.on_alt, 10)
        self.alt = None
        self.gate = None
        if a.dive_first:
            if a.depth_target is None:
                raise SystemExit('--dive-first needs --depth-target')
            self.gate = DepthGate(a.depth_target, start_tol=a.start_tol, settle_tol=a.settle_tol,
                                  settle_hold=a.settle_hold, timeout=a.dive_timeout)
        self.t0 = None
        self.finished = False
        self.aborted = False
        self.last_act = -1e9
        self.stop_sent = 0
        self.logf = None
        if a.log:
            os.makedirs(os.path.dirname(os.path.abspath(a.log)), exist_ok=True)
            self.logf = open(a.log, 'w')
            self.logf.write('t,stamp,lap,wp,x,y,yaw,e,s,dist_target,fwd,yaw_ccw,z\n')

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def publish(self, u, r_ccw, w=0.0):
        cmd = self.TwistStamped()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(u)
        cmd.twist.linear.z = float(w)
        cmd.twist.angular.z = float(self.a.yaw_sign * r_ccw)
        self.pub.publish(cmd)

    def on_alt(self, msg):
        self.alt = float(msg.data)

    def write_gate(self):
        if self.gate is None or not self.a.log:
            return
        path = os.path.join(os.path.dirname(os.path.abspath(self.a.log)), 'depth_gate.json')
        with open(path, 'w') as f:
            json.dump(self.gate.summary(), f, indent=1)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        t = self.now()
        if self.finished:
            return
        if self.gate is not None and self.gate.phase != 'course':
            phase, w = self.gate.feed(t, p.z)
            if phase == 'abort':
                self.node.get_logger().error('NOT STARTING: ' + self.gate.reason)
                self.finished = self.aborted = True
                return
            if phase == 'dive':
                self.publish(0.0, 0.0, w)
                return
            # phase just became 'course': the course is laid out from THIS pose
            self.gate.alt_course_start = self.alt
            g = self.gate
            self.node.get_logger().info(
                f'at depth: {g.z_course_start:+.3f} m after {g.dive_s:.1f} s '
                f'({"settled" if not g.timed_out else "TIMED OUT, starting anyway"}), '
                f'dvl altitude {self.alt if self.alt is not None else float("nan"):.2f} m')
        if self.t0 is None:
            self.t0 = t
            self.node.get_logger().info(
                f'start: estimator heading {math.degrees(yaw_from_quat(msg.pose.pose.orientation)):+.0f} deg '
                f'-> course frame {"rotated to it" if not self.a.no_rotate else "NOT rotated"}')
        t -= self.t0
        if self.a.max_rate > 0:
            if t - self.last_act < 1.0 / self.a.max_rate:
                return
            self.last_act = t
        u, r, done = self.trk.update(t, p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        if done or (self.a.duration and t > self.a.duration):
            self.finished = True
            why = 'course complete' if done else 'time limit'
            self.node.get_logger().info(f'{why} at t={t:.1f} s, laps={self.trk.laps}')
            if self.gate is not None:
                s = self.gate.summary()
                self.node.get_logger().info(
                    f"depth during course: mean {s.get('course_z_mean', float('nan')):+.3f} m, "
                    f"max deviation {s.get('course_z_max_dev', float('nan')):.3f} m")
            return
        w = 0.0
        if self.a.depth_target is not None:
            w = depth_cmd(self.a.depth_target, p.z)
        self.publish(u, r, w)
        if self.gate is not None:
            self.gate.record(p.z, self.alt)
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
                raise SystemExit(2 if self.aborted else 0)


def main():
    import rclpy
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
    ap.add_argument('--dive-first', action='store_true',
                    help='real vehicle: check depth reads ~0 (floating, reset done), sink to --depth-target '
                         'and settle BEFORE laying out the course; records depth_gate.json next to --log')
    ap.add_argument('--start-tol', type=float, default=0.10, help='dive-first: |depth| at start must be under this (m)')
    ap.add_argument('--settle-tol', type=float, default=0.05, help='dive-first: at depth when within this of the target (m)')
    ap.add_argument('--settle-hold', type=float, default=1.0, help='dive-first: must stay within tolerance this long (s)')
    ap.add_argument('--dive-timeout', type=float, default=12.0, help='dive-first: start anyway after this many seconds (flagged)')
    ap.add_argument('--max-rate', type=float, default=0.0,
                    help='diagnostic: cap the control loop at this rate (Hz) by '
                         'dropping odom messages; 0 (default) = act on every message. '
                         'Emulates the real-session estimator rate (~10.5 Hz).')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--log', default=None)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('course_runner')
    r = Runner(node, a)
    node.create_timer(0.05, r.tick)
    node.get_logger().info(f'{a.name or a.law}: law={a.law} course={a.course} cruise={a.cruise} laps={a.laps} '
                           f'depth={a.depth_target} dive_first={a.dive_first}')
    code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        code = int(e.code or 0)
    finally:
        for _ in range(5):
            r.publish(0.0, 0.0, 0.0)
        if r.logf:
            r.logf.close()
        r.write_gate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
