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

--init (real vehicle, replaces --dive-first): a repeatable start for every run.
(1) refuses to start unless depth reads ~0 (vehicle floating, reset done); (2) sinks to
--depth-target and at the same time turns on the spot to the saved course heading
(theta = 0, pointing at the back wall, one heading for both courses; saved with
save_heading.py into heading_ref.json);
(3) starts the course only when depth is within --settle-tol AND heading within
--heading-tol, both continuously for --init-hold seconds; (4) lays the course out from
that position with its x axis exactly on the saved heading, and tells the stick
converter to re-anchor its geofence there. If it cannot settle within --init-timeout s
it does NOT start (exit code 3). Writes start_gate.json and init.csv next to the log.
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


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def heading_cmd(target, yaw, gain=1.0, cap=0.4):
    """Counter-clockwise yaw-rate command (rad/s) turning on the spot to `target`."""
    return max(-cap, min(cap, gain * wrap(target - yaw)))


def load_heading(path, course=None):
    """The saved heading (rad, estimator frame) from save_heading.py's file. One heading,
    theta = 0 at the back wall, is shared by both courses."""
    try:
        with open(path) as f:
            ref = json.load(f)['heading']
    except (OSError, KeyError, ValueError):
        raise SystemExit(f'no saved heading in {path}. Point the ROV at the back wall and run:  '
                         'python3 save_heading.py')
    return float(ref['yaw_rad'])


class StartGate:
    """Repeatable start: sensor-verified depth AND heading before the course. Pure python.

    feed(t, z, yaw) -> (phase, w, r_ccw): phase is 'check', 'settle', 'course' or 'abort';
    w (m/s up) and r_ccw (rad/s) are the commands to publish while settling.
    record(z, alt) tracks depth during the course; summary() -> start_gate.json.
    """

    # PI, not P: on 2026-11-12 a P-only start-up stalled at -0.39 m (buoyancy) and 24 deg off
    # (a steady clockwise push; the 0.4 rad/s cap gave a 160 yaw stick, about 30 us of
    # thruster PWM, inside the T200 dead band). The integrators trim out those steady loads.
    # Floor guard (2026-11-12: the pool floor is ~0.70 m below the floating DVL, and the
    # 0.5 m dive overshot onto it). With a DVL altitude at the start, the target is capped
    # so min_alt stays below the vehicle; below min_alt it never pushes down.
    def __init__(self, depth, heading, start_tol=0.10, depth_tol=0.05,
                 heading_tol=math.radians(6.0), hold=1.0, timeout=45.0,
                 z_kp=0.6, z_ki=0.15, z_imax=0.15, z_cap=0.3, z_izone=0.10, z_kd=0.6,
                 yaw_kp=1.0, yaw_ki=0.08, yaw_imax=0.3, yaw_cap=0.8,
                 yaw_izone=math.radians(10.0), yaw_kd=0.6, yaw_min=0.5,
                 min_alt=0.30, guard_alt=0.20, floor_up=0.10):
        self.depth, self.heading = depth, heading
        self.depth_requested = depth
        self.min_alt, self.guard_alt, self.floor_up, self.z_izone = min_alt, guard_alt, floor_up, z_izone
        self.alt_start = None
        self.floor_hits = 0
        self.floor = []             # floor depths z - alt (m, below the floating position)
        self.floor_rejected = 0
        self.floor_locked = False   # the target is decided once, then kept
        self.low_count = 0          # consecutive DVL readings under guard_alt
        self.z_kd, self.yaw_kd, self.yaw_izone, self.yaw_min = z_kd, yaw_kd, yaw_izone, yaw_min
        self.target_raised = 0      # times the floor guard raised the depth target
        self.start_tol, self.depth_tol, self.heading_tol = start_tol, depth_tol, heading_tol
        self.hold, self.timeout = hold, timeout
        self.z_kp, self.z_ki, self.z_imax, self.z_cap = z_kp, z_ki, z_imax, z_cap
        self.yaw_kp, self.yaw_ki, self.yaw_imax, self.yaw_cap = yaw_kp, yaw_ki, yaw_imax, yaw_cap
        self.iz = 0.0               # depth integrator, m/s (negative = extra down push)
        self.iyaw = 0.0             # heading integrator, rad/s CCW
        self.t_last = None
        self.phase = 'check'
        self.t_start = self.z_start = self.yaw_start = None
        self.settled_since = None
        self.settle_s = None
        self.z_course_start = self.yaw_course_start = self.alt_course_start = None
        self.reason = None
        self.trace = []             # (t, z, yaw, heading_err, w, r_ccw)
        self._zs, self._alt = [], []

    def seed_floor(self, pairs):
        """(z, alt) readings taken while floating, before feed(): decides the target."""
        for z, alt in pairs:
            self._floor_sample(z, alt)
        if pairs and self.alt_start is None:
            self.alt_start = sorted(a for _, a in pairs)[len(pairs) // 2]
        if self.floor and not self.floor_locked:
            self._lock_target()

    def feed(self, t, z, yaw, alt=None, vz=None, yaw_rate=None):
        """alt: DVL height above the floor (m) or None; vz: vertical speed (m/s, UP +, from
        the depth sensor; the DVL's vertical velocity in /deadreckon/odom reads + going DOWN)
        or None; yaw_rate: gyro turn rate (rad/s, CCW +) or None. vz and yaw_rate damp the
        depth and heading loops (2026-11-12: undamped, the dive overshot 0.25 m into the
        floor guard and the heading swung +/-12 deg for 45 s without settling)."""
        if self.phase in ('course', 'abort'):
            return self.phase, 0.0, 0.0
        if self.phase == 'check':
            self.t_start, self.z_start, self.yaw_start = t, z, yaw
            if abs(z) > self.start_tol:
                self.phase = 'abort'
                self.reason = (f'depth reads {z:+.2f} m at start (limit +/-{self.start_tol:.2f}). '
                               'The vehicle must be floating freely when the run starts and '
                               '/deadreckon/reset must have been sent there. Re-float, re-run.')
                return self.phase, 0.0, 0.0
            if alt is not None and self.alt_start is None:
                self.alt_start = alt
            if self.floor_locked and self.depth > -0.10:
                self.phase = 'abort'
                self.reason = (f'floor only {-self._floor_med():.2f} m below the floating position: '
                               f'no room to dive while keeping {self.min_alt:.2f} m clearance.')
                return self.phase, 0.0, 0.0
            self.phase = 'settle'
        err = wrap(self.heading - yaw)
        if abs(z - self.depth) < self.depth_tol and abs(err) < self.heading_tol:
            if self.settled_since is None:
                self.settled_since = t
            if t - self.settled_since >= self.hold:
                self.phase = 'course'
                self.settle_s = t - self.t_start
                self.z_course_start, self.yaw_course_start = z, yaw
                return 'course', 0.0, 0.0
        else:
            self.settled_since = None
        if t - self.t_start >= self.timeout:
            self.phase = 'abort'
            self.reason = (f'not settled after {self.timeout:.0f} s: depth {z:+.2f} m '
                           f'(target {self.depth:+.2f}), heading error {math.degrees(err):+.1f} deg. '
                           'Course NOT started.')
            return self.phase, 0.0, 0.0
        dt = 0.0 if self.t_last is None else min(max(t - self.t_last, 0.0), 0.5)
        self.t_last = t
        if alt is not None and not self.floor_locked:
            # no floating readings before the start: decide the target from the first five
            if self.alt_start is None:
                self.alt_start = alt
            self._floor_sample(z, alt)
            if len(self.floor) >= 5:
                self._lock_target()
        ez = self.depth - z
        # depth integral only near the target, so it cannot wind up during the descent
        w, self.iz = self._pi(ez, dt if abs(ez) < self.z_izone else 0.0,
                              self.z_kp, self.z_ki, self.iz, self.z_imax, self.z_cap)
        if vz is not None:
            w = max(-self.z_cap, min(self.z_cap, w - self.z_kd * vz))
        self.low_count = self.low_count + 1 if (alt is not None and alt < self.guard_alt) else 0
        if self.low_count >= 2:
            # too close to the floor (two readings in a row): back up, drop any accumulated
            # down push, restart the hold, and raise the target so it keeps min_alt over
            # whatever the DVL sees underneath
            w, self.iz, self.settled_since = self.floor_up, max(self.iz, 0.0), None
            self.floor_hits += 1
            raised = min(z - (alt - self.min_alt), 0.0)
            if raised > self.depth + 0.01:
                self.depth = raised
                self.target_raised += 1
        # heading integral only within yaw_izone of the target, so it cannot drive a limit cycle
        r, self.iyaw = self._pi(err, dt if abs(err) < self.yaw_izone else 0.0,
                                self.yaw_kp, self.yaw_ki, self.iyaw, self.yaw_imax, self.yaw_cap)
        if yaw_rate is not None:
            r = max(-self.yaw_cap, min(self.yaw_cap, r - self.yaw_kd * yaw_rate))
        if abs(err) > self.heading_tol:
            # thruster dead band: below about a 200 yaw stick (0.5 rad/s) the vehicle does not
            # turn at all (2026-11-12: stuck 15 deg off for 30 s at a 100 stick). Push at least
            # that hard, unless it is already turning towards the target.
            toward = yaw_rate is not None and yaw_rate * err > 0 and abs(yaw_rate) > 0.15
            if not toward and abs(r) < self.yaw_min:
                r = math.copysign(self.yaw_min, err)
        self.trace.append((t - self.t_start, z, yaw, err, w, r))
        return 'settle', w, r

    def _floor_sample(self, z, alt, window=15, outlier=0.15):
        """Track the floor depth (z - alt) as the median of recent DVL readings and set the
        target to the requested depth or min_alt above that floor, whichever is shallower.
        Single bad readings are rejected (2026-11-12: one 0.22 m reading over a 0.58 m floor,
        then a lost lock, dragged the old one-way limit to -0.02 m: a run at the surface)."""
        f = z - alt
        if len(self.floor) >= 3 and abs(f - self._floor_med()) > outlier:
            self.floor_rejected += 1
            return
        self.floor = (self.floor + [f])[-window:]

    def _floor_med(self):
        return sorted(self.floor)[len(self.floor) // 2]

    def _lock_target(self):
        """Fix the target once: the requested depth or min_alt above the floor, whichever is
        shallower. Not re-estimated afterwards: on 2026-11-12 the live estimate wandered
        -0.58..-0.89 m as the vehicle tilted and every change restarted the hold timer."""
        self.depth = min(max(self.depth_requested, self._floor_med() + self.min_alt), 0.0)
        self.floor_locked = True

    @staticmethod
    def _pi(e, dt, kp, ki, i, imax, cap):
        """PI with clamped integrator; it stops winding while the output is saturated
        in the direction of the error (conditional integration)."""
        out = kp * e + i
        if abs(out) < cap or (out > 0) != (e > 0):
            i = max(-imax, min(imax, i + ki * e * dt))
        return max(-cap, min(cap, kp * e + i)), i

    def record(self, z, alt=None):
        self._zs.append(z)
        if alt is not None:
            self._alt.append(alt)

    def summary(self):
        deg = lambda v: None if v is None else math.degrees(v)  # noqa: E731
        s = dict(depth_target=self.depth, depth_requested=self.depth_requested,
                 alt_start=self.alt_start, min_alt=self.min_alt, floor_guard_samples=self.floor_hits,
                 floor_depth=(self._floor_med() if self.floor else None), floor_locked=self.floor_locked,
                 target_raised_by_guard=self.target_raised,
                 floor_samples_rejected=self.floor_rejected,
                 heading_target_deg=deg(self.heading),
                 depth_tol=self.depth_tol, heading_tol_deg=deg(self.heading_tol), hold_s=self.hold,
                 z_at_start=self.z_start, heading_at_start_deg=deg(self.yaw_start),
                 settle_s=self.settle_s, settled=self.phase == 'course',
                 z_course_start=self.z_course_start, heading_course_start_deg=deg(self.yaw_course_start),
                 heading_err_course_start_deg=(None if self.yaw_course_start is None
                                               else deg(wrap(self.heading - self.yaw_course_start))),
                 alt_course_start=self.alt_course_start, phase=self.phase, reason=self.reason,
                 depth_bias_at_start=self.iz, heading_bias_at_start=self.iyaw,
                 anchor=getattr(self, 'anchor', None))
        if self._zs:
            n = len(self._zs)
            mean = sum(self._zs) / n
            s.update(course_z_mean=mean, course_z_std=(sum((v - mean) ** 2 for v in self._zs) / n) ** 0.5,
                     course_z_max_dev=max(abs(v - self.depth) for v in self._zs), course_samples=n)
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
        self.alt_time = -1e9
        self.alt_wait_start = None
        self.alt_seed = []          # (z, alt) while floating, before the start-up begins
        self.yaw_rate, self.yaw_rate_time = None, -1e9
        self.z_hist = []            # (t, z) for the vertical speed
        self.placement = None
        self.gate = None
        self.init = None
        if a.init:
            from geometry_msgs.msg import PoseStamped
            if a.depth_target is None:
                raise SystemExit('--init needs --depth-target')
            if a.dive_first:
                raise SystemExit('use --init or --dive-first, not both')
            self.init = StartGate(a.depth_target, load_heading(a.heading_ref, a.course),
                                  start_tol=a.start_tol, depth_tol=a.settle_tol,
                                  heading_tol=math.radians(a.heading_tol), hold=a.init_hold,
                                  timeout=a.init_timeout, min_alt=a.min_altitude)
            self.PoseStamped = PoseStamped
            self.origin_pub = node.create_publisher(PoseStamped, '/course/origin', 10)
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Imu
            node.create_subscription(Imu, '/mavros/imu/data', self.on_imu, qos_profile_sensor_data)
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
        self.alt_time = self.now()

    def on_imu(self, msg):
        self.yaw_rate, self.yaw_rate_time = float(msg.angular_velocity.z), self.now()

    def fresh_yaw_rate(self):
        return self.yaw_rate if self.now() - self.yaw_rate_time < 0.5 else None

    def depth_rate(self, t, z, window=1.0):
        """Vertical speed (m/s, up +) as the least-squares slope of the depth reading over
        the last `window` s, or None with too little history."""
        self.z_hist = [(tt, zz) for tt, zz in self.z_hist if t - tt <= window] + [(t, z)]
        if len(self.z_hist) < 4 or self.z_hist[-1][0] - self.z_hist[0][0] < 0.5:
            return None
        n = len(self.z_hist)
        mt = sum(tt for tt, _ in self.z_hist) / n
        mz = sum(zz for _, zz in self.z_hist) / n
        den = sum((tt - mt) ** 2 for tt, _ in self.z_hist)
        return sum((tt - mt) * (zz - mz) for tt, zz in self.z_hist) / den if den > 0 else None

    def fresh_alt(self):
        """DVL altitude if valid (> 0) and under 1 s old, else None."""
        if self.alt is None or self.alt <= 0.0 or self.now() - self.alt_time > 1.0:
            return None
        return self.alt

    def write_gate(self):
        if self.init is not None and self.a.log:
            d = os.path.dirname(os.path.abspath(self.a.log))
            with open(os.path.join(d, 'start_gate.json'), 'w') as f:
                json.dump(self.init.summary(), f, indent=1)
            with open(os.path.join(d, 'init.csv'), 'w') as f:
                f.write('t,z,yaw,heading_err,w,yaw_ccw\n')
                for row in self.init.trace:
                    f.write(','.join(f'{v:.4f}' for v in row) + '\n')
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
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.init is not None and self.init.phase != 'course':
            if self.init.phase == 'check':
                # floating: collect five DVL altitudes (up to 3 s) to decide the depth target
                if self.alt_wait_start is None:
                    self.alt_wait_start = t
                alt = self.fresh_alt()
                if alt is not None:
                    self.alt_seed.append((p.z, alt))
                if len(self.alt_seed) < 5 and t - self.alt_wait_start < 3.0:
                    self.publish(0.0, 0.0, 0.0)
                    return
                self.init.seed_floor(self.alt_seed)
            first = self.init.phase == 'check'
            if first:
                self.placement = (p.x, p.y)     # where the operator put it: on the start mark
            phase, w, r_ccw = self.init.feed(t, p.z, yaw, self.fresh_alt(),
                                             vz=self.depth_rate(t, p.z), yaw_rate=self.fresh_yaw_rate())
            if first and phase == 'settle':
                g = self.init
                if not g.floor_locked:
                    self.node.get_logger().warn('no valid DVL altitude after 3 s: target stays '
                                                f'{g.depth:+.2f} m until the DVL reports the floor')
                elif g.depth != g.depth_requested:
                    self.node.get_logger().warn(
                        f'{g.alt_start:.2f} m above the floor: depth target reduced from '
                        f'{g.depth_requested:+.2f} to {g.depth:+.2f} m to keep {g.min_alt:.2f} m clearance')
                else:
                    self.node.get_logger().info(f'{g.alt_start:.2f} m above the floor: depth target {g.depth:+.2f} m')
            if phase == 'abort':
                self.node.get_logger().error('NOT STARTING: ' + self.init.reason)
                self.finished = self.aborted = True
                return
            if phase == 'settle':
                self.publish(0.0, r_ccw, w)
                return
            # settled. The course is laid out from the PLACEMENT (the start mark, where the
            # estimator was just reset), x axis on the saved heading, so drift while diving
            # and turning does not move the course relative to the pool. --anchor settled
            # lays it out from the settled position instead.
            g = self.init
            g.alt_course_start = self.alt
            ox, oy = (p.x, p.y) if self.a.anchor == 'settled' else self.placement
            g.anchor = dict(mode=self.a.anchor, x=ox, y=oy,
                            drift_along=math.cos(g.heading) * (p.x - self.placement[0]) + math.sin(g.heading) * (p.y - self.placement[1]),
                            drift_side=-math.sin(g.heading) * (p.x - self.placement[0]) + math.cos(g.heading) * (p.y - self.placement[1]))
            self.trk.origin = (ox, oy, g.heading)
            o = self.PoseStamped()
            o.header.stamp = msg.header.stamp
            o.header.frame_id = msg.header.frame_id
            o.pose.position.x, o.pose.position.y, o.pose.position.z = ox, oy, p.z
            o.pose.orientation.z, o.pose.orientation.w = math.sin(g.heading / 2), math.cos(g.heading / 2)
            self.origin_pub.publish(o)
            self.node.get_logger().info(
                f'initialised after {g.settle_s:.1f} s: depth {g.z_course_start:+.3f} m, heading error '
                f'{math.degrees(wrap(g.heading - g.yaw_course_start)):+.1f} deg, dvl altitude '
                f'{self.alt if self.alt is not None else float("nan"):.2f} m; drifted {g.anchor["drift_along"]:+.2f} m '
                f'along / {g.anchor["drift_side"]:+.2f} m sideways from the mark; course laid out from the '
                f'{"mark" if self.a.anchor == "placement" else "settled position"}. Course starts.')
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
            if self.init is None:
                self.node.get_logger().info(
                    f'start: estimator heading {math.degrees(yaw):+.0f} deg '
                    f'-> course frame {"rotated to it" if not self.a.no_rotate else "NOT rotated"}')
        t -= self.t0
        if self.a.max_rate > 0:
            if t - self.last_act < 1.0 / self.a.max_rate:
                return
            self.last_act = t
        u, r, done = self.trk.update(t, p.x, p.y, yaw)
        if done or (self.a.duration and t > self.a.duration):
            self.finished = True
            why = 'course complete' if done else 'time limit'
            self.node.get_logger().info(f'{why} at t={t:.1f} s, laps={self.trk.laps}')
            if self.gate is not None or self.init is not None:
                s = (self.init or self.gate).summary()
                self.node.get_logger().info(
                    f"depth during course: mean {s.get('course_z_mean', float('nan')):+.3f} m, "
                    f"max deviation {s.get('course_z_max_dev', float('nan')):.3f} m")
            return
        w = 0.0
        if self.a.depth_target is not None:
            target = self.init.depth if self.init is not None else self.a.depth_target
            w = depth_cmd(target, p.z)
            if self.init is not None and self.a.depth_trim:
                # opt-in: the start-up's buoyancy trim held through the course. Off by default:
                # on 2026-11-12 it drove the course 0.13 m too deep (driving forward already
                # pulls this vehicle down) and onto the floor.
                w = max(-0.3, min(0.3, w + self.init.iz))
            alt = self.fresh_alt()
            if self.init is not None and alt is not None and alt < self.init.guard_alt:
                w = max(w, 0.10)        # floor guard during the course: never push down here
        self.publish(u, r, w)
        if self.gate is not None or self.init is not None:
            (self.init or self.gate).record(p.z, self.alt)
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
                raise SystemExit((3 if self.init is not None else 2) if self.aborted else 0)


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
    ap.add_argument('--init', action='store_true',
                    help='real vehicle: before the course, sink to --depth-target and turn to the saved heading, '
                         'both sensor-verified (depth within --settle-tol, heading within --heading-tol, for --init-hold s); abort (exit 3) if not settled within --init-timeout')
    ap.add_argument('--heading-ref', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                          'heading_ref.json'),
                    help='init: file written by save_heading.py (default: heading_ref.json next to this script)')
    # 6 deg / 1.0 s (was 3 deg / 1.5 s): on 2026-11-12 depth settled to +/-1 cm but a steady
    # push held the heading at +2..+6 deg (mean +3.7) for 30 s, so a 3 deg window never held.
    # The course axis is the saved heading exactly either way; the law removes the residual.
    ap.add_argument('--heading-tol', type=float, default=6.0, help='init: heading within this many degrees')
    ap.add_argument('--init-hold', type=float, default=1.0,
                    help='init: depth and heading must both stay in tolerance this long (s)')
    ap.add_argument('--init-timeout', type=float, default=45.0, help='init: give up (do not start) after this (s)')
    ap.add_argument('--min-altitude', type=float, default=0.30,
                    help='init: keep at least this much water under the DVL (m); caps the depth target and '
                         'stops any down push below it, during start-up and the course')
    ap.add_argument('--depth-trim', action='store_true',
                    help='init: carry the start-up depth integrator into the course (default: plain P depth law)')
    ap.add_argument('--anchor', choices=['placement', 'settled'], default='placement',
                    help='init: lay the course out from the start mark (placement, default) or from where '
                         'the start-up settled; the course heading is the saved heading either way')
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
                           f'depth={a.depth_target} dive_first={a.dive_first} init={a.init}')
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
