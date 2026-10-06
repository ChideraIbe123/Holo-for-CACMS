#!/usr/bin/env python3
"""Velocity setpoint -> BlueROV2 stick command, with arm / stop keys.

Controllers in this benchmark publish /cmd_vel (TwistStamped: forward m/s, up m/s,
yaw rate rad/s with positive = clockwise). The vehicle accepts MAVLink
MANUAL_CONTROL sticks (x, y, z, r in -1000..1000, z neutral at 500), the same way
bluerov_teleop and bluerov_auto drive it. This node converts one to the other.

IMPORTANT: bluerov_teleop and the Mission Control GUI both send sticks continuously
on udp 14550. Do not run either while this node is connected: they share the port
and would fight over the vehicle. This node therefore carries its own keys:

    a      arm                      SPACE or d   DISARM and latch a stop
    c      clear a latched stop     i k j l      jog fwd / back / yaw-left / yaw-right
                                                 (only between runs, low power)

Gains come from the 2026-09-17 single-axis tub runs, MANUAL mode, ~15.4 V:
  forward  500 -> 0.33 m/s, 1000 -> 0.66 m/s (estimator)  => 1500 stick per m/s
  yaw      500 -> 1.30 rad/s, 1000 -> 2.5 rad/s           => 400 stick per rad/s (CW +)
  throttle +250 -> ~0.15 m/s up, +500 -> 0.30             => 1650 stick per m/s
They are open-loop and valid only in MANUAL mode. In ALT_HOLD or STABILIZE the
autopilot reinterprets the yaw stick: re-measure --k-yaw before trusting it there.

Safety behaviour (none of it replaces a person at the pool):
  - stick limits and a slew-rate limit
  - no fresh /cmd_vel for --cmd-timeout s -> neutral sticks (still sent, so the
    autopilot's pilot-input failsafe stays quiet and the vehicle stays armed)
  - geofence in the course frame (origin and heading latched when a run starts);
    leaving it latches a stop until 'c' is pressed
  - --max-run s per run
  - neutral sticks, then DISARM, on exit

Modes:
  default      send to the vehicle over MAVLink (needs pymavlink)
  --dry-run    compute and publish /auto/cmd only
  --loopback   simulator test: turn the sticks back into a velocity setpoint on
               --loopback-topic, so limits, slew, timeout and fence run in closed loop

Usage (real):  python3 cmdvel_to_manual.py --depth-mode passthrough
Usage (sim):   python3 cmdvel_to_manual.py --loopback --in-topic /cmd_vel_ctrl --depth-mode passthrough
"""
import argparse
import math
import os
import select
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

NEUTRAL = (0.0, 500.0, 0.0)   # x, z, r
JOG = {'i': (200.0, 500.0, 0.0), 'k': (-200.0, 500.0, 0.0),
       'j': (0.0, 500.0, -200.0), 'l': (0.0, 500.0, 200.0)}
JOG_HOLD = 0.4                # s a jog key stays active after the last keypress


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class StickBridge:
    def __init__(self, node, a):
        self.node, self.a = node, a
        self.log = node.get_logger()
        self.cmd = None             # (u, w, r_cw)
        self.cmd_time = -1e9
        self.sticks = list(NEUTRAL)
        self.active = False         # a run is in progress
        self.run_start = 0.0
        self.origin = None          # (x0, y0, yaw0) latched at run start
        self.pose = None            # latest estimator pose
        self.tripped = None         # reason string once latched
        self.jog = None             # (sticks, time)
        self.sent = 0
        self.armed = None
        self.mode = None
        self.fence = [float(v) for v in a.fence.split(',')]
        self.master = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        if not (a.dry_run or a.loopback):
            from pymavlink import mavutil
            self.mavutil = mavutil
            self.master = mavutil.mavlink_connection(a.connect, source_system=a.sysid)
            self.log.info(f'waiting for vehicle heartbeat on {a.connect} ...')
            if self.master.wait_heartbeat(timeout=10) is None:
                raise SystemExit('no heartbeat from the vehicle: not starting')
            self.log.info(f'MAVLink connected (sys={self.master.target_system})')
            threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        self.auto_pub = node.create_publisher(TwistStamped, '/auto/cmd', 10)
        self.loop_pub = node.create_publisher(TwistStamped, a.loopback_topic, 10) if a.loopback else None
        node.create_subscription(TwistStamped, a.in_topic, self.on_cmd, 10)
        node.create_subscription(Odometry, '/deadreckon/odom', self.on_odom, 50)
        self.dt = 1.0 / a.rate
        node.create_timer(self.dt, self.tick)
        node.create_timer(0.5, self.poll_vehicle)
        mode = 'LOOPBACK (sim)' if a.loopback else 'DRY RUN' if a.dry_run else 'LIVE'
        self.log.info(f'{mode}: k_surge={a.k_surge:.0f} k_yaw={a.k_yaw:.0f} k_heave={a.k_heave:.0f} '
                      f'limits x<={a.max_x:.0f} r<={a.max_r:.0f} depth={a.depth_mode} fence={self.fence}')
        self._old_term = None
        if not a.no_keys and sys.stdin.isatty():
            threading.Thread(target=self._key_loop, daemon=True).start()
            self.log.info('keys: a=arm  SPACE/d=DISARM+stop  c=clear stop  i/k/j/l=jog')

    # ---------------------------------------------------------------- vehicle link
    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _heartbeat_loop(self):
        m = self.mavutil.mavlink
        while not self._stop.is_set():
            with self._lock:
                self.master.mav.heartbeat_send(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._stop.wait(0.5)

    def _arm_disarm(self, arm):
        if self.master is None:
            self.log.info(f'({"arm" if arm else "disarm"} ignored: no vehicle link in this mode)')
            return
        m = self.mavutil.mavlink
        with self._lock:
            for _ in range(1 if arm else 3):
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    m.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1 if arm else 0, 0, 0, 0, 0, 0, 0)
        self.log.warn('ARM sent' if arm else 'DISARM sent')

    def poll_vehicle(self):
        """Track armed state and flight mode from the autopilot's heartbeat."""
        if self.master is None:
            return
        m = self.mavutil.mavlink
        while True:
            with self._lock:
                msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == self.master.target_system \
                    and msg.get_srcComponent() == 1:
                armed = bool(msg.base_mode & m.MAV_MODE_FLAG_SAFETY_ARMED)
                inv = {v: k for k, v in (self.master.mode_mapping() or {}).items()}
                mode = inv.get(msg.custom_mode, str(msg.custom_mode))
                if (armed, mode) != (self.armed, self.mode):
                    self.armed, self.mode = armed, mode
                    self.log.warn(f'vehicle: {"ARMED" if armed else "disarmed"}, mode {mode}')
                    if mode != 'MANUAL':
                        self.log.warn('stick gains were measured in MANUAL mode; yaw gain is not valid here')

    # ---------------------------------------------------------------- keys
    def _key_loop(self):
        import termios
        import tty
        fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while not self._stop.is_set():
                r, _, _ = select.select([fd], [], [], 0.1)
                if not r:
                    continue
                k = os.read(fd, 1).decode('utf-8', errors='ignore')
                if k == 'a':
                    if self.tripped:
                        self.log.warn("stop is latched: press 'c' first")
                    else:
                        self._arm_disarm(True)
                elif k in (' ', 'd'):
                    self.tripped = self.tripped or 'operator stop'
                    self._arm_disarm(False)
                    self.log.error('OPERATOR STOP latched')
                elif k == 'c':
                    if self.now() - self.cmd_time < self.a.cmd_timeout:
                        self.log.warn('a controller is still publishing: stop it before clearing')
                    else:
                        self.tripped, self.active, self.origin = None, False, None
                        self.log.warn('stop cleared')
                elif k in JOG:
                    self.jog = (JOG[k], time.monotonic())
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, self._old_term)

    # ---------------------------------------------------------------- inputs
    def on_cmd(self, msg):
        self.cmd = (msg.twist.linear.x, msg.twist.linear.z, msg.twist.angular.z)
        self.cmd_time = self.now()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def course_xy(self):
        if self.origin is None or self.pose is None:
            return None
        x0, y0, yaw0 = self.origin
        dx, dy = self.pose[0] - x0, self.pose[1] - y0
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        return c * dx - s * dy, s * dx + c * dy

    # ---------------------------------------------------------------- loop
    def tick(self):
        a, now = self.a, self.now()
        fresh = (now - self.cmd_time) < a.cmd_timeout
        if fresh and not self.active and self.tripped is None:
            self.active, self.run_start = True, now
            self.origin = self.pose
            self.log.info('run started' + ('' if self.pose else ' (no estimator pose yet: fence inactive)'))
        if self.active and not fresh and self.tripped is None and now - self.cmd_time > a.cmd_timeout + a.release:
            self.active, self.origin = False, None
            self.log.info('run ended: holding neutral')
        if self.active and self.tripped is None:
            xy = self.course_xy()
            if xy is not None and not (self.fence[0] <= xy[0] <= self.fence[1]
                                       and self.fence[2] <= xy[1] <= self.fence[3]):
                self.tripped = f'geofence left at course x={xy[0]:+.2f} y={xy[1]:+.2f}'
            elif now - self.run_start > a.max_run:
                self.tripped = f'run longer than {a.max_run:.0f} s'
            if self.tripped:
                self.log.error(f"STOP LATCHED: {self.tripped}. Stop the controller, then press 'c'.")
        if self.tripped is not None:
            self.sticks = list(NEUTRAL)          # immediate, no slew
        else:
            if fresh:
                u, w, r_cw = self.cmd
                target = (clamp(a.k_surge * u, -a.max_x, a.max_x),
                          500.0 if a.depth_mode == 'neutral' else clamp(500.0 + a.k_heave * w,
                                                                        500.0 - a.max_z, 500.0 + a.max_z),
                          clamp(a.k_yaw * r_cw, -a.max_r, a.max_r))
            elif self.jog is not None and not self.active and time.monotonic() - self.jog[1] < JOG_HOLD:
                target = self.jog[0]
            else:
                target = NEUTRAL
            step = a.slew * self.dt
            for i in range(3):
                self.sticks[i] += clamp(target[i] - self.sticks[i], -step, step)
        x, z, r = int(round(self.sticks[0])), int(round(self.sticks[1])), int(round(self.sticks[2]))
        if self.master is not None:
            with self._lock:
                self.master.mav.manual_control_send(self.master.target_system, x, 0, z, r, 0)
        self.sent += 1
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = float(x), 0.0, float(z)
        m.twist.angular.z = float(r)
        self.auto_pub.publish(m)
        if self.loop_pub is not None and (self.active or self.tripped is not None):
            v = TwistStamped()
            v.header = m.header
            v.twist.linear.x = x / a.k_surge
            v.twist.linear.z = (z - 500.0) / a.k_heave
            v.twist.angular.z = r / a.k_yaw
            self.loop_pub.publish(v)

    def shutdown(self):
        self._stop.set()
        if self.master is not None:
            with self._lock:
                for _ in range(10):
                    self.master.mav.manual_control_send(self.master.target_system, 0, 0, 500, 0, 0)
                    time.sleep(0.03)
            self._arm_disarm(False)
        if self._old_term is not None:
            import termios
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old_term)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--connect', default='udpin:0.0.0.0:14550')
    ap.add_argument('--sysid', type=int, default=255)
    ap.add_argument('--in-topic', default='/cmd_vel')
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--k-surge', type=float, default=1500.0, help='stick per m/s')
    ap.add_argument('--k-yaw', type=float, default=400.0, help='stick per rad/s, clockwise positive')
    ap.add_argument('--k-heave', type=float, default=1650.0, help='stick per m/s upward')
    ap.add_argument('--max-x', type=float, default=600.0)
    ap.add_argument('--max-r', type=float, default=400.0)
    ap.add_argument('--max-z', type=float, default=200.0, help='max throttle deviation from 500')
    ap.add_argument('--slew', type=float, default=3000.0, help='stick units per second')
    ap.add_argument('--depth-mode', choices=['neutral', 'passthrough'], default='neutral',
                    help='neutral: always z=500. passthrough: use the controller\'s up-velocity')
    ap.add_argument('--cmd-timeout', type=float, default=0.5)
    ap.add_argument('--release', type=float, default=1.0, help='s without commands before a run is closed')
    ap.add_argument('--max-run', type=float, default=120.0)
    ap.add_argument('--fence', default='-0.4,2.4,-0.4,1.0',
                    help='xmin,xmax,ymin,ymax in the course frame, m (default suits the 2.0 x 0.6 rectangle)')
    ap.add_argument('--no-keys', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--loopback', action='store_true')
    ap.add_argument('--loopback-topic', default='/cmd_vel')
    a = ap.parse_args()
    if a.loopback and a.in_topic == a.loopback_topic:
        ap.error('--loopback needs --in-topic different from --loopback-topic')
    rclpy.init()
    node = rclpy.create_node('cmdvel_to_manual')
    b = StickBridge(node, a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        b.shutdown()
        node.get_logger().info(f'sent {b.sent} stick frames')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
