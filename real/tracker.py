#!/usr/bin/env python3
"""Path-following laws and course bookkeeping, with no ROS dependency.

Used by course_runner.py on the real vehicle and in the simulator, and by
test_tracker.py offline. Everything here works in the COURSE FRAME: origin at the
vehicle's pose when the run starts, +x along its starting heading, +y to its left.
The estimator's own frame is compass-aligned (2026-09-17 tub data: start yaw -69 deg),
so the course must be rotated by the starting heading or the first waypoint ends up
in a wall.

Conventions
  yaw, yaw-rate : counter-clockwise positive (ROS).  Laws return CCW-positive rad/s.
  cross-track e : positive when the vehicle is LEFT of the path direction.
"""
import math
from dataclasses import dataclass

# Registered benchmark course (sim/waypoint_controller.py WAYPOINTS, same order).
RECT = [(0.0, 0.0), (2.0, 0.0), (2.0, 0.6), (0.0, 0.6), (0.0, 0.0)]
# Offset-start line: vehicle starts 0.3 m to the right of a 2.4 m line and must
# converge onto it. Single pass. Measures settling, which the rectangle cannot.
LINE = [(0.0, 0.3), (2.4, 0.3)]
COURSES = {"rect": (RECT, True), "line": (LINE, False)}   # (points, closed loop?)


def make_course(course, line_length=2.4, line_offset=0.3):
    """Course points and whether it loops. The line can be resized to the pipe on hand."""
    if course == "line":
        return [(0.0, line_offset), (line_length, line_offset)], False
    return COURSES[course]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Obs:
    t: float            # s since run start
    dt: float           # s since previous distinct update (0 on the first)
    x: float            # course frame, m
    y: float
    yaw: float          # course frame, rad
    e: float            # cross-track, m, + = left of path
    s: float            # distance along the active segment, m
    seg_len: float
    chi: float          # path tangent angle, rad
    dist_target: float  # to the active waypoint, m
    bearing: float      # angle from vehicle to the active waypoint, rad
    ax: float           # active segment start
    ay: float
    bx: float           # active segment end (the target waypoint)
    by: float
    cruise: float
    new_segment: bool   # True on the first update of each segment


# ------------------------------------------------------------------ laws
class LawP:
    """Heading to the waypoint, proportional. Chidera's registered p_baseline."""
    name = "p"

    def __init__(self, kp=1.4):
        self.kp = kp

    def steer(self, o):
        err = wrap(o.bearing - o.yaw)
        return self.kp * err, err


class LawPursuit:
    """Registered 'pursuit': aim a point 0.4 of the leg ahead of the projection."""
    name = "pursuit"

    def __init__(self, kp=1.4, frac=0.4):
        self.kp, self.frac = kp, frac

    def steer(self, o):
        t = min(max(o.s / max(o.seg_len, 1e-9), 0.0), 1.0)
        u = min(t + self.frac, 1.0)
        lx, ly = o.ax + u * (o.bx - o.ax), o.ay + u * (o.by - o.ay)
        err = wrap(math.atan2(ly - o.y, lx - o.x) - o.yaw)
        return self.kp * err, err


class LawLOS:
    """Lookahead line-of-sight guidance (Fossen 2021 ch. 12) + P heading loop.
    chi_d = chi - atan(e / delta). delta 0.4 m: the 0.6 m legs rule out the usual
    1-2 vehicle lengths (0.46-0.9 m)."""
    name = "los"

    def __init__(self, kpsi=1.4, delta=0.4):
        self.kpsi, self.delta = kpsi, delta

    def steer(self, o):
        err = wrap(o.chi - math.atan(o.e / self.delta) - o.yaw)
        return self.kpsi * err, err


class LawILOS:
    """Integral LOS (Borhaug, Pavlov & Pettersen, CDC 2008).
    chi_d = chi - atan((e + sigma*z)/delta),  dz/dt = delta*e / (delta^2 + (e+sigma*z)^2).
    The integral is reset on each new leg: legs point in different directions and the
    biases expected here (tether pull, yaw-rate offset) are not earth-fixed."""
    name = "ilos"

    def __init__(self, kpsi=1.4, delta=0.4, sigma=0.2, zmax=1.0):
        self.kpsi, self.delta, self.sigma, self.zmax = kpsi, delta, sigma, zmax
        self.z = 0.0

    def steer(self, o):
        if o.new_segment:
            self.z = 0.0
        q = o.e + self.sigma * self.z
        self.z += min(o.dt, 0.3) * self.delta * o.e / (self.delta ** 2 + q ** 2)
        self.z = max(-self.zmax, min(self.zmax, self.z))
        q = o.e + self.sigma * self.z
        err = wrap(o.chi - math.atan(q / self.delta) - o.yaw)
        return self.kpsi * err, err


_BIG = 1e30
# AUVSL/rgator_motion_controllers include/.../config.h (ANFIS-tuned rule base,
# "Max's tuning"), copied verbatim. Inputs: distance to target, distance to line,
# theta far, theta near. 17 trapezoids, 30 rules, 9 output levels.
_MFS = [
    (_BIG, _BIG, 0.17690551280975342, 1.4224464893341064),
    (0.17690551280975342, 1.4224464893341064, -_BIG, -_BIG),
    (_BIG, _BIG, -0.26394927501678467, -0.16424866020679474),
    (-0.26394927501678467, -0.16424866020679474, -0.1637570858001709, -0.03788042813539505),
    (-0.1637570858001709, -0.03788042813539505, 0.03788042813539505, 0.1637570858001709),
    (0.03788042813539505, 0.1637570858001709, 0.16424866020679474, 0.26394927501678467),
    (0.16424866020679474, 0.26394927501678467, -_BIG, -_BIG),
    (-3.1415927410125732, -3.1415927410125732, -1.0128507614135742, -0.6697894930839539),
    (-1.0128507614135742, -0.6697894930839539, -0.49034059047698975, -0.1442113220691681),
    (-0.49034059047698975, -0.1442113220691681, 0.1442113220691681, 0.49034059047698975),
    (0.1442113220691681, 0.49034059047698975, 0.6697894930839539, 1.0128507614135742),
    (0.6697894930839539, 1.0128507614135742, 3.1415927410125732, 3.1415927410125732),
    (-3.1415927410125732, -3.1415927410125732, -0.8355019688606262, -0.5374418497085571),
    (-0.8355019688606262, -0.5374418497085571, -0.4873863756656647, -0.03811681643128395),
    (-0.4873863756656647, -0.03811681643128395, 0.03811681643128395, 0.4873863756656647),
    (0.03811681643128395, 0.4873863756656647, 0.5374418497085571, 0.8355019688606262),
    (0.5374418497085571, 0.8355019688606262, 3.1415927410125732, 3.1415927410125732),
]
_MF_INPUT = [0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3]
_IN_INDEX = [0, 2, 7, 12]
_RULES = [
    (1, 0, 1, 0, 1), (1, 0, 2, 0, 3), (1, 0, 3, 0, 5), (1, 0, 4, 0, 7), (1, 0, 5, 0, 9),
    (2, 1, 0, 1, 1), (2, 1, 0, 2, 2), (2, 1, 0, 3, 3), (2, 1, 0, 4, 4), (2, 1, 0, 5, 7),
    (2, 2, 0, 1, 1), (2, 2, 0, 2, 3), (2, 2, 0, 3, 4), (2, 2, 0, 4, 5), (2, 2, 0, 5, 8),
    (2, 3, 0, 1, 2), (2, 3, 0, 2, 4), (2, 3, 0, 3, 5), (2, 3, 0, 4, 6), (2, 3, 0, 5, 8),
    (2, 4, 0, 1, 2), (2, 4, 0, 2, 4), (2, 4, 0, 3, 5), (2, 4, 0, 4, 6), (2, 4, 0, 5, 8),
    (2, 5, 0, 1, 3), (2, 5, 0, 2, 6), (2, 5, 0, 3, 7), (2, 5, 0, 4, 8), (2, 5, 0, 5, 9),
]
_ANG = [-4.807295322418213, -3.5405654907226562, -2.344144821166992, -1.205312728881836,
        0.0, 1.205312728881836, 2.344144821166992, 3.5405654907226562, 4.807295322418213]


def _trapmf(x, mf):
    left = 1.0 if abs(mf[1] - mf[0]) < 1e-8 else (x - mf[0]) / (mf[1] - mf[0])
    right = 1.0 if abs(mf[3] - mf[2]) < 1e-8 else (mf[3] - x) / (mf[3] - mf[2])
    return max(0.0, min(left, right, 1.0))


def fuzzy_lab_raw(dist_target, dist_line, theta_far, theta_near):
    """Verbatim port of Fuzzy::fuzzyControl (product AND, weighted average)."""
    inp = (dist_target, dist_line, theta_far, theta_near)
    mf = [_trapmf(inp[_MF_INPUT[i]], _MFS[i]) for i in range(17)]
    num = den = 0.0
    for rule in _RULES:
        w = 1.0
        for j in range(4):
            k = rule[j]
            if k:
                w *= mf[_IN_INDEX[j] + k - 1]
        den += w
        num += _ANG[rule[4] - 1] * w
    return num / den if den > 0 else 0.0


class LawFuzzyLab:
    """The lab's ANFIS-tuned fuzzy path tracker, rule base unchanged.
    Input definitions follow AUVSL/auvsl_control RL_control.py fuzzy_error():
      dist_line  + when the vehicle is RIGHT of the path (so it is -e here)
      theta_far  bearing to a point on the line just ahead (0.9*projection + 0.1*target)
      theta_near path tangent minus heading
    Its output was tuned for a ground vehicle and peaks at 4.8 rad/s, so it is scaled
    by out_scale (default: peak maps to 0.8 rad/s). That scale is the one change."""
    name = "fuzzy_lab"

    def __init__(self, out_scale=0.8 / 4.807295322418213):
        self.out_scale = out_scale

    def steer(self, o):
        seg = (o.bx - o.ax, o.by - o.ay)
        t = min(max(o.s / max(o.seg_len, 1e-9), 0.0), 1.0)
        px, py = o.ax + t * seg[0], o.ay + t * seg[1]
        fx, fy = 0.9 * px + 0.1 * o.bx, 0.9 * py + 0.1 * o.by
        if math.hypot(fx - o.x, fy - o.y) < 1e-3:
            theta_far = wrap(o.chi - o.yaw)
        else:
            theta_far = wrap(math.atan2(fy - o.y, fx - o.x) - o.yaw)
        theta_near = wrap(o.chi - o.yaw)
        r = fuzzy_lab_raw(o.dist_target, -o.e, theta_far, theta_near)
        return self.out_scale * r, theta_near


LAWS = {c.name: c for c in (LawP, LawPursuit, LawLOS, LawILOS, LawFuzzyLab)}


# ------------------------------------------------------------------ tracker
class CourseTracker:
    """Feeds estimator poses in, gets (surge m/s, yaw-rate CCW rad/s, done) out."""

    def __init__(self, law, course="rect", cruise=0.3, reach=0.3, yaw_cap=0.8,
                 laps=2, rotate=True, slow_angle=1.0, slow_speed=0.1,
                 line_length=2.4, line_offset=0.3):
        self.law = law
        self.pts, self.closed = make_course(course, line_length, line_offset)
        self.cruise, self.reach, self.yaw_cap = cruise, reach, yaw_cap
        self.laps_goal, self.rotate = laps, rotate
        self.slow_angle, self.slow_speed = slow_angle, slow_speed
        self.origin = None          # (x0, y0, yaw0) in the estimator frame
        self.i = 1                  # index of the active target waypoint
        self.laps = 0
        self.done = False
        self._last = None           # (t, x, y, yaw) of the previous distinct update
        self._new_seg = True
        self.last_obs = None

    def to_course(self, x, y, yaw):
        if self.origin is None:
            self.origin = (x, y, yaw if self.rotate else 0.0)
        x0, y0, y0aw = self.origin
        dx, dy = x - x0, y - y0
        c, s = math.cos(-y0aw), math.sin(-y0aw)
        return c * dx - s * dy, s * dx + c * dy, wrap(yaw - y0aw)

    def _advance(self):
        self.i += 1
        self._new_seg = True
        if self.i >= len(self.pts):
            if self.closed:
                self.laps += 1
                self.i = 1
                if self.laps_goal and self.laps >= self.laps_goal:
                    self.done = True
            else:
                self.i = len(self.pts) - 1
                self.done = True

    def update(self, t, x_est, y_est, yaw_est):
        x, y, yaw = self.to_course(x_est, y_est, yaw_est)
        if self.done:
            return 0.0, 0.0, True
        for _ in range(len(self.pts)):          # may skip a waypoint already passed
            ax, ay = self.pts[self.i - 1]
            bx, by = self.pts[self.i]
            L = math.hypot(bx - ax, by - ay)
            s = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / L
            if math.hypot(bx - x, by - y) < self.reach or s > L:
                self._advance()
                if self.done:
                    return 0.0, 0.0, True
                continue
            break
        e = ((bx - ax) * (y - ay) - (by - ay) * (x - ax)) / L
        dt = 0.0
        if self._last is not None:
            if (x, y, yaw) == self._last[1:]:
                dt = 0.0                        # duplicate pose inside a burst
            else:
                dt = max(0.0, t - self._last[0])
        o = Obs(t=t, dt=dt, x=x, y=y, yaw=yaw, e=e, s=s, seg_len=L,
                chi=math.atan2(by - ay, bx - ax), dist_target=math.hypot(bx - x, by - y),
                bearing=math.atan2(by - y, bx - x), ax=ax, ay=ay, bx=bx, by=by,
                cruise=self.cruise, new_segment=self._new_seg)
        self._new_seg = False
        if dt > 0.0 or self._last is None:
            self._last = (t, x, y, yaw)
        r, steer_err = self.law.steer(o)
        r = max(-self.yaw_cap, min(self.yaw_cap, r))
        u = self.cruise if abs(steer_err) < self.slow_angle else self.slow_speed
        self.last_obs = o
        return u, r, False
