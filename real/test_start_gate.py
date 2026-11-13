#!/usr/bin/env python3
"""Offline test of course_runner's StartGate (--init) on a toy vehicle. No ROS.
Heave and yaw are first-order responses to the commanded rates, with sensor noise.
Usage: python3 test_start_gate.py"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from course_runner import StartGate, depth_cmd, wrap  # noqa: E402
from tracker import LAWS, CourseTracker  # noqa: E402

DT = 0.11            # estimator period, s (about 9 Hz on the vehicle)
TAU_Z = 0.8          # heave response, s
TAU_R = 0.4          # yaw-rate response, s
Z_NOISE = 0.003      # m
YAW_NOISE = math.radians(0.3)
REF = math.radians(-79.0)   # saved heading, the value seen on 2026-11-12


def run_vehicle(gate, z0=0.0, yaw0=0.0, seconds=40.0, heave=True, turn=True, yaw_bias=0.0, seed=1):
    """Returns (phase, t_course, final yaw). yaw_bias is a constant disturbance rate (rad/s)."""
    rng = random.Random(seed)
    z, vz, yaw, r, t = z0, 0.0, yaw0, 0.0, 0.0
    t_course = None
    while t < seconds:
        phase, w, r_cmd = gate.feed(t, z + rng.gauss(0, Z_NOISE), wrap(yaw + rng.gauss(0, YAW_NOISE)))
        if phase == 'abort':
            return phase, None, yaw
        if phase == 'course':
            if t_course is None:
                t_course = t
            w, r_cmd = depth_cmd(gate.depth, z), 0.0
        vz += ((w if heave else 0.0) - vz) * DT / TAU_Z
        r += ((r_cmd if turn else 0.0) - r) * DT / TAU_R
        z += vz * DT
        yaw = wrap(yaw + (r + yaw_bias) * DT)
        t += DT
        if t_course is not None and t - t_course > 2.0:
            break
    return gate.phase, t_course, yaw


def check(name, cond):
    print(f'{"PASS" if cond else "FAIL"}  {name}')
    return cond


ok = True

# 1. Floating, facing 100 deg away from the saved heading: dives and turns, then starts.
g = StartGate(-0.5, REF)
phase, t_c, _ = run_vehicle(g, yaw0=REF + math.radians(100))
s = g.summary()
ok &= check(f'settles and starts (after {s["settle_s"]:.1f} s)', phase == 'course' and s['settled'])
ok &= check(f'depth at start within 5 cm ({s["z_course_start"]:+.3f} m)', abs(s['z_course_start'] + 0.5) < 0.05)
ok &= check(f'heading at start within the 6 deg tolerance ({s["heading_err_course_start_deg"]:+.2f} deg)',
            abs(s['heading_err_course_start_deg']) < 6.0)

# 2. The turn takes the short way across +/-180 deg.
ref2 = math.radians(175.0)
g = StartGate(-0.5, ref2)
phase, _, _ = run_vehicle(g, yaw0=math.radians(-170.0))
first_r = g.trace[0][5]
# from -170 to +175 deg the short way is 15 deg clockwise (negative CCW rate), not 345 deg round;
# outside the tolerance the dead-band minimum (0.5 rad/s) applies
ok &= check(f'across +/-180: turns the short way, clockwise (first rate {first_r:+.2f} rad/s)',
            first_r < 0 and abs(first_r) <= 0.8 and phase == 'course')
ok &= check(f'across +/-180: settles quickly ({g.settle_s:.1f} s)', g.settle_s < 15.0)

# 3. Not floating at start: refuses, nothing commanded.
g = StartGate(-0.5, REF)
phase, _, _ = run_vehicle(g, z0=-0.3)
ok &= check('refuses when depth is not ~0 at start', phase == 'abort' and 'floating' in g.reason and not g.trace)

# 4. Yaw does not respond: does NOT start (unlike the old dive gate, which started anyway).
g = StartGate(-0.5, REF, timeout=30.0)
phase, t_c, _ = run_vehicle(g, yaw0=REF + math.radians(40), turn=False)
ok &= check('no yaw response -> aborts, course never starts', phase == 'abort' and t_c is None)
ok &= check('abort reason names the heading error', 'heading error' in (g.reason or ''))

# 5. Heave does not respond: does NOT start.
g = StartGate(-0.5, REF, timeout=30.0)
phase, t_c, _ = run_vehicle(g, yaw0=REF, heave=False)
ok &= check('no heave response -> aborts, course never starts', phase == 'abort' and t_c is None)

# 6. A steady disturbance (tether pulling the nose round) still settles inside 3 deg.
g = StartGate(-0.5, REF)
phase, _, _ = run_vehicle(g, yaw0=REF + math.radians(30), yaw_bias=0.02)
s = g.summary()
ok &= check(f'with 0.02 rad/s disturbance: settles ({s["heading_err_course_start_deg"]:+.2f} deg)',
            phase == 'course' and abs(s['heading_err_course_start_deg']) < 6.0)

# 7. Depth and heading must hold TOGETHER for the full hold time.
g = StartGate(-0.5, 0.0, hold=1.5)
seq = [(0.0, 0.0, 0.0), (0.5, -0.5, 0.0), (1.0, -0.5, 0.0), (1.4, -0.5, math.radians(8)),   # heading leaves
       (1.6, -0.5, 0.0), (2.6, -0.5, 0.0), (3.0, -0.5, 0.0), (3.2, -0.5, 0.0)]
phases = [g.feed(t, z, y)[0] for t, z, y in seq]
ok &= check('a heading excursion restarts the hold timer', phases[5] == 'settle')
ok &= check('starts after 1.5 s with both in tolerance', phases[-1] == 'course')

# 8. Course frame: with the origin set to the saved heading, a vehicle 2 deg off the saved
#    heading sees itself 2 deg off the course axis (the course is not rotated to the error).
trk = CourseTracker(LAWS['los'](), course='rect')
trk.origin = (3.0, -1.0, REF)
x, y, yaw = trk.to_course(3.0, -1.0, REF + math.radians(2.0))
ok &= check('course x axis is the saved heading, not the residual error',
            abs(x) < 1e-9 and abs(y) < 1e-9 and abs(math.degrees(yaw) - 2.0) < 1e-6)
x, y, _ = trk.to_course(3.0 + math.cos(REF), -1.0 + math.sin(REF), REF)
ok &= check('1 m along the saved heading is course x = +1 m', abs(x - 1.0) < 1e-9 and abs(y) < 1e-9)

print('\nALL CHECKS PASS' if ok else '\nSOME CHECKS FAILED')
sys.exit(0 if ok else 1)
