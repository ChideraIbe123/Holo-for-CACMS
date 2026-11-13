#!/usr/bin/env python3
"""Offline test of course_runner's DepthGate on a toy buoyant vehicle. No ROS.
Usage: python3 test_depth_gate.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from course_runner import DepthGate, depth_cmd  # noqa: E402

DT = 0.1          # estimator period, s
TAU = 0.8         # heave response time constant, s (vehicle + stick converter)
NOISE = 0.003     # depth sensor noise, m (tub floor 2.5 mm)


def run_vehicle(gate, z0=0.0, seconds=30.0, respond=True, seed=1):
    """First-order heave plant: commanded up-velocity w is reached with time constant TAU.
    Returns (phase, t_course_start, z_history)."""
    import random
    rng = random.Random(seed)
    z, vz, t = z0, 0.0, 0.0
    hist, t_course = [], None
    while t < seconds:
        meas = z + rng.gauss(0.0, NOISE)
        phase, w = gate.feed(t, meas)
        if phase == 'abort':
            return phase, None, hist
        if phase == 'course':
            if t_course is None:
                t_course = t
            w = depth_cmd(gate.target, meas)   # the runner keeps holding depth during the course
            gate.record(meas)
        if not respond:
            w = 0.0
        vz += (w - vz) * DT / TAU
        z += vz * DT
        hist.append((t, z, w))
        t += DT
    return gate.phase, t_course, hist


def check(name, cond):
    print(f'{"PASS" if cond else "FAIL"}  {name}')
    return cond


ok = True

# 1. Normal run: floating at the surface, reset done (z = 0), dives to -0.5 and settles.
g = DepthGate(-0.5)
phase, t_c, hist = run_vehicle(g)
s = g.summary()
ok &= check('normal dive reaches course phase', phase == 'course')
ok &= check(f'settled, not timed out (dive {s["dive_s"]:.1f} s)', s['settled'] and not s['timed_out'])
ok &= check(f'depth at course start within 5 cm ({s["z_course_start"]:+.3f})',
            abs(s['z_course_start'] - (-0.5)) < 0.05)
ok &= check('dive takes a few seconds, not instant', 2.0 < s['dive_s'] < 12.0)
ok &= check(f'depth held during course (max dev {s["course_z_max_dev"]:.3f} m)', s['course_z_max_dev'] < 0.06)

# 2. Not floating / reset not done: depth reads -0.35 at start -> refuse.
g = DepthGate(-0.5)
phase, _, _ = run_vehicle(g, z0=-0.35)
ok &= check('refuses to start when depth is not ~0 at start', phase == 'abort' and 'floating' in g.reason)

# 3. Slightly submerged at reset (bobbing, 6 cm) is accepted.
g = DepthGate(-0.5)
phase, _, _ = run_vehicle(g, z0=-0.06)
ok &= check('6 cm offset at start is accepted', phase == 'course')

# 4. Vertical thrusters not responding: times out, starts anyway, flagged.
g = DepthGate(-0.5, timeout=12.0)
phase, t_c, _ = run_vehicle(g, respond=False)
s = g.summary()
ok &= check('no heave response -> timed out but course still starts', phase == 'course' and s['timed_out'])
ok &= check(f'timeout honoured ({s["dive_s"]:.1f} s)', 11.9 <= s['dive_s'] <= 12.2)
ok &= check('summary marks it not settled', not s['settled'])

# 5. Simulator-style start: already at depth. Gate passes after just the hold time.
g = DepthGate(-0.5)
phase, t_c, _ = run_vehicle(g, z0=-0.5)
# z_start -0.5 fails the ~0 check by design: the sim never uses --dive-first.
ok &= check('absolute-depth start is rejected by the gate (sim must not use --dive-first)', phase == 'abort')

# 6. Transient crossing does not count: settle needs a full hold inside the band.
g = DepthGate(-0.5, settle_hold=1.0)
ts = [(0.0, 0.0), (0.5, -0.1), (1.0, -0.48), (1.5, -0.49), (1.8, -0.6), (2.5, -0.49), (3.0, -0.5), (3.6, -0.5)]
phases = [g.feed(t, z)[0] for t, z in ts]
ok &= check('brief pass through the band does not start the course', phases[3] == 'dive' and phases[4] == 'dive')
ok &= check('course starts after a continuous 1 s inside the band', phases[-1] == 'course')

print('\nALL CHECKS PASS' if ok else '\nSOME CHECKS FAILED')
sys.exit(0 if ok else 1)
