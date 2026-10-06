#!/usr/bin/env python3
"""Offline check of tracker.py on a unicycle with the real vehicle's measured lags.
No ROS, no simulator. Starts at the heading seen in the 2026-09-17 tub data (-69 deg)
and feeds poses with the real estimator's bursty timing, so the two real-pool
failure modes (unrotated course, burst timing) are exercised.
Usage: python3 test_tracker.py"""
import math
import random
import sys

from tracker import LAWS, CourseTracker, fuzzy_lab_raw

YAW0 = math.radians(-69.0)
X0, Y0 = 3.7, -1.2            # arbitrary estimator-frame start


def run(law_name, course, rotate=True, seed=0, tau_u=0.6, tau_r=0.25, tmax=150.0):
    rng = random.Random(seed)
    trk = CourseTracker(LAWS[law_name](), course=course, laps=2, rotate=rotate)
    x, y, yaw, u, r = X0, Y0, YAW0, 0.0, 0.0
    t, dt = 0.0, 0.005
    cmd_u = cmd_r = 0.0
    next_burst, errs, done, wall = 0.0, [], False, False
    while t < tmax and not done:
        if t >= next_burst:                       # ~9 Hz bursts of 3-5 messages
            for k in range(rng.randint(3, 5)):
                cmd_u, cmd_r, done = trk.update(t + 0.001 * k, x, y, yaw)
            next_burst = t + rng.uniform(0.09, 0.13)
            if trk.last_obs is not None:
                errs.append(abs(trk.last_obs.e))
            # TRUE position in the pool frame (pool long axis = starting heading).
            # Pool is 4 x 2 m with the course centred: walls 0.7 m either side of it.
            c, s_ = math.cos(-YAW0), math.sin(-YAW0)
            px = c * (x - X0) - s_ * (y - Y0)
            py = s_ * (x - X0) + c * (y - Y0)
            if py < -0.7 or py > 1.3 or px < -1.0 or px > 3.0:
                wall = True
                break
        u += dt / tau_u * (cmd_u - u)
        r += dt / tau_r * (cmd_r - r)
        yaw += r * dt
        x += u * math.cos(yaw) * dt
        y += u * math.sin(yaw) * dt
        t += dt
    rmse = math.sqrt(sum(v * v for v in errs) / max(len(errs), 1))
    return dict(done=done, wall=wall, t=t, laps=trk.laps, rmse=rmse, mx=max(errs) if errs else 0.0)


def main():
    ok = True
    # rule-base sanity: on the line, pointing along it -> ~0; right of line -> turn left (+)
    assert abs(fuzzy_lab_raw(1.5, 0.0, 0.0, 0.0)) < 1e-6
    assert fuzzy_lab_raw(1.5, 0.3, 0.0, 0.0) > 0 > fuzzy_lab_raw(1.5, -0.3, 0.0, 0.0)
    assert fuzzy_lab_raw(0.1, 0.0, 0.5, 0.0) > 0 > fuzzy_lab_raw(0.1, 0.0, -0.5, 0.0)
    print("fuzzy rule base: sign checks pass")
    print(f"\n{'law':<10}{'course':<7}{'finished':>9}{'time s':>8}{'laps':>5}{'RMSE m':>8}{'max m':>7}")
    for course in ("rect", "line"):
        for name in LAWS:
            m = run(name, course)
            good = m["done"] and not m["wall"]
            ok &= good
            print(f"{name:<10}{course:<7}{('yes' if good else 'WALL' if m['wall'] else 'no'):>9}"
                  f"{m['t']:>8.1f}{m['laps']:>5}{m['rmse']:>8.3f}{m['mx']:>7.3f}")
    m = run("p", "rect", rotate=False)
    print(f"\nwithout the heading rotation (how the frozen controllers behave): "
          f"{'hits the pool boundary at t=%.1f s' % m['t'] if m['wall'] else 'finished=%s' % m['done']}")
    ok &= m["wall"]
    print("\nALL CHECKS PASS" if ok else "\nFAILURES ABOVE")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
