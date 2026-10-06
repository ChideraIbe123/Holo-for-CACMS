#!/usr/bin/env python3
"""Score course_runner runs. Works on sim and real directories alike.
For each <dir>/<tag>/: runner.csv (always), sim_gt.csv (sim only).
Reports laps, time, cross-track RMSE on the estimator's own track (what the real
vehicle can also give), and, where ground truth exists, RMSE of the true track
against the active path segment (truth is re-projected using the logged lap/wp).
Usage: score_runs.py <dir> [<dir> ...]"""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tracker import COURSES


def seg_err(p, a, b):
    a, b, p = np.array(a), np.array(b), np.array(p)
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1)
    return float(np.linalg.norm(p - (a + t * ab)))


def score(d):
    f = os.path.join(d, 'runner.csv')
    if not os.path.exists(f):
        return None
    r = np.genfromtxt(f, delimiter=',', names=True)
    if r.size < 20:
        return None
    course = 'line' if '_line' in os.path.basename(d) else 'rect'
    pts, _ = COURSES[course]
    out = dict(tag=os.path.basename(d), course=course, t=float(r['t'][-1]), laps=int(r['lap'][-1]),
               rmse_est=float(np.sqrt(np.mean(r['e'] ** 2))), max_est=float(np.abs(r['e']).max()),
               path=float(np.sum(np.hypot(np.diff(r['x']), np.diff(r['y'])))))
    # distance along the 0.3 m band on the line course: first time |e| stays under 0.05 m
    if course == 'line':
        inside = np.abs(r['e']) < 0.05
        k = next((i for i in range(len(inside)) if inside[i:].all()), None)
        out['settle_x'] = float(r['x'][k]) if k is not None else float('nan')
        # vehicle starts RIGHT of the line (e < 0); overshoot = how far it crosses to the left
        out['overshoot'] = float(max(0.0, r['e'].max()))
    g = os.path.join(d, 'sim_gt.csv')
    out['rmse_gt'] = float('nan')
    if os.path.exists(g):
        gt = np.genfromtxt(g, delimiter=',', names=True)
        if gt.size > 50:
            # runner.csv 'stamp' is the estimator message stamp = simulator time, the same
            # clock as sim_gt.csv. The sim spawns at yaw 0, so the course frame is the
            # truth frame shifted to the truth position at the runner's first stamp.
            j0 = int(np.argmin(np.abs(gt['t'] - r['stamp'][0])))
            gx, gy = gt['x'] - gt['x'][j0], gt['y'] - gt['y'][j0]
            sel = (gt['t'] >= r['stamp'][0]) & (gt['t'] <= r['stamp'][-1])
            wp = np.interp(gt['t'][sel], r['stamp'], r['wp']).round().astype(int)
            e = [seg_err((x, y), pts[w - 1], pts[w]) for x, y, w in zip(gx[sel], gy[sel], wp)]
            out['rmse_gt'] = float(np.sqrt(np.mean(np.square(e))))
    return out


def main():
    rows = []
    for root in sys.argv[1:]:
        for d in sorted(glob.glob(os.path.join(root, '*'))):
            if os.path.isdir(d):
                s = score(d)
                if s:
                    rows.append(s)
    print(f"{'run':<26}{'course':<7}{'laps':>5}{'time s':>8}{'path m':>8}{'RMSE est':>10}{'RMSE true':>10}{'max est':>9}{'settle x':>9}{'overshoot':>10}")
    for s in rows:
        print(f"{s['tag']:<26}{s['course']:<7}{s['laps']:>5}{s['t']:>8.1f}{s['path']:>8.2f}{s['rmse_est']:>10.3f}"
              f"{s['rmse_gt']:>10.3f}{s['max_est']:>9.3f}"
              f"{s.get('settle_x', float('nan')):>9.2f}{s.get('overshoot', float('nan')):>10.3f}")


if __name__ == '__main__':
    main()
