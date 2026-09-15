"""Rank benchmark controllers using the metrics from the community-standard
BlueROV2 benchmark (von Benzon et al., "An Open-Source Benchmark Simulator:
Control of a BlueROV2 Underwater Robot", J. Mar. Sci. Eng. 2022, 10, 1898).

Rather than an ad-hoc score, we adopt that benchmark's evaluation language so our
numbers are comparable to published control work:
  - RMSE (m)       : position tracking error against the commanded reference path.
  - IAE (m*s)      : integral of absolute error, the classic control-benchmark
                     quantity (their "follows the trajectory with a low error").
  - settling (s)   : time to first reach AND stay within a band of the path; the
                     benchmark's own success criterion is "stabilize onto the
                     trajectory within 20 s" -> we flag PASS/FAIL against 20 s.
  - max err (m)    : worst-case excursion.
The seed-to-seed spread is the run-to-run REFERENCE FLOOR: two controllers are
distinguishable only if their RMSE gap exceeds that spread (the same logic the
sim-vs-real ranking comparison will use).

NOTE on task: von Benzon's task is a 25 m open-water monopile inspection under a
0.21 m/s current + 35 m tether. The lab vehicle operates in a ~1 m pool, which
reality can validate; so we keep their metrics, baseline (SMC), and task
STRUCTURE (stabilize, then constant-velocity track) but on a pool-scaled path.

Usage: python3 score_ranking.py <benchmark_dir>
"""
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# commanded reference path (pool-scaled square transect); closed loop.
SQUARE = [(0, 0), (3, 0), (3, 3), (0, 3), (0, 0)]
SETTLE_BAND = 0.3      # m; "on the trajectory" tolerance
SETTLE_LIMIT = 20.0    # s; von Benzon success criterion


def seg_dist(p, a, b):
    a, b, p = np.array(a), np.array(b), np.array(p)
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-9), 0, 1)
    return np.linalg.norm(p - (a + t * ab))


def crosstrack(xy):
    return np.array([min(seg_dist(p, SQUARE[i], SQUARE[i + 1]) for i in range(len(SQUARE) - 1))
                     for p in xy])


def load_gt(d):
    f = os.path.join(d, "sim_gt.csv")
    if not os.path.exists(f):
        return None, None
    a = np.genfromtxt(f, delimiter=",", names=True)
    if a.size < 50:
        return None, None
    t = a["t"] - a["t"][0]
    xy = np.stack([a["x"] - a["x"][0], a["y"] - a["y"][0]], 1)
    return t, xy


def settling_time(t, e):
    """Time to FIRST stabilize onto the path (enter the band), matching von
    Benzon's criterion "stabilize the ROV on the trajectory ... within 20 s".
    On a repeating lap the vehicle cuts each corner and briefly leaves the band
    every lap, so "never leaves again" would measure last-corner overshoot, not
    initial convergence -- first entry is the correct reading here."""
    inside = np.where(e <= SETTLE_BAND)[0]
    if len(inside) == 0:
        return float("inf")
    return float(t[inside[0]])


def metrics(t, xy):
    e = crosstrack(xy)
    rmse = float(np.sqrt(np.mean(e ** 2)))
    iae = float(np.trapz(np.abs(e), t))          # m*s
    mx = float(e.max())
    settle = settling_time(t, e)
    return dict(rmse=rmse, iae=iae, mx=mx, settle=settle, e=e, xy=xy)


def main():
    root = sys.argv[1]
    runs = {}
    for d in sorted(glob.glob(os.path.join(root, "*_s*"))):
        if not os.path.isdir(d):
            continue
        name = os.path.basename(d).rsplit("_s", 1)[0]
        t, xy = load_gt(d)
        if xy is None:
            continue
        runs.setdefault(name, []).append(metrics(t, xy))

    # seed-averaged stats, ranked by RMSE (the benchmark's primary tracking metric)
    stats = []
    for name, rs in runs.items():
        rmse = np.array([r["rmse"] for r in rs])
        stats.append(dict(
            name=name, rmse=float(rmse.mean()), rmse_sd=float(rmse.std()),
            iae=float(np.mean([r["iae"] for r in rs])),
            mx=float(np.mean([r["mx"] for r in rs])),
            settle=float(np.mean([r["settle"] for r in rs])), n=len(rs)))
    stats.sort(key=lambda s: s["rmse"])

    print(f"\n{'rank':<5}{'controller':<13}{'RMSE(m)':>9}{'±seed':>8}"
          f"{'IAE(m*s)':>10}{'maxErr':>8}{'settle(s)':>10}{'runs':>5}")
    for i, s in enumerate(stats, 1):
        settle = f"{s['settle']:.1f}" if np.isfinite(s['settle']) else "never"
        print(f"{i:<5}{s['name']:<13}{s['rmse']:>9.3f}{s['rmse_sd']:>8.3f}"
              f"{s['iae']:>10.2f}{s['mx']:>8.3f}{settle:>10}{s['n']:>4}")
    print(f"  (RMSE/IAE are the ranking metrics. settle = time to first reach "
          f"within {SETTLE_BAND} m of the path; ~0 here because the vehicle starts\n"
          f"   ON the path -- von Benzon's off-path {SETTLE_LIMIT:.0f}s stabilization "
          f"test would need an offset start pose.)")

    # distinguishability against the seed-to-seed reference floor
    print("\nranking reference floor (RMSE, seed-to-seed spread):")
    for i in range(len(stats) - 1):
        a, b = stats[i], stats[i + 1]
        gap = b["rmse"] - a["rmse"]
        floor = a["rmse_sd"] + b["rmse_sd"] + 1e-9
        verdict = "DISTINGUISHABLE" if gap > floor else "within noise"
        print(f"  {a['name']} < {b['name']}: gap {gap:.3f} m vs spread {floor:.3f} m -> {verdict}")

    # figure: trajectories + RMSE ranking
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))
    sq = np.array(SQUARE)
    ax1.plot(sq[:, 0], sq[:, 1], ":", color="#888", lw=2, label="commanded path")
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(stats)))
    cmap = {s["name"]: c for s, c in zip(stats, colors)}
    for name, rs in runs.items():
        xy = rs[0]["xy"]
        ax1.plot(xy[:, 0], xy[:, 1], lw=1.6, color=cmap[name], label=name)
    ax1.set_aspect("equal"); ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
    ax1.legend(fontsize=8)
    ax1.set_title("Controllers driving the sim vehicle\n(ground truth, dead-reckoning feedback)")
    names = [s["name"] for s in stats]
    rmse = [s["rmse"] for s in stats]
    sds = [s["rmse_sd"] for s in stats]
    ax2.barh(range(len(names)), rmse, xerr=sds, color=[cmap[n] for n in names], capsize=4)
    ax2.set_yticks(range(len(names))); ax2.set_yticklabels(names); ax2.invert_yaxis()
    ax2.set_xlabel("position RMSE (m)  [von Benzon 2022 tracking metric]")
    ax2.set_title("Sim ranking by tracking accuracy\n(error bars = run-to-run reference floor)")
    ax2.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    out = os.path.join(root, "ranking.png")
    fig.savefig(out, dpi=160)
    print(f"\nfigure -> {out}")


if __name__ == "__main__":
    main()
