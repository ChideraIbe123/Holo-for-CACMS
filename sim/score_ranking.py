"""Score and rank the benchmark controllers by cross-track error against the
commanded square course, using ground-truth trajectories. Produces a ranking
table and a figure. The seed-to-seed spread is the run-to-run "reference floor"
for the ranking: two controllers are distinguishable only if they differ by more
than that spread (the same logic the sim-vs-real ranking comparison will use).

Usage: python3 score_ranking.py <benchmark_dir>
"""
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SQUARE = [(0, 0), (3, 0), (3, 3), (0, 3), (0, 0)]


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
        return None
    a = np.genfromtxt(f, delimiter=",", names=True)
    xy = np.stack([a["x"] - a["x"][0], a["y"] - a["y"][0]], 1)
    return xy


def main():
    root = sys.argv[1]
    runs = {}
    for d in sorted(glob.glob(os.path.join(root, "*_s*"))):
        if not os.path.isdir(d):
            continue
        base = os.path.basename(d)
        name = base.rsplit("_s", 1)[0]
        xy = load_gt(d)
        if xy is None or len(xy) < 50:
            continue
        cte = crosstrack(xy)
        runs.setdefault(name, []).append((base, xy, float(cte.mean()), float(cte.max())))

    # rank by mean cross-track error, averaged across seeds
    stats = []
    for name, rs in runs.items():
        means = [r[2] for r in rs]
        stats.append((name, np.mean(means), np.std(means), np.mean([r[3] for r in rs]), len(rs)))
    stats.sort(key=lambda s: s[1])

    print(f"\n{'rank':<5}{'controller':<14}{'mean CTE (m)':>14}{'±seed':>8}{'max CTE':>9}{'runs':>6}")
    for i, (name, m, sd, mx, n) in enumerate(stats, 1):
        print(f"{i:<5}{name:<14}{m:>14.3f}{sd:>8.3f}{mx:>9.3f}{n:>6}")

    # distinguishability: is each adjacent pair separated by more than the seed spread?
    print("\nranking reference floor (seed-to-seed spread):")
    for i in range(len(stats) - 1):
        gap = stats[i + 1][1] - stats[i][1]
        floor = stats[i][2] + stats[i + 1][2] + 1e-9
        print(f"  {stats[i][0]} < {stats[i+1][0]}: gap {gap:.3f} m vs spread {floor:.3f} m "
              f"-> {'DISTINGUISHABLE' if gap > floor else 'within noise'}")

    # figure: trajectories + ranking bars
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))
    sq = np.array(SQUARE)
    ax1.plot(sq[:, 0], sq[:, 1], ":", color="#888", lw=2, label="commanded square")
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(stats)))
    order = {name: c for (name, *_), c in zip(stats, colors)}
    for name, rs in runs.items():
        xy = rs[0][1]
        ax1.plot(xy[:, 0], xy[:, 1], lw=1.6, color=order[name], label=name)
    ax1.set_aspect("equal"); ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
    ax1.legend(fontsize=8); ax1.set_title("Controllers driving the sim vehicle\n(ground truth, dead-reckoning feedback)")
    names = [s[0] for s in stats]; means = [s[1] for s in stats]; sds = [s[2] for s in stats]
    ax2.barh(range(len(names)), means, xerr=sds, color=[order[n] for n in names], capsize=4)
    ax2.set_yticks(range(len(names))); ax2.set_yticklabels(names); ax2.invert_yaxis()
    ax2.set_xlabel("mean cross-track error (m)")
    ax2.set_title("Sim ranking by tracking accuracy\n(error bars = run-to-run spread)")
    ax2.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    out = os.path.join(root, "ranking.png")
    fig.savefig(out, dpi=160)
    print(f"\nfigure -> {out}")


if __name__ == "__main__":
    main()
