#!/usr/bin/env python3
"""Choose the statistical threshold for 'acceptable noise' EMPIRICALLY.

A fidelity pass/fail criterion is only trustworthy if (a) genuine real data
passes it at the designed rate (calibration) and (b) data that is wrong in
known ways fails it (power). This study evaluates candidate criteria on both,
using only real bags + synthetic controls — no sim involved, so the threshold
is fixed BEFORE judging the twin (pre-registration of the ruler).

Candidates, per channel-metric:
  MWU        current: Mann-Whitney sim-real vs real-real distances, fail if p<0.05.
             (Known flaws: tests difference not equivalence; pairwise distances
             share bags -> dependence; power grows with floor size.)
  CONFORMAL  leave-out rank: margin = alpha-quantile of held-out-real median
             distances (each real bag scored against the rest). Pass if the
             candidate's median distance <= margin. Sample-size stable,
             bag-level (no pair dependence), equivalence-framed.
  MWU_FDR    MWU with Benjamini-Hochberg correction across the 31 metrics.

Controls (should FAIL):
  white      Gaussian white noise, per-channel std and length matched to a real
             bag (right amplitude, wrong shape/spectrum/order).
  shuffle    a real bag with each channel's INCREMENTS time-shuffled then
             re-integrated (exact amplitude distribution, destroyed ordering —
             tests order-sensitivity of the criterion).

Usage: threshold_study.py <real_npz_dir> [more dirs...] [--splits 40] [--alpha 0.95]
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fidelity_scorecard import CHANNELS, pair_distances, mannwhitney_p

RNG = np.random.default_rng(7)


def load_all(dirs):
    runs, names = [], []
    for d in dirs:
        for f in sorted(glob.glob(os.path.join(d, "*.npz"))):
            z = np.load(f)
            runs.append({k: z[k] for k in ("imu", "dvl", "alt")})
            names.append(os.path.basename(f))
    return runs, names


def metric_medians(dist):
    """{metric_key: median} from a pair_distances() output."""
    out = {}
    for name, _, _ in CHANNELS:
        for m in ("ks", "w1", "spec"):
            v = dist[name][m]
            if v:
                out[f"{name}.{m}"] = float(np.median(v))
    if dist["_mmd"]:
        out["window_mmd"] = float(np.median(dist["_mmd"]))
    return out


def mwu_fails(cand_runs, floor_runs, alpha_p=0.05):
    """Set of metric keys the CURRENT criterion fails."""
    rr = pair_distances(floor_runs, floor_runs, exclude_same=True)
    sr = pair_distances(cand_runs, floor_runs)
    fails = set()
    for name, _, _ in CHANNELS:
        for m in ("ks", "w1", "spec"):
            a, b = rr[name][m], sr[name][m]
            if a and b and mannwhitney_p(a, b) < alpha_p \
                    and np.median(b) > np.median(a):
                fails.add(f"{name}.{m}")
    if rr["_mmd"] and sr["_mmd"] and mannwhitney_p(rr["_mmd"], sr["_mmd"]) < alpha_p \
            and np.median(sr["_mmd"]) > np.median(rr["_mmd"]):
        fails.add("window_mmd")
    return fails


def conformal_margins(floor_runs, alpha=0.95):
    """Per-metric margin: alpha-quantile of each held-out real bag's median
    distance to the remaining floor bags. 'A genuine real run lands under
    this' — exceeding it means more foreign than (1-alpha) of real data."""
    per_metric = {}
    for i in range(len(floor_runs)):
        rest = floor_runs[:i] + floor_runs[i + 1:]
        med = metric_medians(pair_distances([floor_runs[i]], rest))
        for k, v in med.items():
            per_metric.setdefault(k, []).append(v)
    return {k: float(np.quantile(v, alpha)) for k, v in per_metric.items() if len(v) >= 5}


def conformal_fails(cand_runs, margins, floor_runs):
    med = metric_medians(pair_distances(cand_runs, floor_runs))
    return {k for k, v in med.items() if k in margins and v > margins[k]}


def bh_fdr(pvals, q=0.05):
    """Benjamini-Hochberg: returns set of rejected (failed) keys."""
    items = sorted(pvals.items(), key=lambda kv: kv[1])
    n = len(items)
    thresh = 0
    for i, (_, p) in enumerate(items, 1):
        if p <= q * i / n:
            thresh = i
    return {k for k, _ in items[:thresh]}


def mwu_fdr_fails(cand_runs, floor_runs, q=0.05):
    rr = pair_distances(floor_runs, floor_runs, exclude_same=True)
    sr = pair_distances(cand_runs, floor_runs)
    pv = {}
    for name, _, _ in CHANNELS:
        for m in ("ks", "w1", "spec"):
            a, b = rr[name][m], sr[name][m]
            if a and b and np.median(b) > np.median(a):
                pv[f"{name}.{m}"] = mannwhitney_p(a, b)
    if rr["_mmd"] and sr["_mmd"] and np.median(sr["_mmd"]) > np.median(rr["_mmd"]):
        pv["window_mmd"] = mannwhitney_p(rr["_mmd"], sr["_mmd"])
    return bh_fdr(pv)


# ---------------------------------------------------------------- controls
def white_control(run):
    out = {}
    for k in ("imu", "dvl", "alt"):
        a = run[k]
        if len(a) < 20:
            out[k] = a
            continue
        b = a.copy()
        for c in range(1, a.shape[1]):
            b[:, c] = a[:, c].mean() + RNG.normal(0, a[:, c].std(), len(a))
        out[k] = b
    return out


def shuffle_control(run):
    """Shuffle increments, re-integrate: same amplitude marginals, no ordering."""
    out = {}
    for k in ("imu", "dvl", "alt"):
        a = run[k]
        if len(a) < 20:
            out[k] = a
            continue
        b = a.copy()
        for c in range(1, a.shape[1]):
            d = np.diff(a[:, c])
            RNG.shuffle(d)
            b[1:, c] = a[0, c] + np.cumsum(d)
        out[k] = b
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--splits", type=int, default=40)
    ap.add_argument("--holdout", type=int, default=4)
    ap.add_argument("--alpha", type=float, default=0.95)
    args = ap.parse_args()

    runs, names = load_all(args.dirs)
    n = len(runs)
    print(f"{n} real bags loaded from {len(args.dirs)} dir(s)")
    crits = ("MWU", "MWU_FDR", "CONFORMAL")
    # counters: fraction of metrics failed, per criterion, per candidate type
    tally = {c: {"real": [], "white": [], "shuffle": []} for c in crits}

    for s in range(args.splits):
        idx = RNG.permutation(n)
        cand_i, floor_i = idx[:args.holdout], idx[args.holdout:]
        cand = [runs[i] for i in cand_i]
        floor = [runs[i] for i in floor_i]
        margins = conformal_margins(floor, args.alpha)
        n_metrics = max(len(margins), 1)
        controls = {"real": cand,
                    "white": [white_control(r) for r in cand],
                    "shuffle": [shuffle_control(r) for r in cand]}
        for label, cruns in controls.items():
            tally["MWU"][label].append(len(mwu_fails(cruns, floor)) / n_metrics)
            tally["MWU_FDR"][label].append(len(mwu_fdr_fails(cruns, floor)) / n_metrics)
            tally["CONFORMAL"][label].append(
                len(conformal_fails(cruns, margins, floor)) / n_metrics)
        if (s + 1) % 10 == 0:
            print(f"  split {s + 1}/{args.splits}")

    print(f"\nmean FRACTION OF METRICS FAILED (splits={args.splits}, "
          f"holdout={args.holdout}, alpha={args.alpha}):")
    print(f"{'criterion':<12}{'real (want ~%.02f)' % (1 - args.alpha):>20}"
          f"{'white (want high)':>20}{'shuffle (want high)':>21}")
    for c in crits:
        r = np.mean(tally[c]["real"])
        w = np.mean(tally[c]["white"])
        sh = np.mean(tally[c]["shuffle"])
        print(f"{c:<12}{r:>20.3f}{w:>20.3f}{sh:>21.3f}")
    print("\nInterpretation: a good criterion fails ~alpha-complement of real "
          "data (calibration)\nand a HIGH fraction of the controls (power). "
          "Order-blind metrics cannot fail 'shuffle'\nno matter the criterion — "
          "that gap is closed by adding ACF/spectral-family metrics.")


if __name__ == "__main__":
    main()
