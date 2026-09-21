#!/usr/bin/env python3
"""Method-selection study: which statistic can CONFIDENTLY call a run
close / unsure / divergent from real, given run-to-run noise?

Frames fidelity as a detection problem. Positives = held-out REAL runs scored
against the remaining real floor (as real as it gets). Negatives = a GRADED
ladder: historical superseded sim campaigns (known-worse twins, in generation
order) and controlled corruptions of real runs (noise x1.25/1.5/2, increment
time-shuffle, channel swap, matched white noise). Candidate statistics compete
by ROC AUC per tier; the winner + two run-level thresholds (95% TPR "close",
95% TNR "divergent", between = "unsure") become the verdict rule, each hard
verdict carrying its measured error rate. Start-of-run transients are trimmed
by a knee detector (validated by its AUC delta, not assumed).

The FINAL sim (mc_runs_tub3) is never used for selection — no peeking.

Usage:
  method_selection_study.py --real DIR [DIR...] --hist DIR [DIR...]
                            [--splits 30] [--holdout 4] [--no-trim] [--final DIR]
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fidelity_scorecard import (CHANNELS, load, ks_stat, wasserstein1,
                                spectral_dist, acf_dist, window_features, mmd_rbf)

RNG = np.random.default_rng(11)


# ------------------------------------------------------------- trimming
def knee_trim(run, max_frac=0.25, min_s=2.0):
    """Trim the start-of-run transient: knee of cumulative increment variance
    (max distance-to-chord over the first max_frac of the run)."""
    out = {}
    imu = run["imu"]
    if len(imu) < 40:
        return run
    t = imu[:, 0]
    dur = t[-1] - t[0]
    d = np.abs(np.diff(imu[:, 1:7], axis=0))
    e = (d / (d.std(0) + 1e-12)).sum(1)
    c = np.cumsum(e)
    n = max(int(len(c) * max_frac), 5)
    seg = c[:n]
    x = np.linspace(0, 1, len(seg))
    y = (seg - seg[0]) / (seg[-1] - seg[0] + 1e-12)
    knee = int(np.argmax(np.abs(y - x)))
    t_cut = t[0] + max(min_s, min(t[knee + 1] - t[0], max_frac * dur))
    for k in ("imu", "dvl", "alt"):
        a = run[k]
        out[k] = a[a[:, 0] >= t_cut] if len(a) else a
    return out


# ------------------------------------------------------------- statistics
def _inc(run, key, col):
    a = run[key]
    return np.diff(a[:, col]) if len(a) >= 20 else None


def cv_dist(da, db):
    ca = np.std(da) / (np.mean(np.abs(da)) + 1e-12)
    cb = np.std(db) / (np.mean(np.abs(db)) + 1e-12)
    return abs(np.log((ca + 1e-9) / (cb + 1e-9)))


def iqr_dist(da, db):
    qa = np.subtract(*np.percentile(da, [75, 25]))
    qb = np.subtract(*np.percentile(db, [75, 25]))
    return abs(np.log((qa + 1e-9) / (qb + 1e-9)))


PAIRWISE = {"cv": cv_dist, "iqr": iqr_dist, "ks": ks_stat, "w1": wasserstein1,
            "acf": acf_dist}


def fid_frechet(X, Y):
    """Gaussian (FID-style) Fréchet distance between window-feature clouds."""
    mu1, mu2 = X.mean(0), Y.mean(0)
    c1 = np.cov(X.T) + 1e-9 * np.eye(X.shape[1])
    c2 = np.cov(Y.T) + 1e-9 * np.eye(Y.shape[1])
    diff = ((mu1 - mu2) ** 2).sum()
    # tr(C1 + C2 - 2 (C1 C2)^{1/2}) via eigvals of C1 C2
    ev = np.linalg.eigvals(c1 @ c2)
    covmean = np.sqrt(np.clip(ev.real, 0, None)).sum()
    return float(diff + np.trace(c1) + np.trace(c2) - 2 * covmean)


def run_scores(run, floor, methods, floor_feats):
    """Per-method score of ONE run vs the floor set: mean over channels of the
    median pairwise distance, normalized later by floor-of-floors."""
    out = {}
    for m in methods:
        if m in PAIRWISE:
            vals = []
            for name, key, col in CHANNELS:
                da = _inc(run, key, col)
                if da is None:
                    continue
                ds = [PAIRWISE[m](da, _inc(f, key, col)) for f in floor
                      if _inc(f, key, col) is not None]
                if m == "spec":
                    continue
                if ds:
                    vals.append(np.median(ds))
            out[m] = float(np.mean(vals)) if vals else np.nan
        elif m == "spec":
            vals = []
            for name, key, col in CHANNELS:
                a = run[key]
                if len(a) < 20:
                    continue
                ds = [spectral_dist(a[:, col], f[key][:, col]) for f in floor
                      if len(f[key]) >= 20]
                ds = [d for d in ds if d is not None]
                if ds:
                    vals.append(np.median(ds))
            out[m] = float(np.mean(vals)) if vals else np.nan
        elif m in ("mmd", "fid"):
            fa = window_features(run)
            if fa is None or len(fa) < 3:
                out[m] = np.nan
                continue
            fn = mmd_rbf if m == "mmd" else fid_frechet
            ds = [fn(fa, fb) for fb in floor_feats if fb is not None and len(fb) > 2]
            out[m] = float(np.median(ds)) if ds else np.nan
    return out


# ------------------------------------------------------------- corruptions
def corrupt(run, kind, rng):
    out = {k: run[k].copy() for k in ("imu", "dvl", "alt")}
    for k in ("imu", "dvl"):
        a = out[k]
        if len(a) < 20:
            continue
        ncol = a.shape[1]
        for c in range(1, min(ncol, 7)):
            x = a[:, c]
            if kind.startswith("scale"):
                f = float(kind[5:])
                extra = np.sqrt(max(f * f - 1, 0)) * np.std(np.diff(x)) / np.sqrt(2)
                a[:, c] = x + rng.normal(0, extra, len(x))
            elif kind == "shuffle":
                d = np.diff(x)
                rng.shuffle(d)
                a[1:, c] = x[0] + np.cumsum(d)
            elif kind == "white":
                a[:, c] = x.mean() + rng.normal(0, x.std(), len(x))
        if kind == "swap" and k == "imu" and ncol >= 4:
            a[:, [1, 2]] = a[:, [2, 1]]
    return out


def auc(pos, neg):
    """P(neg score > pos score) — rank AUC, higher = better detector."""
    pos, neg = np.asarray(pos), np.asarray(neg)
    pos, neg = pos[~np.isnan(pos)], neg[~np.isnan(neg)]
    if len(pos) == 0 or len(neg) == 0:
        return np.nan
    gt = (neg[:, None] > pos[None, :]).mean()
    eq = (neg[:, None] == pos[None, :]).mean()
    return float(gt + 0.5 * eq)


# ------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", nargs="+", required=True)
    ap.add_argument("--hist", nargs="+", default=[])
    ap.add_argument("--final", default=None)
    ap.add_argument("--splits", type=int, default=25)
    ap.add_argument("--holdout", type=int, default=4)
    ap.add_argument("--no-trim", action="store_true")
    a = ap.parse_args()

    def load_dir(d):
        return [load(f) for f in sorted(glob.glob(os.path.join(d, "*.npz")))]

    reals = [r for d in a.real for r in load_dir(d)]
    if not a.no_trim:
        reals = [knee_trim(r) for r in reals]
    print(f"{len(reals)} real runs (trim={'off' if a.no_trim else 'on'})")

    METHODS = ["cv", "iqr", "ks", "w1", "spec", "acf", "mmd", "fid"]  # composites appended after scoring
    TIERS = ["hist_late", "hist_early", "scale1.25", "scale1.5", "scale2",
             "shuffle", "swap", "white"]

    # historical sims split into late (close generations) and early (crude)
    hist_runs = {"hist_late": [], "hist_early": []}
    for d in a.hist:
        gen = "".join(ch for ch in os.path.basename(d.rstrip("/")) if ch.isdigit())
        tier = "hist_late" if gen and int(gen) >= 10 else "hist_early"
        runs = load_dir(d)
        if not a.no_trim:
            runs = [knee_trim(r) for r in runs]
        hist_runs[tier] += runs
    for k, v in hist_runs.items():
        if len(v) > 40:
            hist_runs[k] = [v[i] for i in RNG.choice(len(v), 40, replace=False)]
        print(f"{k}: {len(hist_runs[k])} runs")

    scores = {m: {"pos": [], **{t: [] for t in TIERS}} for m in METHODS}
    n = len(reals)
    for s in range(a.splits):
        idx = RNG.permutation(n)
        held, floor_i = idx[:a.holdout], idx[a.holdout:]
        floor = [reals[i] for i in floor_i]
        floor_feats = [window_features(f) for f in floor]
        for i in held:
            r = reals[i]
            sc = run_scores(r, floor, METHODS, floor_feats)
            for m in METHODS:
                scores[m]["pos"].append(sc[m])
            for kind in ("scale1.25", "scale1.5", "scale2", "shuffle", "swap", "white"):
                cr = corrupt(r, kind if not kind.startswith("scale") else kind, RNG)
                sc = run_scores(cr, floor, METHODS, floor_feats)
                for m in METHODS:
                    scores[m][kind].append(sc[m])
        if s == 0:      # historical sims scored once vs the full real floor
            ff = [window_features(f) for f in reals]
            for tier, runs in hist_runs.items():
                for r in runs:
                    sc = run_scores(r, reals, METHODS, ff)
                    for m in METHODS:
                        scores[m][tier].append(sc[m])
        print(f"  split {s + 1}/{a.splits}", flush=True)

    # composites: z-normalize each method by its positives, then combine —
    # w1 covers amplitude tiers, acf covers the nearly-right-twin/order tiers
    for combo_name, members in (("w1+acf", ("w1", "acf")),
                                ("w1+acf+mmd", ("w1", "acf", "mmd"))):
        mu = {m: np.nanmean(scores[m]["pos"]) for m in members}
        sd = {m: np.nanstd(scores[m]["pos"]) + 1e-12 for m in members}
        scores[combo_name] = {}
        for key in ["pos"] + TIERS:
            zs = np.array([[(v - mu[m]) / sd[m] for v in scores[m][key]]
                           for m in members])
            scores[combo_name][key] = list(np.nanmax(zs, axis=0))
        METHODS.append(combo_name)

    print(f"\n=== AUC per method x tier (trim={'off' if a.no_trim else 'on'}) ===")
    print("%-11s" % "method" + "".join("%10s" % t for t in TIERS))
    best, best_key = -1, None
    for m in METHODS:
        row = [auc(scores[m]["pos"], scores[m][t]) for t in TIERS]
        print("%-11s" % m + "".join("%10.3f" % v for v in row))
        mean_hard = np.nanmean(row[:5])      # graded tiers = the discriminating part
        if mean_hard > best:
            best, best_key = mean_hard, m
    print(f"\nWINNER on graded tiers: {best_key} (mean AUC {best:.3f})")

    # run-level thresholds from the winner: close = 95% TPR, divergent = 95% TNR
    pos = np.array([v for v in scores[best_key]["pos"] if not np.isnan(v)])
    hard_neg = np.array([v for t in ("hist_late", "scale1.25", "scale1.5")
                         for v in scores[best_key][t] if not np.isnan(v)])
    thr_close = float(np.quantile(pos, 0.95))
    thr_div = float(np.quantile(hard_neg, 0.05))
    fp_close = float((hard_neg <= thr_close).mean())
    fn_div = float((pos >= thr_div).mean())
    print(f"thr_close={thr_close:.4g} (real runs beyond this: 5%; hardest negatives "
          f"wrongly 'close': {100 * fp_close:.0f}%)")
    print(f"thr_div  ={thr_div:.4g} (hardest negatives below this: 5%; real runs "
          f"wrongly 'divergent': {100 * fn_div:.0f}%)")

    np.savez(os.path.expanduser("~/data/method_scores.npz"),
             **{f"{m}__{k}": np.array(v) for m in scores for k, v in scores[m].items()})

    if a.final:
        finals = load_dir(a.final)
        if not a.no_trim:
            finals = [knee_trim(r) for r in finals]
        ff = [window_features(f) for f in reals]
        base = [m for m in ("w1", "acf", "mmd") if m in best_key] or [best_key]
        mu = {m: np.nanmean(scores[m]["pos"]) for m in base}
        sd = {m: np.nanstd(scores[m]["pos"]) + 1e-12 for m in base}
        verdicts = {"close": 0, "unsure": 0, "divergent": 0}
        for r in finals:
            sc = run_scores(r, reals, base, ff)
            v = max((sc[m] - mu[m]) / sd[m] for m in base) if len(base) > 1 else sc[base[0]]
            verdicts["close" if v <= thr_close else
                     "divergent" if v >= thr_div else "unsure"] += 1
        print(f"\nFINAL SIM ({a.final}) run verdicts by {best_key}: {verdicts}")


if __name__ == "__main__":
    main()
