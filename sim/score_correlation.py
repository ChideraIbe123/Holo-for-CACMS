#!/usr/bin/env python3
"""Sim-vs-real controller-ranking correlation — the paper's headline number.

The claim under test: the sim ranks controllers in the SAME ORDER reality does,
within run-to-run noise. Inputs are two benchmark result directories (one from
sim, one from the real-vehicle session), each in the run_benchmark.sh layout:
<dir>/<controller>_s<run>/sim_gt.csv (or traj.csv) with columns t,x,y[,...].
Both sides are scored with the SAME von Benzon RMSE metric (score_ranking.py).

Reported:
  - both rankings side by side, with per-controller RMSE mean +/- run spread
  - Spearman rho and Kendall tau between the rankings
  - CHANCE test: exact permutation p-value — probability a random ranking of n
    controllers correlates this well (small n makes this exactly enumerable).
  - NOISE-FLOOR test: Monte Carlo under the null "sim and real agree perfectly;
    observed swaps are run-to-run noise". Each side's per-controller RMSE is
    redrawn as Normal(shared mean, that side's measured seed spread); the
    resulting rho distribution is the reference floor band. Observed rho inside
    the band -> disagreement attributable to noise; below it -> the sim
    genuinely mis-orders controllers beyond what noise explains.

Usage:
  python3 score_correlation.py <sim_dir> <real_dir>
  python3 score_correlation.py --split-seeds <dir>   # mock: run 1 = "sim", run 2 = "real"
  python3 score_correlation.py --self-test
"""
import argparse
import glob
import itertools
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from score_ranking import load_gt, metrics  # same metric on both sides

MC_SAMPLES = 20000
RNG_SEED = 0  # deterministic report


# ---------------------------------------------------------------- rank stats
def rankdata(v):
    """Average ranks (1-based), ties averaged."""
    v = np.asarray(v, float)
    order = np.argsort(v, kind="stable")
    ranks = np.empty(len(v))
    i = 0
    while i < len(v):
        j = i
        while j + 1 < len(v) and v[order[j + 1]] == v[order[i]]:
            j += 1
        ranks[order[i:j + 1]] = (i + j) / 2 + 1
        i = j + 1
    return ranks


def spearman(a, b):
    ra, rb = rankdata(a), rankdata(b)
    ra, rb = ra - ra.mean(), rb - rb.mean()
    d = math.sqrt((ra ** 2).sum() * (rb ** 2).sum())
    return float((ra * rb).sum() / d) if d > 0 else 0.0


def kendall(a, b):
    n = len(a)
    conc = disc = 0
    for i, j in itertools.combinations(range(n), 2):
        s = (a[i] - a[j]) * (b[i] - b[j])
        if s > 0:
            conc += 1
        elif s < 0:
            disc += 1
    tot = n * (n - 1) / 2
    return (conc - disc) / tot if tot else 0.0


def chance_p(rmse_a, rmse_b):
    """Exact permutation p: P(spearman(random ranking, b) >= observed)."""
    obs = spearman(rmse_a, rmse_b)
    n = len(rmse_a)
    if n > 8:  # 8! = 40320 still fine; beyond that sample
        rng = np.random.default_rng(RNG_SEED)
        rhos = [spearman(rng.permutation(rmse_a), rmse_b) for _ in range(MC_SAMPLES)]
        return obs, float(np.mean([r >= obs - 1e-12 for r in rhos]))
    rhos = [spearman(np.asarray(rmse_a)[list(p)], rmse_b)
            for p in itertools.permutations(range(n))]
    return obs, float(np.mean([r >= obs - 1e-12 for r in rhos]))


def noise_floor_band(stats_a, stats_b, names):
    """rho distribution under 'both sides share true means; spreads are noise'.

    Shared mean = average of the two sides' RMSE means per controller. Spread =
    each side's own seed spread (floored at the median spread so a 1-run or
    zero-spread controller doesn't pretend to be noiseless).
    """
    rng = np.random.default_rng(RNG_SEED)
    mu_a = np.array([stats_a[n][0] for n in names])
    mu_b = np.array([stats_b[n][0] for n in names])
    mu = (mu_a + mu_b) / 2
    sd_a = np.array([stats_a[n][1] for n in names])
    sd_b = np.array([stats_b[n][1] for n in names])
    # A side with a single run (or zero spread) has no measurable floor; the
    # best available noise estimate is then the paired cross-side difference
    # (successive-difference estimator: |mu_a - mu_b| / sqrt(2)).
    fallback = max(float(np.median(np.abs(mu_a - mu_b))) / math.sqrt(2), 1e-4)
    for sd in (sd_a, sd_b):
        floor = np.median(sd[sd > 0]) if (sd > 0).any() else fallback
        np.clip(sd, max(float(floor), 1e-4), None, out=sd)
    rhos = np.empty(MC_SAMPLES)
    for k in range(MC_SAMPLES):
        rhos[k] = spearman(rng.normal(mu, sd_a), rng.normal(mu, sd_b))
    return float(np.percentile(rhos, 5)), float(np.median(rhos)), \
        float(np.percentile(rhos, 95))


# ---------------------------------------------------------------- data load
DEFAULT_CSVS = ("sim_gt.csv", "traj.csv")


def load_traj(d, csvs):
    """First matching trajectory CSV (t,x,y[,...]) in run dir d, start-zeroed."""
    for fname in csvs:
        if fname == "sim_gt.csv":  # score_ranking's own loader
            t, xy = load_gt(d)
            if xy is not None:
                return t, xy
            continue
        f = os.path.join(d, fname)
        if os.path.exists(f):
            a = np.genfromtxt(f, delimiter=",", names=True)
            if a.size >= 50:
                t = a["t"] - a["t"][0]
                return t, np.stack([a["x"] - a["x"][0], a["y"] - a["y"][0]], 1)
    return None, None


def load_side(root, run_filter=None, csvs=DEFAULT_CSVS):
    """-> {controller: (rmse_mean, rmse_sd, n_runs)} using score_ranking metrics."""
    per = {}
    for d in sorted(glob.glob(os.path.join(root, "*_s*"))):
        if not os.path.isdir(d):
            continue
        base = os.path.basename(d)
        name, run = base.rsplit("_s", 1)
        if run_filter is not None and run != run_filter:
            continue
        t, xy = load_traj(d, csvs)
        if xy is None:
            continue
        per.setdefault(name, []).append(metrics(t, xy)["rmse"])
    return {n: (float(np.mean(v)), float(np.std(v)), len(v)) for n, v in per.items()}


# ---------------------------------------------------------------- report
def report(stats_sim, stats_real, label_a="sim", label_b="real"):
    names = sorted(set(stats_sim) & set(stats_real))
    dropped = sorted(set(stats_sim) ^ set(stats_real))
    if dropped:
        print(f"[warn] not on both sides, excluded: {', '.join(dropped)}")
    if len(names) < 3:
        raise SystemExit(f"need >=3 common controllers, have {len(names)}")

    rmse_a = np.array([stats_sim[n][0] for n in names])
    rmse_b = np.array([stats_real[n][0] for n in names])
    rank_a, rank_b = rankdata(rmse_a), rankdata(rmse_b)

    print(f"\n{'controller':<14}{label_a+' RMSE':>12}{'±':>7}{'rank':>5}"
          f"{label_b+' RMSE':>12}{'±':>7}{'rank':>5}")
    for i, n in enumerate(sorted(range(len(names)), key=lambda k: rank_a[k])):
        nm = names[n]
        print(f"{nm:<14}{rmse_a[n]:>12.3f}{stats_sim[nm][1]:>7.3f}{int(rank_a[n]):>5}"
              f"{rmse_b[n]:>12.3f}{stats_real[nm][1]:>7.3f}{int(rank_b[n]):>5}")

    rho, p = chance_p(rmse_a, rmse_b)
    tau = kendall(rmse_a, rmse_b)
    lo, med, hi = noise_floor_band(stats_sim, stats_real, names)

    print(f"\nSpearman rho = {rho:+.3f}   Kendall tau = {tau:+.3f}"
          f"   (n = {len(names)} controllers)")
    print(f"chance test: exact P(random ranking correlates >= observed) = {p:.4f}"
          f" -> {'BETTER THAN CHANCE' if p < 0.05 else 'not distinguishable from chance'}")
    print(f"noise floor: rho under perfect-agreement + measured run spreads = "
          f"[{lo:+.3f} .. {hi:+.3f}] (median {med:+.3f}, 5-95%)")
    if rho >= lo:
        print("verdict: observed correlation is INSIDE the noise band -> "
              f"{label_a} ordering consistent with {label_b} within run-to-run noise")
    else:
        print("verdict: observed correlation BELOW the noise band -> "
              f"{label_a} genuinely mis-orders controllers beyond noise")
    return rho, tau, p, (lo, med, hi)


# ---------------------------------------------------------------- self test
def self_test():
    a = [0.10, 0.12, 0.15, 0.18, 0.20]
    assert abs(spearman(a, a) - 1.0) < 1e-12
    assert abs(spearman(a, a[::-1]) + 1.0) < 1e-12
    assert abs(kendall(a, a) - 1.0) < 1e-12
    assert abs(kendall(a, a[::-1]) + 1.0) < 1e-12
    # known case: one adjacent swap in n=5 -> rho = 1 - 6*2/(5*24) = 0.9
    b = [0.10, 0.15, 0.12, 0.18, 0.20]
    assert abs(spearman(a, b) - 0.9) < 1e-12
    # ties average: [1, 2.5, 2.5, 4]
    assert np.allclose(rankdata([1, 2, 2, 3]), [1, 2.5, 2.5, 4])
    _, p_same = chance_p(np.array(a), np.array(a))
    assert p_same <= 1 / 100  # perfect agreement ~ 1/n! + ties
    # noise band sanity: tiny spreads -> band hugs +1
    s = {n: (v, 0.001, 2) for n, v in zip("abcde", a)}
    lo, _, hi = noise_floor_band(s, s, list("abcde"))
    assert lo > 0.85 and hi == 1.0, (lo, hi)
    # single-run sides (sd=0): paired-difference fallback must widen the band
    # rather than collapse it to exactly +1
    s1 = {n: (v, 0.0, 1) for n, v in zip("abcde", a)}
    s2 = {n: (v + d, 0.0, 1)
          for n, v, d in zip("abcde", a, [0.01, -0.02, 0.02, -0.01, 0.01])}
    lo, _, _ = noise_floor_band(s1, s2, list("abcde"))
    assert lo < 0.95, lo
    print("self-test OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="*", help="<sim_dir> <real_dir>")
    ap.add_argument("--split-seeds", metavar="DIR",
                    help="mock validation: run 1 of DIR = 'sim', run 2 = 'real'")
    ap.add_argument("--csv-a", default=None, metavar="NAME",
                    help="trajectory CSV for side A (e.g. sim_dr.csv for the "
                         "matched DR-vs-DR comparison; default sim_gt.csv)")
    ap.add_argument("--csv-b", default=None, metavar="NAME",
                    help="trajectory CSV for side B (default sim_gt.csv/traj.csv, "
                         "use sim_dr.csv for real-vehicle recordings)")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    csvs_a = (a.csv_a,) if a.csv_a else DEFAULT_CSVS
    csvs_b = (a.csv_b,) if a.csv_b else DEFAULT_CSVS
    if a.self_test:
        self_test()
        return
    if a.split_seeds:
        report(load_side(a.split_seeds, run_filter="1", csvs=csvs_a),
               load_side(a.split_seeds, run_filter="2", csvs=csvs_b),
               label_a="run1", label_b="run2")
        return
    if len(a.dirs) != 2:
        ap.error("need <sim_dir> <real_dir> (or --split-seeds / --self-test)")
    report(load_side(a.dirs[0], csvs=csvs_a), load_side(a.dirs[1], csvs=csvs_b))


if __name__ == "__main__":
    main()
