#!/usr/bin/env python3
"""Power analysis for the sim-vs-real ranking-correlation experiment.

Question answered BEFORE spending pool time: with 6 controllers whose sim RMSE
means/spreads are known (frozen 4-seed in-pool benchmark), how many real-side
repeats are needed for the experiment to be informative?

Two operating characteristics, Monte Carlo over measured spreads:
  POWER   P(detect agreement)   — if reality truly shares the sim's ordering,
          how often does the measured Spearman rho clear the exact-permutation
          chance threshold (alpha = 0.05, n=6 -> rho >= ~0.77)?
  FPR     P(false agreement)    — if reality's ordering were RANDOM, how often
          would we wrongly clear the same threshold? (should stay ~alpha)
Also reports the expected MMRV under true agreement (what "good" looks like).

Real-side spreads are unknown, so three scenarios: same as sim, 1.5x, 2x
(real runs add battery/tether/pilot variance).

Usage: power_analysis.py            # uses the frozen bench5 numbers below
"""
import itertools

import numpy as np

# Frozen 4-seed in-pool predictions, v18 twin, seeded runs (2026-10-28).
# Seven registered entrants: (RMSE mean, seed std) per course.
BENCH_RECT = {
    "pursuit":   (0.1431, 0.0012),
    "los":       (0.1499, 0.0016),
    "alos":      (0.1591, 0.0032),
    "smc":       (0.1676, 0.0024),
    "ilos":      (0.1707, 0.0009),
    "p":         (0.1769, 0.0031),
    "fuzzy_lab": (0.2180, 0.0019),
}
BENCH_LINE = {
    "los":       (0.1343, 0.0017),
    "alos":      (0.1430, 0.0027),
    "fuzzy_lab": (0.1467, 0.0031),
    "ilos":      (0.1510, 0.0010),
    "pursuit":   (0.1780, 0.0024),
    "smc":       (0.2170, 0.0037),
    "p":         (0.2241, 0.0024),
}
BENCHES = {"rect": BENCH_RECT, "line": BENCH_LINE}
ALPHA = 0.05
MC = 20000
RNG = np.random.default_rng(3)


def spearman(a, b):
    ra = np.argsort(np.argsort(a))
    rb = np.argsort(np.argsort(b))
    ra, rb = ra - ra.mean(), rb - rb.mean()
    d = np.sqrt((ra ** 2).sum() * (rb ** 2).sum())
    return (ra * rb).sum() / d if d > 0 else 0.0


def mmrv(s, r):
    n = len(s)
    worst = np.zeros(n)
    for i in range(n):
        for j in range(n):
            if (s[i] - s[j]) * (r[i] - r[j]) < 0:
                worst[i] = max(worst[i], abs(r[i] - r[j]))
    return worst.mean()


def chance_threshold(n, alpha):
    """Smallest rho whose exact permutation p-value is < alpha."""
    base = np.arange(n, dtype=float)
    rhos = sorted((spearman(np.array(p, dtype=float), base)
                   for p in itertools.permutations(range(n))), reverse=True)
    k = int(alpha * len(rhos))
    return rhos[k]


def main():
  for course, bench in BENCHES.items():
    print("=" * 20, "course:", course, "=" * 20)
    names = list(bench)
    mu = np.array([bench[n][0] for n in names])
    sd_sim = np.array([bench[n][1] for n in names])
    n = len(names)
    thr = chance_threshold(n, ALPHA)
    print(f"n = {n} controllers; exact chance threshold: rho >= {thr:.3f} "
          f"for p < {ALPHA}")
    print(f"sim side: 4 seeds (spread of the mean = sd/2)\n")
    print(f"{'real spread':>12} {'repeats':>8} {'POWER':>7} {'FPR':>6} "
          f"{'E[rho|agree]':>13} {'E[MMRV|agree] m':>16}")
    # v18 seeded runs collapsed the sim seed std to 1-4 mm, so real spread
    # scenarios are ABSOLUTE (m RMSE std per run), not multiples of sim:
    # 5 mm = repeats as tight as the Oct-8 shakedown pairs, 20 mm = pessimistic
    # (battery/tether/handling drift between runs).
    for sd_abs in (0.005, 0.010, 0.020):
        sd_real = np.full_like(sd_sim, sd_abs)
        for reps in (2, 3, 4, 5):
            det = fp = 0
            rho_sum = rv_sum = 0.0
            for _ in range(MC):
                sim_obs = RNG.normal(mu, sd_sim / np.sqrt(4))
                real_true = RNG.normal(mu, sd_real / np.sqrt(reps))
                r = spearman(sim_obs, real_true)
                rho_sum += r
                rv_sum += mmrv(sim_obs, real_true)
                det += r >= thr
                real_rand = RNG.permutation(real_true)
                fp += spearman(sim_obs, real_rand) >= thr
            print(f"{sd_abs:>10.3f}m {reps:>8} {det/MC:>7.2f} {fp/MC:>6.3f} "
                  f"{rho_sum/MC:>13.2f} {rv_sum/MC:>16.4f}")
        print()
  print("Read: POWER is the probability the experiment DETECTS agreement "
        "(rho above chance)\nwhen the sim ordering is truly correct; FPR "
        "should sit near alpha. If POWER is low\nat feasible repeats, the "
        "fix is separating controllers more (gain spacing), not\nmore "
        "repeats alone — the fuzzy/p-vs-rest gaps carry most of the "
        "signal.")


if __name__ == "__main__":
    main()
