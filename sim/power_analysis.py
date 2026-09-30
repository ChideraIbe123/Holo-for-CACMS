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

# Frozen 4-seed in-pool benchmark (drag-corrected model, 2026-09-20):
# controller: (RMSE mean, seed std)
BENCH = {
    "aggressive":    (0.089, 0.019),
    "pursuit":       (0.111, 0.006),
    "smc_vonbenzon": (0.117, 0.012),
    "p_baseline":    (0.119, 0.008),
    "tight_pd":      (0.123, 0.011),
    "sluggish":      (0.143, 0.003),
}
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
    names = list(BENCH)
    mu = np.array([BENCH[n][0] for n in names])
    sd_sim = np.array([BENCH[n][1] for n in names])
    n = len(names)
    thr = chance_threshold(n, ALPHA)
    print(f"n = {n} controllers; exact chance threshold: rho >= {thr:.3f} "
          f"for p < {ALPHA}")
    print(f"sim side: 4 seeds (spread of the mean = sd/2)\n")
    print(f"{'real spread':>12} {'repeats':>8} {'POWER':>7} {'FPR':>6} "
          f"{'E[rho|agree]':>13} {'E[MMRV|agree] m':>16}")
    for scale in (1.0, 1.5, 2.0):
        sd_real = sd_sim * scale
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
            print(f"{scale:>11.1f}x {reps:>8} {det/MC:>7.2f} {fp/MC:>6.3f} "
                  f"{rho_sum/MC:>13.2f} {rv_sum/MC:>16.4f}")
        print()
    print("Read: POWER is the probability the experiment DETECTS agreement "
          "(rho above chance)\nwhen the sim ordering is truly correct; FPR "
          "should sit near alpha. If POWER is low\nat feasible repeats, the "
          "fix is separating controllers more (gain spacing), not\nmore "
          "repeats alone — the aggressive-vs-rest gap carries most of the "
          "signal.")


if __name__ == "__main__":
    main()
