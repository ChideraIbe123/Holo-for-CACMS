#!/usr/bin/env python3
"""Digital-twin fidelity scorecard with a real-to-real reference floor.

Compares SIM runs against REAL runs using distributional distances on the sensor
streams, and judges fidelity by the reference-floor criterion: the twin passes a
channel if sim-to-real distances are statistically indistinguishable from
real-to-real distances (Mann-Whitney U, two-sided).

Distances per channel (gyro xyz, accel xyz, dvl xyz, rel_alt):
  - KS statistic and Wasserstein-1 on INCREMENT distributions (motion-robust)
  - log-spectral L2 distance on Welch PSDs
  - RBF-MMD on 5 s window feature vectors (std of every channel per window)

Usage: python3 fidelity_scorecard.py --real real1.npz real2.npz ... --sim sim1.npz ...
       [--out report_prefix]
"""
import argparse
import itertools
import json

import numpy as np

CHANNELS = [("gyro_x", "imu", 1), ("gyro_y", "imu", 2), ("gyro_z", "imu", 3),
            ("accel_x", "imu", 4), ("accel_y", "imu", 5), ("accel_z", "imu", 6),
            ("dvl_x", "dvl", 1), ("dvl_y", "dvl", 2), ("dvl_z", "dvl", 3),
            ("rel_alt", "alt", 1)]


def load(path):
    d = np.load(path)
    return {k: d[k] for k in ("imu", "dvl", "alt")}


def increments(run, arr_key, col):
    a = run[arr_key]
    if len(a) < 20:
        return None
    return np.diff(a[:, col])


def ks_stat(a, b):
    a, b = np.sort(a), np.sort(b)
    grid = np.concatenate([a, b])
    ca = np.searchsorted(a, grid, side="right") / len(a)
    cb = np.searchsorted(b, grid, side="right") / len(b)
    return float(np.abs(ca - cb).max())


def wasserstein1(a, b):
    n = 512
    qs = np.linspace(0.01, 0.99, n)
    return float(np.abs(np.quantile(a, qs) - np.quantile(b, qs)).mean())


def log_psd(x, fs):
    """Welch-style averaged periodogram, log10."""
    seg = 128
    if len(x) < 2 * seg:
        seg = max(32, len(x) // 4)
    hops = range(0, len(x) - seg, seg // 2)
    w = np.hanning(seg)
    ps = [np.abs(np.fft.rfft((x[i:i + seg] - x[i:i + seg].mean()) * w)) ** 2 for i in hops]
    return np.log10(np.mean(ps, axis=0) + 1e-12)


def spectral_dist(a, b, fs=10.0):
    pa, pb = log_psd(a, fs), log_psd(b, fs)
    n = min(len(pa), len(pb))
    return float(np.sqrt(np.mean((pa[:n] - pb[:n]) ** 2)))


def window_features(run, win=5.0):
    """Per 5 s window: std of every channel -> feature matrix."""
    feats = []
    imu, dvl = run["imu"], run["dvl"]
    if len(imu) < 30:
        return None
    t0, t1 = imu[0, 0], imu[-1, 0]
    for ws in np.arange(t0, t1 - win, win):
        mi = imu[(imu[:, 0] >= ws) & (imu[:, 0] < ws + win)]
        md = dvl[(dvl[:, 0] >= ws) & (dvl[:, 0] < ws + win)] if len(dvl) else np.empty((0, 4))
        if len(mi) < 10:
            continue
        f = list(mi[:, 1:7].std(axis=0))
        f += list(md[:, 1:4].std(axis=0)) if len(md) > 5 else [0, 0, 0]
        feats.append(f)
    return np.array(feats) if feats else None


def mmd_rbf(X, Y):
    """RBF-kernel MMD^2 with median-heuristic bandwidth (features z-scored jointly)."""
    Z = np.vstack([X, Y])
    mu, sd = Z.mean(0), Z.std(0) + 1e-9
    X, Y = (X - mu) / sd, (Y - mu) / sd
    Z = np.vstack([X, Y])
    d2 = ((Z[:, None, :] - Z[None, :, :]) ** 2).sum(-1)
    gamma = 1.0 / (np.median(d2[d2 > 0]) + 1e-9)
    K = np.exp(-gamma * d2)
    n, m = len(X), len(Y)
    kxx = (K[:n, :n].sum() - np.trace(K[:n, :n])) / (n * (n - 1))
    kyy = (K[n:, n:].sum() - np.trace(K[n:, n:])) / (m * (m - 1))
    kxy = K[:n, n:].mean()
    return float(kxx + kyy - 2 * kxy)


def mannwhitney_p(a, b):
    """Two-sided Mann-Whitney U via normal approximation."""
    a, b = np.asarray(a), np.asarray(b)
    n, m = len(a), len(b)
    allv = np.concatenate([a, b])
    ranks = allv.argsort().argsort() + 1.0
    # midranks for ties
    order = np.argsort(allv)
    sv = allv[order]
    r = np.empty_like(ranks)
    i = 0
    while i < len(sv):
        j = i
        while j + 1 < len(sv) and sv[j + 1] == sv[i]:
            j += 1
        r[order[i:j + 1]] = (i + j) / 2 + 1
        i = j + 1
    U = r[:n].sum() - n * (n + 1) / 2
    mu = n * m / 2
    sd = np.sqrt(n * m * (n + m + 1) / 12.0) + 1e-12
    z = (U - mu) / sd
    from math import erf
    return float(2 * (1 - 0.5 * (1 + erf(abs(z) / np.sqrt(2)))))


def pair_distances(runs_a, runs_b, exclude_same=False):
    """All cross distances between two run sets, per channel per metric."""
    out = {name: {"ks": [], "w1": [], "spec": []} for name, _, _ in CHANNELS}
    out["_mmd"] = []
    pairs = [(i, j) for i in range(len(runs_a)) for j in range(len(runs_b))
             if not (exclude_same and i >= j)]
    for i, j in pairs:
        ra, rb = runs_a[i], runs_b[j]
        for name, key, col in CHANNELS:
            da, db = increments(ra, key, col), increments(rb, key, col)
            if da is None or db is None:
                continue
            out[name]["ks"].append(ks_stat(da, db))
            out[name]["w1"].append(wasserstein1(da, db))
            out[name]["spec"].append(spectral_dist(ra[key][:, col], rb[key][:, col]))
        fa, fb = window_features(ra), window_features(rb)
        if fa is not None and fb is not None and len(fa) > 2 and len(fb) > 2:
            out["_mmd"].append(mmd_rbf(fa, fb))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", nargs="+", required=True)
    ap.add_argument("--sim", nargs="+", required=True)
    ap.add_argument("--out", default="scorecard")
    args = ap.parse_args()

    real = [load(p) for p in args.real]
    sim = [load(p) for p in args.sim]
    print(f"loaded {len(real)} real runs, {len(sim)} sim runs")

    rr = pair_distances(real, real, exclude_same=True)   # reference floor
    sr = pair_distances(sim, real)                        # twin vs reality

    report = {}
    print(f"\n{'channel':<10}{'metric':<6}{'real-real med':>14}{'sim-real med':>14}{'ratio':>7}{'p(MWU)':>8}  verdict")
    passes = total = 0
    for name, _, _ in CHANNELS:
        for metric in ("ks", "w1", "spec"):
            a, b = rr[name][metric], sr[name][metric]
            if not a or not b:
                continue
            med_a, med_b = float(np.median(a)), float(np.median(b))
            p = mannwhitney_p(a, b)
            ok = (p > 0.05) or (med_b <= med_a)
            passes += ok; total += 1
            report[f"{name}.{metric}"] = {"real_real_med": med_a, "sim_real_med": med_b, "p": p, "pass": bool(ok)}
            print(f"{name:<10}{metric:<6}{med_a:>14.4g}{med_b:>14.4g}{med_b/max(med_a,1e-12):>7.2f}{p:>8.3f}  {'PASS' if ok else 'FAIL'}")
    if rr["_mmd"] and sr["_mmd"]:
        med_a, med_b = float(np.median(rr["_mmd"])), float(np.median(sr["_mmd"]))
        p = mannwhitney_p(rr["_mmd"], sr["_mmd"])
        ok = (p > 0.05) or (med_b <= med_a)
        passes += ok; total += 1
        report["window_mmd"] = {"real_real_med": med_a, "sim_real_med": med_b, "p": p, "pass": bool(ok)}
        print(f"{'windows':<10}{'mmd':<6}{med_a:>14.4g}{med_b:>14.4g}{med_b/max(med_a,1e-12):>7.2f}{p:>8.3f}  {'PASS' if ok else 'FAIL'}")

    print(f"\nSCORE: {passes}/{total} channel-metrics pass the reference-floor criterion")
    with open(args.out + ".json", "w") as f:
        json.dump(report, f, indent=2)
    print(f"wrote {args.out}.json")


if __name__ == "__main__":
    main()
