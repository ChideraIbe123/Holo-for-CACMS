# Pre-registration: sim-vs-real controller-ranking experiment

**Date frozen:** 2026-09-30
**Frozen configuration:** repo `ChideraIbe123/Holo-for-CACMS` release **v1.0.0**,
commit `8acc9ecaa728b9e2a67cd010b10c297da6f45ea8`. All simulator physics, noise,
sensor, and environment parameters are fixed at this commit BEFORE any real
ranking session. Any later change to the twin voids this pre-registration and
requires a new one.

## Frozen fidelity state (context, not a hypothesis)

38/41 matched-venue statistical checks indistinguishable from real recordings
(conformal criterion, α=0.95; strict MWU secondary agrees at 37/41); run-level
verdict rule = max-z composite of Wasserstein-1 + increment-ACF (selected by
AUC against graded negatives, never exposed to the final sim); all 12 final-sim
runs verdict "close". Dynamics fitted per-axis; knee-trimming of transients
tested and rejected.

## Hypothesis

H1: the simulator ranks path-following controllers in the same order the real
vehicle does, beyond chance and within run-to-run noise.

## Frozen sim-side ranking (to be tested, not revised)

4 seeds per controller, drag-corrected (v17) model, Intex pool twin,
2.0 × 0.6 m course, RMSE (m) per von Benzon 2022 metrics:

| rank | controller | RMSE | ±seed |
|---|---|---|---|
| 1 | aggressive | 0.089 | 0.019 |
| 2 | pursuit | 0.111 | 0.006 |
| 3 | smc_vonbenzon | 0.117 | 0.012 |
| 4 | p_baseline | 0.119 | 0.008 |
| 5 | tight_pd | 0.123 | 0.011 |
| 6 | sluggish | 0.143 | 0.003 |

NOTE: when the lab's real controllers (ANFIS-DDPG / fuzzy / PPO) are added via
`lab_controller_adapter.py`, the benchmark will be rerun at ≥4 seeds and THIS
TABLE REPLACED — the freeze that binds is the one dated before the real
session, with its own commit hash appended below.

## Planned real session (docs/real_ranking_protocol.md governs)

Same 2.0 × 0.6 m rectangle in the real Intex pool; **4 repeats per controller**
(power analysis, `power_analysis.py`: 3 repeats = 0.80 power best-case but
0.56–0.67 if real spread is 1.5–2× sim; 4 repeats recovers 0.61–0.84; FPR
calibrates at α). Randomized controller order across repeats; runs with pilot
intervention voided and rerun; both rosbags and live `sim_dr.csv` recorded.

## Planned analysis (exact commands, no alternatives)

1. Real-side ranking: `score_ranking.py` on the session directory.
2. Primary comparison (matched observable, DR-vs-DR):
   `score_correlation.py <sim> <real> --csv-a sim_dr.csv --csv-b sim_dr.csv`
3. Reported: Spearman ρ with exact-permutation chance p (α=0.05, n controllers
   → threshold ρ≈0.77 at n=6), Kendall τ, **MMRV** (SIMPLER convention), and
   the Monte-Carlo noise-floor band (perfect-agreement null with measured
   spreads). Secondary: sim ground-truth vs real DR.
4. Interpretation rule, fixed in advance: ρ ≥ chance threshold AND within the
   noise band ⇒ H1 supported; ρ below the noise band ⇒ sim mis-orders beyond
   noise (H1 rejected); otherwise inconclusive — reported as such, not
   reframed.
5. The new tub-session recordings double as a held-out fidelity validation of
   the frozen twin (scorecard at this commit, no retuning before scoring).

## Amendment log

- (append-only; each entry: date, what changed, new commit hash)
