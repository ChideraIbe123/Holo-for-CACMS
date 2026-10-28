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
- **2026-10-10 — Amendment 1: actual experiment entrants frozen.** The original
  table ranked six in-house stand-in controllers; the real experiment's entrants
  are the five laws delivered in the pilot kit (P, pursuit, LOS, ILOS, and the
  lab's fuzzy tracker). THE BINDING SIM PREDICTION is their ranking under the
  official procedure (4 seeds, Intex pool twin, 2.0x0.6 m course, RMSE m),
  generated at code commit `7a3504c` with the twin physics unchanged since
  v1.0.0 (all post-freeze code changes are opt-in flags, default-off):

  | rank | law | sim RMSE | ±seed |
  |---|---|---|---|
  | 1 | pursuit | 0.140 | 0.002 |
  | 2 | los | 0.154 | 0.001 |
  | 3 | ilos | 0.174 | 0.002 |
  | 4 | p | 0.181 | 0.002 |
  | 5 | fuzzy_lab | 0.226 | 0.001 |

  Analysis update for n=5: exact chance threshold is rho >= 0.800 (alpha=0.05);
  power at 4 real repeats = 0.92 if the sim ordering is correct; the design
  tolerates up to two adjacent swaps. Repeats: 4 per law, line course included.
  DISCLOSURE: a shakedown pool session (2026-10-08, 1-2 runs per law) occurred
  before this amendment and its preliminary comparison was examined. The sim
  prediction above was NOT altered in response — the twin is unchanged since
  v1.0.0; the only change is running the newly delivered laws through the
  frozen twin. The shakedown is reported as such and is not the confirmatory
  session.
- **2026-10-12 — Amendment 2: analysis statistics + failure-mode prediction (added
  BEFORE the confirmatory session; no confirmatory data exists).**
  (a) PRIMARY statistic updated to pairwise-separable ordering agreement: of the
  controller pairs whose RMSE gap exceeds the summed repeat spreads on BOTH sides,
  the fraction the sim orders correctly, with a one-sided binomial test vs chance;
  MMRV (margin-weighted, SIMPLER convention) reported with it. Spearman rho + exact
  permutation p and the noise-floor band remain secondary. Rationale: field-standard
  for small-n ranking validation (SIMPLER lineage); adopted from the literature, not
  from our data.
  (b) ADDITIONAL PRE-REGISTERED PREDICTION (falsifiable): under the frozen twin,
  fuzzy_lab is the only law whose maximum lateral excursion on the rectangle exceeds
  the 1.0 m safety boundary (sim max |y| = 1.02 m; all other laws 0.80-0.92 m).
  Prediction for the confirmatory session: fuzzy_lab is the law most likely to
  approach or breach the fence; no other law breaches it. (The 2026-10-08 shakedown
  is consistent — fuzzy crossed at 1.03 m — and is disclosed as prior observation;
  this amendment makes the prediction binding for the confirmatory session.)
- **2026-10-22 — Amendment 3: course-pooled primary statistic + Pearson (added
  BEFORE the confirmatory session; no confirmatory data exists).** The primary
  pairwise-separable statistic is computed over pairs POOLED ACROSS BOTH courses
  (rectangle and line), following the field-standard remedy for unstable
  small-n rank statistics (H2RBench pools across tasks for the same reason;
  arXiv 2609.24778). Pearson r is reported alongside Spearman/Kendall/MMRV,
  completing the standard trio of the SIMPLER-lineage evaluators. Implemented
  in sim/score_correlation.py (--sim2/--real2 pool a second course); adopted
  from the literature, not from our data.
- **2026-10-28 — Amendment 4: entrant set expanded to seven; v18 predictions
  frozen (added BEFORE the confirmatory session; no confirmatory data exists).**
  (a) TWO ENTRANTS ADDED, chosen from the literature (mechanism diversity per
  the ranking-validation field standard), not from our data; the lab's learned
  controllers (ANFIS-DDPG, PPO) are unavailable for the session. `smc` — the
  boundary-layer sliding-mode heading law of von Benzon et al. 2022 (JMSE
  10(12) 1898 sec 7.1, Table 6 yaw gains unchanged), a port of the original
  stand-in table's sixth entrant to the shared tracker, restoring the
  robust-control family. `alos` — adaptive LOS (Fossen, Pettersen & Galeazzi,
  IEEE TCST 2015), crab-angle estimation (gamma=0.3, beta clamp 0.6 rad;
  delta=0.4 and kpsi=1.4 matching los/ilos), adding the adaptive family.
  Controller code: commit `83d3aec`; all seven laws pass `test_tracker.py` on
  both courses.
  (b) TWIN RE-FREEZE UNDER v18. Since Amendment 1 the twin received
  measurement-chain and surface-dynamics fixes (v18: engine depth-sensor noise
  relocated into the bridge and venue-conditioned, surface heave damping,
  spawn at float point, seeded bit-reproducible runs; fidelity 41/41 on all 4
  seeds in the matched-venue configuration). THE BINDING SIM PREDICTION is
  regenerated under v18: 4 seeds (`--seed 1..4 --venue pool`), Intex pool
  twin, both courses, RMSE (m):

  | rect rank | law | RMSE | ±seed |  | line rank | law | RMSE | ±seed |
  |---|---|---|---|---|---|---|---|---|
  | 1 | pursuit | 0.1431 | 0.0012 |  | 1 | los | 0.1343 | 0.0017 |
  | 2 | los | 0.1499 | 0.0016 |  | 2 | alos | 0.1430 | 0.0027 |
  | 3 | alos | 0.1591 | 0.0032 |  | 3 | fuzzy_lab | 0.1467 | 0.0031 |
  | 4 | smc | 0.1676 | 0.0024 |  | 4 | ilos | 0.1510 | 0.0010 |
  | 5 | ilos | 0.1707 | 0.0009 |  | 5 | pursuit | 0.1780 | 0.0024 |
  | 6 | p | 0.1769 | 0.0031 |  | 6 | smc | 0.2170 | 0.0037 |
  | 7 | fuzzy_lab | 0.2180 | 0.0019 |  | 7 | p | 0.2241 | 0.0024 |

  CONSISTENCY DISCLOSURE: the five incumbent laws keep their Amendment-1 rect
  order and their 2026-10-23 line order exactly; v18 moved no incumbent pair.
  The fuzzy_lab failure-mode prediction of Amendment 2 carries over unchanged.
  (c) POWER AT n=7 (power_analysis.py, absolute real-spread scenarios 5/10/20
  mm): exact chance threshold rho >= 0.679 (alpha=0.05). Three repeats per law
  per course give power 0.89-1.00 (0.89 is the pessimistic 20 mm rect case);
  a fourth rect repeat raises that case to 0.94. SESSION PLAN: 3 repeats per
  law per course (42 runs), randomized order, with a fourth rect repeat for
  all laws if pool time remains. This supersedes the 4-repeat/5-law plan of
  Amendment 1.
