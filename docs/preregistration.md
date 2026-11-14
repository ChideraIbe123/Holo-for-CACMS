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
- **2026-11-12 — Amendment 5: real-run start condition aligned to the simulator
  (added BEFORE the confirmatory session; no confirmatory data exists).**
  The simulator begins every prediction run with the vehicle already at its
  0.5 m target depth and stationary (in-pool spawn, `intex_pool.SPAWN`). The
  real protocol previously started the controller with the vehicle floating at
  the surface, so the first ~2 s of each real run (20% of a line run) were a
  dive the simulator never performed. From this amendment, `run_one.sh` passes
  `--dive-first` to `course_runner.py`: the runner (a) requires the estimator
  depth to read within 0.10 m of zero at start (the estimator zeroes depth at
  the `/deadreckon/reset` sent while the vehicle floats), (b) sinks with zero
  surge and yaw until the depth sensor is within 0.05 m of the target for
  1.0 s (12 s timeout, flagged in `depth_gate.json`), and (c) lays out the
  course from the pose at that moment. Depth and DVL altitude are recorded
  before, at and during the course. The steering laws, the simulator, and the
  Amendment 4 predictions are unchanged: the simulator path does not use the
  flag and starts at depth already. Known residual: the real depth sensor
  floats ~0.06 m below the surface, so a 0.5 m dive places it ~0.56 m down
  versus 0.50 m in the simulator; recorded, not corrected. Runs that time out
  the dive gate are reported separately. Code: `real/course_runner.py`
  (`DepthGate`), offline test `real/test_depth_gate.py`.
- **2026-11-13 — Amendment 6: vehicle model re-fitted from measurements (DVL payload,
  float trim), real start-up reproduced in the twin, predictions re-frozen (added
  BEFORE the confirmatory session; no confirmatory controller data exists).**
  DISCLOSURE: on 2026-11-12 the lab ran the vehicle in the pool to develop and tune
  the start-up procedure in (b) (runs aborted or used for tuning; a first rectangle
  run struck the far wall under the old surface-start protocol). None of those runs
  were scored or compared with the simulator; the changes below were driven by the
  October float recordings, the DVL datasheet and the lab's transform file.
  (a) VEHICLE MODEL v19 (`sim/bluerov2_standard_model.py`, commits `acf6646`,
  `18c0e05`). The Water Linked DVL A50 (170 g in air, 105 g in water) is added as a
  point mass at the lab's measured mount (17 cm aft, 9 cm left, 11 cm below centre):
  mass 11.67 kg, inertia +2.2/4.5/3.9 %. The centre of gravity is set so the model
  floats at the MEASURED attitude of the lab's vehicle, pitch +12.72 deg nose-down,
  roll +0.49 deg (tub float recordings, sd 0.02 deg; confirmed by the resting
  accelerometer, accel_x = -2.01 m/s^2 = -g sin 11.8 deg); the bare Wu body floated
  level. Net buoyancy (0.2 kg) remains an assumption. The simulated DVL altitude is
  now measured from the DVL face rather than the depth sensor.
  (b) PROTOCOL. Each real run now starts with `course_runner.py --init` (rov-pilot-kit
  branch confirmatory-session, pushed 2026-11-12): the vehicle floats on the start
  mark, the estimator is zeroed there, the vehicle sinks to the depth target and turns
  to a saved heading (the course axis), and the course starts only after depth
  (0.05 m) and heading (6 deg) have held for 1 s; a floor guard caps the depth to keep
  0.30 m under the DVL. The course is anchored at the placement point along the saved
  heading. The TWIN RUNS THE SAME CODE PATH (`real/sim_run.sh INIT=1`: surface spawn,
  estimator reset at the float point, `--init`), in a 0.965 m pool (38 in, as
  reported by the operator; the published fill line is 1.05 m). If the real floating
  DVL altitude recorded in `start_gate.json` differs from the twin's 0.81 m by more
  than 0.05 m, the campaign in (c) is re-run at the matching water depth before the
  session and logged here as Amendment 6b.
  (c) BINDING PREDICTIONS, v19 twin, 4 repeats (`--seed 1..4`; the seed fixes the
  sensor streams, not the closed-loop timing, so the spread is the twin's run-to-run
  floor), Intex pool twin, RMSE (m), campaign `pred_v19` (84 runs, no aborted
  start-up; settle 10.5-13.6 s; depth target -0.497 m, guard inactive at this depth):

  | rect rank | law | RMSE | ±rep |  | line rank | law | RMSE | ±rep |
  |---|---|---|---|---|---|---|---|---|
  | 1 | pursuit | 0.1119 | 0.0021 |  | 1 | alos | 0.1337 | 0.0031 |
  | 2 | los | 0.1155 | 0.0028 |  | 2 | los | 0.1366 | 0.0028 |
  | 3 | smc | 0.1159 | 0.0025 |  | 3 | ilos | 0.1434 | 0.0017 |
  | 4 | alos | 0.1163 | 0.0020 |  | 4 | fuzzy_lab | 0.1470 | 0.0010 |
  | 5 | p | 0.1222 | 0.0011 |  | 5 | pursuit | 0.1801 | 0.0011 |
  | 6 | ilos | 0.1331 | 0.0026 |  | 6 | smc | 0.2168 | 0.0026 |
  | 7 | fuzzy_lab | 0.1663 | 0.0008 |  | 7 | p | 0.2221 | 0.0016 |

  (d) WHAT CHANGED vs Amendment 4 and WHY. Spearman between the Amendment-4 and
  v19 orders is +0.93 on both courses. Rect: pursuit and los stay 1st/2nd, fuzzy_lab
  stays last; p and ilos swap (5th/6th); los, smc and alos are now within 0.8 mm and
  will be treated as inseparable by the primary statistic. Line: alos and los swap
  (2.9 mm), ilos and fuzzy_lab swap (3.6 mm); pursuit, smc, p unchanged. An
  attribution set (same v19 model on the pre-Amendment-6 path: 1.05 m, spawn at
  depth, no start-up; 2 repeats) reproduces the full set within 1-2 mm per law, so
  the change is attributable to the vehicle model (trim + payload), not to the
  start-up or pool depth. Absolute RMSE fell 20-25 % on the rectangle while lap
  times did not change (p: 40.6 s), i.e. gentler cornering; corner overshoot
  shrank from 0.27-0.42 m to 0.17-0.29 m beyond the course ends.
  (e) FAILURE-MODE PREDICTION RESTATED. Under v19, fuzzy_lab remains the law with the
  largest lateral excursion (max |y| 0.92-0.94 m over 4 repeats; all others
  0.73-0.86 m) but it no longer crosses the 1.0 m fence (Amendment 2 / v18: 1.01-
  1.03 m). Binding prediction: fuzzy_lab is the law most likely to approach the fence;
  no law breaches it. A real fuzzy_lab breach would indicate lateral disturbance the
  twin does not model (the tether, whose ~0.5 N stability cliff for fuzzy_lab was
  established 2026-10-23). The 2026-10-08 shakedown breach at 1.03 m occurred under
  the old surface-start protocol and is disclosed as prior observation.
  (f) POWER (`sim/power_analysis.py`, v19 spreads; chance threshold rho >= 0.679 at
  n = 7). Rectangle: 3 repeats 0.98/0.83/0.55 at 5/10/20 mm real spread, 4 repeats
  0.99/0.88/0.61. Line: >= 0.96 in every case. The primary statistic (pairwise-
  separable agreement pooled across both courses, Amendments 2-3) is unchanged.
  SESSION PLAN: 3 repeats per law per course (42 runs, randomized order as printed in
  `real/CONTROLLER_RUN_INSTRUCTIONS.md`); a 4th rectangle repeat is now strongly
  recommended rather than optional. Runs whose start-up aborts are not runs (folder
  `*_ABORTED`) and are simply repeated; runs whose floor guard raised the target are
  kept and the depth reported.
  (g) Fidelity under the v19 model (tub scorecard, same configuration as the 41/41
  result) is being re-scored at the time of writing and will be appended as a
  context line; it is not a hypothesis of this pre-registration.
  (g, result 2026-11-13) Fidelity under the v19 model, tub scorecard in the Amendment-
  4 configuration (`--venue tub --rel-alt-bias -0.0326 --accel-lp 1.25`, 4 seeds,
  `scorecard_v19tub*.json`): **41/41 in every seed and pooled** (secondary MWU
  40/40/41/41), identical to the v18 result. Context only, not a hypothesis.
- **2026-11-14 — Amendment 6b: depth clause exercised as a sensitivity set; analysis
  tool corrected.** The line-course confirmatory session took place on 2026-11-13
  (7 laws x 4 repeats, Box folder "2026-11-13 Swimming tub test - LINE"); the
  rectangle session has not. The `start_gate.json` records show the real floating
  DVL altitude was 0.58 m (0.51-0.75) against the twin's 0.81 m, so the floor guard
  capped the real depth target to 0.28 m (0.21-0.40) instead of 0.50 m, and the
  heading error at course start was +4 to +8 deg (twin: ~0). Because line data now
  exists, the re-run foreseen in Amendment 6(b) CANNOT replace the frozen line
  predictions. It is run instead as a pre-specified SENSITIVITY set for both courses
  (campaign `pred_v19_d074`, water depth 0.74 m so the twin's floating altitude
  matches 0.58 m and its guard caps the same way; launched 2026-11-14 before any
  pooled statistic was computed) and will be reported beside the Amendment-6(c)
  predictions, which remain the binding ones for BOTH courses. ANALYSIS TOOL: the
  pre-registered scorer measured every run against the rectangle geometry, which is
  wrong for the line course; it now uses the runner's own cross-track column `e`
  (the quantity the frozen predictions were computed from) when present. No
  statistic or threshold changes. The fix (`score_correlation.py`, this date) was
  made and pushed before any pooled or rectangle statistic was computed; a line-only
  preliminary was computed immediately after and is logged in the work log, not here.
  (addendum, same date) A second tool defect was found and fixed on the same run: a
  stray early `return` left in `report()` by the Amendment-3 edit ended the report
  after the pairwise block, so the Spearman/Pearson/Kendall/MMRV lines and the
  noise-band verdict never printed (`cdaa0b1`). No statistic or threshold changes.
  (result, 2026-11-14) Sensitivity set `pred_v19_d074` (56 runs, all start-ups
  settled): with 0.74 m of water the twin's floating DVL altitude is 0.586 m (real
  0.583), its guard caps the depth target to -0.285 m (real -0.280) and the course
  runs at -0.32 m (real -0.33), i.e. the real protocol is reproduced. Predicted RMSEs
  move by 1-4 mm; Spearman between the frozen (c) orders and the sensitivity orders is
  +0.964 on both courses, the only differences being one near-tie pair per course
  (rect los/smc, 1.3 mm; line alos/los, 0.7 mm). The frozen (c) predictions remain
  binding; the depth difference is reported as a sensitivity, not corrected.
