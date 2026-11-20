# Results and analysis to date (2026-11-20)

Everything found so far, in one place. The pre-registration (`preregistration.md`) is
the binding record of what was predicted and when; this file is the record of what
came out. `PAPER_BRIEF.md` is the writer's guide and has the detail and the sources.
Raw per-run logs and the frozen simulator predictions are under `data/` (if that
directory is not yet present, the copy from the lab server is pending; the runs also
live in the lab's Box archive under the session folder names given below).

## 1. Headline

Under a pre-registered protocol, the digital twin correctly ordered every controller
pair it was confident about: **12 of 12 separable pairs, pooled over two courses and 56
real runs, one-sided binomial p = 0.0002.** Hypothesis H1 (the twin ranks controllers
the way the real vehicle does, beyond chance and within run-to-run noise) is supported.
Pairs the twin predicted as near-ties finished as near-ties in the pool. The twin's two
misses are a five-way near-tie it over-separated on the rectangle, and two boundary
excursions (p, smc) it did not predict, both at the corner where the tether pulls.

## 2. Fidelity scorecard (the twin against raw sensor recordings)

`sim/fidelity_scorecard.py`: 41 checks = 10 channels (3 accel, 3 gyro, 3 DVL, depth)
x 4 metrics (KS, Wasserstein-1, log-spectral distance, increment autocorrelation) + a
windowed MMD, all on sample-to-sample increments, each judged against the real-to-real
spread of the lab's own recordings with a conformal leave-one-out margin (alpha 0.95).

| configuration | result (4 seeds) |
|---|---|
| v18 physics only, no optional flags | 40/41 in every seed (only `accel_x.w1` fails, a real driving-width gap) |
| v18 + measured tub floors, measured depth datum (-0.0326 m), 1.25 s accel low-pass | 41/41 in every seed |
| v19 (DVL payload + measured 12.7 deg nose-down trim), same configuration | 41/41 in every seed (MWU 40/40/41/41) |

Tightest margins: `accel_z.w1` at 0.68 of its margin, `dvl_z.acf` at 0.92. The margin
construction is lenient (n = 6 leave-one-out) and the tub floors and datum were
calibrated on the bags that are scored; both are stated as limitations.

Method findings that stand on their own:
- Criterion calibration on held-out real data (20 splits): Mann-Whitney rejects genuine
  real data 18 % of the time, MWU+FDR 13.5 %, conformal 5.5 % (target 5 %) while still
  detecting white noise (87 %) and time-shuffled real data (5 %, at the false-alarm
  rate, as it should).
- Floor size matters in both directions: the same twin scored 17/31 against a 5-bag
  floor and 6/31 against a 13-bag floor.
- Verdict-rule selection by ROC AUC against a graded ladder of negatives: Wasserstein-1
  + ACF max-z composite wins (mean AUC 0.928). Start-transient trimming rejected.
  Channel swaps fool every method (~0.5 AUC): a stated blind spot.
- Two-level reporting: switching off trajectory servos leaves the pass count unchanged
  (40/41) while continuous distances degrade 25-73 % on the trajectory-sensitive
  channels. The binary verdict answers "within real variability?"; the distance
  answers "which twin is closer?".
- v18 audit: the earlier 39/40-of-41 flip-flop was unseeded Monte-Carlo noise; the
  apparent sim "bobbing" was the engine's own depth-sensor noise (Sigma 0.011) with
  physics at rest at 0.18 mm; the fix was damping (surface heave zeta 0.04 -> 0.90),
  not more flags.

## 3. Pre-registered predictions (frozen, Amendment 6, v19 twin, 4 repeats)

Cross-track RMSE (m); seed spread 0.9-3.7 mm.

| rank | rectangle | | line | |
|---|---|---|---|---|
| 1 | pursuit | 0.1119 | alos | 0.1337 |
| 2 | los | 0.1155 | los | 0.1366 |
| 3 | smc | 0.1159 | ilos | 0.1434 |
| 4 | alos | 0.1163 | fuzzy_lab | 0.1470 |
| 5 | p | 0.1222 | pursuit | 0.1801 |
| 6 | ilos | 0.1331 | smc | 0.2168 |
| 7 | fuzzy_lab | 0.1663 | p | 0.2221 |

Failure-mode prediction (Amendments 2 and 6e): fuzzy_lab is the law most likely to
approach the 1.0 m safety boundary on the rectangle (predicted max |y| 0.92-0.94 m vs
0.73-0.86 m for all others); no law breaches it.

History: v18 freeze (Amendment 4) to v19 re-freeze moved no incumbent pair; Spearman
v18 vs v19 = +0.93 on both courses; attribution runs show the shift is the vehicle
model (trim + payload), not the new start-up or pool depth. The two courses rank the
laws nearly inversely (fuzzy 7th vs 4th, pursuit 1st vs 5th), which is what makes the
pooled pairwise test sharp.

## 4. Shakedown, 2026-10-08 (disclosed, not confirmatory)

One clean run per law, five laws, rectangle, surface-start protocol: ilos 0.085 < los
0.091 < p 0.101 < pursuit 0.118 < fuzzy 0.140. Pursuit/ilos were swapped against the
v18 prediction; fuzzy last on both sides and crossed the fence at 1.03 m (sim 1.02).

Diagnostics that followed (2026-10-23, `--tether FX,FY` in the bridge):
- FALSIFIED: a constant lateral tether pull as the swap mechanism (0.6 N leaves
  pursuit/ilos at baseline).
- FALSIFIED: sensor-noise differential (ranking identical with noise off; the sim's
  absolute-RMSE inflation is dynamics-driven, not noise-driven).
- CONFIRMED: a fuzzy_lab stability cliff at ~0.5 N of lateral pull, the mechanism for
  its intermittent boundary excursions.

## 5. Line session, 2026-11-13

7 laws x 4 repeats, every start-up settled, start-up `--heading saved`. Data:
`data/real_1113_line/<law>_line_r<rep>/` (runner log, start-gate record).

| rank | predicted | real (4 runs) |
|---|---|---|
| 1 | alos 0.134 | alos 0.088 |
| 2 | los 0.137 | ilos 0.109 |
| 3 | ilos 0.143 | los 0.112 |
| 4 | fuzzy_lab 0.147 | pursuit 0.133 |
| 5 | pursuit 0.180 | fuzzy_lab 0.140 |
| 6 | smc 0.217 | smc 0.156 |
| 7 | p 0.222 | p 0.162 |

Line alone: 6/6 separable pairs correct (p = 0.016); Spearman +0.929 (exact chance
p = 0.003), Pearson +0.877, Kendall +0.810, MMRV 0.003 m; inside the noise band
[+0.36, +0.96]. The two adjacent swaps: los/ilos was a predicted 3 mm near-tie and is
a 3 mm near-tie in reality; pursuit/fuzzy (sim 33 mm apart, real 7 mm) is the one real
disagreement and is inseparable on the real side.

Protocol facts from the start-gate records: the floating DVL altitude was 0.58 m (twin
0.81 m), so the floor guard capped the real depth to 0.28 m (0.21-0.40) instead of
0.50 m; heading error at course start was +4 to +8 deg on every run (one-sided);
settle 4-37 s. The autopilot compass heading at the same hand placement wandered from
-81 to -142 deg across the 28 runs against one saved target (-70.4), so the start-up
turned the vehicle 10-71 deg (mean 27) before every run. This led to Amendment 7.

Depth sensitivity set (Amendment 6b, `pred_v19_d074`, water 0.74 m, 56 runs): the twin
reproduces the real start-up exactly (floating altitude 0.586 vs 0.583 m, depth target
-0.285 vs -0.280 m, course depth -0.32 vs -0.33 m); predictions move 1-4 mm; Spearman
vs the frozen set +0.964 on both courses (one near-tie pair per course). The depth
difference explains nothing, including the absolute RMSE gap. Frozen predictions kept.

## 6. Rectangle session, 2026-11-19

7 laws x 4 repeats, every start-up `--heading placed` (Amendment 7 pushed at 13:53,
first run 14:17). Heading error at course start -7.8 to +7.8 deg, mean -0.2, sd 4.4
(two-sided, centred: the placed-heading start did what it was meant to). Settle 5-14 s.
Water deeper than on 11-13: floating altitude 0.67-0.97 m, depth target -0.36 to -0.50.
Data: `data/real_1119_rect/<law>_rect_r<rep>/`.

Three runs tripped the 1.0 m safety box and did not finish, all at the far-left corner
on lap 0: fuzzy_lab r3 (x +1.74, y +1.01; the lab's first attempt at this repeat also
tripped at x +2.17, y +1.02 and was deleted and redone, disclosed in their NOTES.txt),
p r2 (x +1.62, y +1.00), smc r4 (x +1.71, y +1.00). Every law has >= 3 completed
repeats. Handling of tripped runs was not pre-specified; the primary uses the 25
completed runs and both alternatives are reported.

| rank | predicted | real (completed runs) | max y, real (m) |
|---|---|---|---|
| 1 | pursuit 0.112 | smc 0.088 | 0.67-0.75 (r4 tripped) |
| 2 | los 0.116 | pursuit 0.088 | 0.63-0.66 |
| 3 | smc 0.116 | alos 0.089 | 0.67-0.72 |
| 4 | alos 0.116 | ilos 0.091 | 0.70-0.75 |
| 5 | p 0.122 | los 0.091 | 0.66-0.77 |
| 6 | ilos 0.133 | p 0.113 (sd 0.029) | 0.69-0.97 (r2 tripped) |
| 7 | fuzzy_lab 0.166 | fuzzy_lab 0.120 | 0.82-0.94 (r3 tripped, both attempts) |

Rectangle alone (completed runs): 6/6 separable pairs correct; Spearman +0.643 (exact
chance p = 0.069, under the pre-registered 0.679 bar for n = 7), Pearson +0.766,
Kendall +0.524, MMRV 0.0075 m; inside the noise band [+0.21, +0.96]. The rank
correlation is weak because five laws finished within 3 mm of each other in reality
where the twin spread them over 21 mm; the pairwise statistic declares those pairs
inseparable, which is the correct call.

## 7. Pre-registered primary statistic (pooled, both courses)

Command (Amendment 3 form): `sim/score_correlation.py <pred_v19_rect> <real rect view>
--sim2 <pred_v19_line> --real2 <real line view> --csv-a traj.csv --csv-b traj.csv`.
Full outputs: `results_pooled_2026-11-19.txt`.

| real rectangle runs used | separable pairs correct | binomial p |
|---|---|---|
| 25 completed runs (primary) | 12/12 (6 rect + 6 line) | 0.0002 |
| all 28 as logged | 6/6 (rect adds no separable pair; tripped runs inflate three spreads) | 0.016 |
| tripped runs truncated at the fence line | 11/11 | 0.0005 |

H1 supported under every handling. Per the interpretation rule fixed in advance
(above chance and inside the noise band): supported.

## 8. Failure-mode prediction: outcome

- CONFIRMED: fuzzy_lab had the largest lateral excursion among completed runs and
  tripped on both attempts of one repeat.
- FALSIFIED: "no other law breaches". p and smc each breached once (1 of 4 repeats),
  at the same corner, where the twin predicts max y 0.77-0.80 for them.
- Reading, pre-stated in Amendment 6e: a lateral disturbance the twin does not model,
  consistent with the tether and the 0.5 N stability cliff measured in October. Reported
  as a twin limitation; the twin and predictions were not changed.

## 9. Absolute error gap

Real cross-track RMSE is 20-30 % below the twin's on most laws on both courses (p and
fuzzy_lab closer to the twin on the rectangle). Not a noise effect (noise ablation) and
not a depth effect (sensitivity set); the residual is dynamics/estimator. Ranking
validation does not depend on it.

## 10. Protocol findings worth keeping

- Depth: the estimator zeroes depth at the reset on the surface, so the depth target
  is relative to the float point; the floor guard (0.30 m under the DVL) caps it in
  shallow water. Report the achieved depth per run from `start_gate.json`.
- Heading: the autopilot compass wanders tens of degrees between runs for the same
  placement; taking the course heading from the operator's placement removed the
  one-sided start error (+4..+8 deg on 11-13 -> -7.8..+7.8, mean -0.2, on 11-19).
- The two sessions differ in this one protocol detail (heading source); the line data
  are used as collected.

## 11. Limitations (all to be stated)

1. The scorecard validates the replay path (recorded PWMs -> thruster model, 1 s
   command low-pass); the controllers drive through a force-setpoint path with no
   thruster model or lag.
2. No motion capture: dead reckoning vs dead reckoning (matched observable).
3. No slow bias state in the sensor-noise model.
4. Lenient margins at n = 6 bags; tub floors and datum calibrated on the scored bags.
5. Per-channel metrics cannot detect channel swaps.
6. Sway/yaw drag corrections fit from one command level.
7. One vehicle, one pool, seven classical controllers.
8. Shakedown data seen before Amendments 1-4 (disclosed).
9. Tripped-run handling decided after the data (all handlings reported).
10. Lateral tether disturbance is not modelled (the cause of the unpredicted trips).

## 12. Reproducing the numbers

- Real runs: `data/real_1113_line`, `data/real_1119_rect` (runner log `runner.csv`
  with the cross-track column `e`, `start_gate.json`, `init.csv`, lab `NOTES.txt`).
  Bags remain on the lab's archive.
- Frozen predictions: `data/pred_v19_rect`, `data/pred_v19_line` (`traj.csv`,
  `start_gate.json` per repeat).
- Scoring views are symlink trees named `<law>_s<rep>` with `traj.csv` = the runner
  log; `scripts/` has nothing special for this, a two-line shell loop builds them.
- `sim/score_correlation.py` as in Section 7; `sim/power_analysis.py` for the power
  numbers quoted in Amendment 6f.
