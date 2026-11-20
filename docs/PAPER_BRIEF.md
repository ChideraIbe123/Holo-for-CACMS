# Paper brief: BlueROV2 digital twin + pre-registered sim-vs-real controller ranking

For whoever is writing the paper. This is the condensed, current version of
`~/Documents/Ocean/work.md` (1,400+ lines, chronological, written as a lab notebook).
Everything below is current as of 2026-11-20 (Sections 7b, 7c and 9 carry the confirmatory results). Where a number has a source, the
work.md section is named in brackets so you can pull the detail.

**Target venue:** IEEE RA-L, submit ~Nov 20, 2026 (rolling review; a reject can be
re-targeted to IROS 2027, deadline Mar 1).
**Blocking item:** the RECTANGLE confirmatory session (21 runs + recommended 4th repeat).
The line session is done (Section 7b). The pooled primary statistic needs both.

---

## 1. The paper in one paragraph

We built a digital twin of the lab's BlueROV2 in HoloOcean and validated it the way
nobody in underwater simulation does: 41 statistical checks on the raw IMU, DVL and
depth streams, each judged against how much *real recordings differ from each other*
(a real-to-real reference floor), with a pass threshold calibrated so genuine real data
passes 95% of the time. Then we used the twin to predict how seven path-following
controllers rank on two pool courses, froze those predictions publicly (git
timestamps), and test whether the real vehicle ranks them in the same order. The
novelty is that last step: **does the simulator get the controller ORDER right, beyond
chance and within real run-to-run noise**, tested prospectively. No underwater benchmark
does this.

## 2. Claims, and how to word them

Three pillars (strategy locked 2026-10-12 [Paper strategy locked]):

1. **Ranking agreement** (headline, pending data). Pre-registered pairwise-separable
   ordering agreement, pooled across both courses, plus the standard Spearman / Kendall /
   Pearson / MMRV set.
2. **Failure-mode prediction** (pending data; restated in Amendment 6). The twin
   predicts the lab fuzzy controller is the law with by far the largest lateral
   excursion: v19 max |y| 0.92-0.94 m over 4 repeats vs 0.73-0.86 m for all others.
   Under v18 it crossed the 1.0 m fence (1.01-1.03 m); under v19 it stays just inside.
   The Oct 8 shakedown fuzzy run crossed at 1.03 m under the old surface-start
   protocol. Binding v19 prediction: fuzzy approaches the fence, nobody breaches; a
   real breach points to unmodeled lateral disturbance (tether; 0.5 N cliff found
   2026-10-23). Write it as a near-miss prediction, not an over-the-line one.
3. **Diagnose-and-explain arc** for any sim-real disagreement. Already have a clean
   diagnostics story from the shakedown (Section 7): three candidate mechanisms
   falsified, one confirmed.

Scoped novelty wording (from `paper/related_work_table.md`; say each "to our
knowledge" and name the nearest prior art):
- Distributional/spectral/temporal statistical testing of NAVIGATION-sensor streams
  against real recordings (nearest: ACOUSIM, sonar imagery only, fixed thresholds).
- A real-to-real reference floor as the acceptance threshold (no precedent found).
- A conformally calibrated pass criterion for simulator validation (lineage:
  equivalence testing / TOST).
- First underwater instance of prospective, pre-registered sim-vs-real ranking
  validation, on classical controllers, against a repeatability floor.

**Do NOT claim:**
- That ranking-based simulator validation is a new concept. It is not: SRCC (Kadian et
  al., RA-L 2020) and MMRV (SIMPLER, Li et al. 2024) established it. Cite them.
- Breadth. Frame as "methodology + first instantiation" of an H2RBench/SIMPLER-style
  evaluator template: one vehicle, one pool, seven controllers.

## 2b. Closest prior work: H2RBench (our template; read it in full)

Xiao, Zhan, Krishna, Meng, Irshad, Zakharov, Held, "H2RBench: A Real-to-Sim Benchmark
for Evaluating Human-to-Robot Transfer," CoRL 2026, arXiv:2609.24778. This is the
paper our evaluation design follows most closely; the framing is "an underwater
instantiation of an H2RBench-style evaluator."

What they do: 4 human-to-robot learning methods (Phantom, Point Policy, AMPLIFY,
GHOST) x 4 tabletop tasks (pick-and-place, stacking, insertion, sequential) on a
Franka Panda. Sim scenes are reconstructed from the real ones (built on the PolaRiS
pipeline, using Marble + Gaussian splatting) in Isaac Lab. The metric is success rate.
Sim-vs-real agreement: Pearson r = 0.894, Spearman rho = 0.851, MMRV = 0.06.

What we took from it:
- **Pooling across tasks.** Their Appendix D.4 pools Pearson/Spearman over all 16
  method-task pairs because "each individual task contains only four methods, which
  leads to unstable rank-based statistics." That is the rationale of our Amendment 3
  (pool rectangle + line); cite it there.
- **The statistic set.** Pearson + Spearman + MMRV (MMRV per SIMPLER: computed per
  task, then averaged).
- **Per-task breakdown + sensitivity analysis.** They show agreement varies by task
  (stacking is the outlier, MMRV up to 0.171) and that removing one method (AMPLIFY)
  or one task changes the metrics, concluding discrepancies are "concentrated in a
  small number of method-task interactions." We should report the same: per-course
  results plus a leave-one-controller-out check. Our pursuit/ilos shakedown swap and
  its diagnostics are the analog of their AMPLIFY discussion.

Where we go beyond it (the contrast for Related Work):
- **Simulator fidelity is validated, not assumed.** H2RBench uses "the default physics
  configuration from Isaac Lab / Isaac Sim" with minimal friction tuning, and its
  only sim-real check is the policy-level correlation. It lists "imperfect physical
  fidelity" as a limitation. We validate the sensor streams statistically (41
  calibrated checks against a real-to-real floor) BEFORE trusting the ranking.
- **Prospective and pre-registered.** Their correlations are computed after both sides
  exist; ours are frozen and timestamped before the real data.
- **Agreement judged against measured run-to-run noise** (pairwise-separable pairs),
  not raw correlation alone.
- **Different regime:** classical controllers on a dynamics-dominated underwater
  vehicle with continuous RMSE, vs learned manipulation policies scored by success
  rate on a tabletop.

Reporting note: `score_correlation.py` pools courses only for the primary pairwise
statistic; Spearman/Pearson/MMRV print for one course. To report comparably to
H2RBench/SIMPLER, run it per course and average MMRV across the two (reporting only;
the pre-registered analysis is unchanged).

Related evaluators in the same lineage to cite alongside: SIMPLER (Li et al. 2024,
origin of MMRV), PolaRiS (RSS 2026, H2RBench's reconstruction base), REALM, SRCC
(Kadian et al., RA-L 2020).

## 3. The twin (Methods)

**Platform.** HoloOcean 2.3.0 (BYU, Unreal Engine), headless on the lab GPU server.
A bridge (`sim/mavros_bridge.py`) publishes the lab's exact ROS 2 topics
(`/mavros/imu/data`, `/dvl/twist`, `/mavros/global_position/rel_alt`, `/dvl/fom`,
`/dvl/velocity_valid`, plus raw IMU, static pressure, DVL altitude), field-for-field
identical to the real vehicle (frame ids, covariances, rates 10 / ~18 / 3 Hz), so the
lab's unmodified `bluerov_dr` dead-reckoning node runs against it.

**Vehicle dynamics** [Vehicle model; SysID magnitudes]. HoloOcean only ships the
8-thruster Heavy, so we wrote a custom 6-DOF Fossen model of the standard 6-thruster
BlueROV2 (`sim/bluerov2_standard_model.py`); the engine body is just the visual shell.
- Sources: thruster geometry from the community Gazebo model (clydemcqueen/bluerov2_gz);
  rigid body, added mass, damping from Wu 2018 (Flinders thesis, Tables 5.1-5.3); T200
  thrust curve from von Benzon et al. 2022 Eq. 18, plus a measured deadband (0.075).
- Validation on the lab's own bags: surge velocity correlation +0.76 at 1 s horizon
  (+0.85 at 0.5 s); cruise 0.389 m/s at 25% command vs real 0.369 (5%) while full
  throttle stays at the documented ~1.5 m/s; surge generalizes to 4 unseen bags
  (corr +0.55 to +0.77).
- Per-axis steady-state fit from single-axis tub runs: uncorrected model was off by
  surge +16%, heave +24%, sway +88%, yaw -39%; folded in as per-axis drag corrections
  [1.34, 3.55, 1.54, 1, 1, 0.38]. Caveat: one command level, few windows.
- v18 surface fix: Wu's heave damping is a deep-water value (damping ratio 0.04 at the
  surface, a 15 s ring-down); added surface heave damping (ratio 0.90) fading out
  below the surface.

**Pool.** Intex Clearview Prism Frame 26770 (4.0 x 2.0 x 1.22 m; water depth 1.05 m
from the published fill volume), built spec-exact from the manual photographed at
the lab [Indoor pool twin].

**Sensor model and stream quirks** [Sensor noise; Fidelity scorecard; DVL duplicate
stream; v16]. Everything below was discovered in the real data and replicated:
- Noise is AR(1) colored = discrete first-order Gauss-Markov, the navigation field
  standard. Real DVL residual lag-1 autocorrelation ~0.59 is cited by the lab's own
  EKF paper; white noise provably fails the time-structure checks.
- Accelerometer: the 10 Hz topic is actually a ~2 Hz sample-and-hold (81% of
  consecutive samples identical) with 1 milli-g quantization.
- DVL: measures ~9 Hz but every measurement is published twice (exactly 50% duplicate
  samples, ~18 Hz). Raw topic is FRD-signed, mount yaw misalignment -1.848 deg, lever
  arm [-0.171, +0.089, -0.114] m (from the lab's BlueROV-Tools configs).
- Depth: 1 mm quantization; depth-sensor socket sits 0.060 m above the body origin
  (measured, v18).
- Noise decomposition (a finding in its own right): sensor noise measured in static
  floats is 20-100x below the in-mission "noise floor"; ~78% of what looks like noise
  is real motion (vibration, rocking). Armed-but-idle electronics add essentially
  nothing (tub armed float accel_x std 0.006 vs 0.41 in the big pool): the big-pool
  idle noise is water motion. So noise must be **venue-conditioned**, now a `--venue`
  flag.

**Payload and trim (v19, 2026-11-13)** [work.md "v19: DVL payload + MEASURED float
trim"]. The lab's vehicle carries a Water Linked DVL A50 (170 g in air, 105 g in
water) 17 cm aft, 9 cm left and 11 cm below the centre (lab transform file); the
model carries it as a point mass (mass 11.67 kg, inertia +2 to 4.5 %). More important:
the float recordings show the real vehicle rests **12.7 degrees nose-down** (roll 0.5,
steady to 0.02 degrees), independently confirmed by the resting accelerometer
(accel_x = -2.0 m/s^2 = -g sin 12). The model's centre of gravity is set so it floats
at exactly that attitude; consequently 22 % of forward thrust points at the floor,
which is why the real vehicle sinks while driving and why the real protocol needed a
floor guard. This is a good "measured, not assumed" paragraph: the old `accel_x
offset` the scorecard was blind to turned out to be a physical trim. Still assumed:
net buoyancy 0.2 kg (ask for the vehicle's weight in air and its waterline).

**Dead reckoning sanity check** [Verification results]: sim DR drift 3-5% of distance
vs real 3-4% (lab EKF vs the DVL vendor's onboard DR on a real bag: mean 0.24 m over a
7.6 m path).

## 4. The fidelity scorecard (Methods + a Results subsection that is DONE)

`sim/fidelity_scorecard.py`. 41 checks = 10 channels (3 accel, 3 gyro, 3 DVL,
depth) x 4 metrics (KS, Wasserstein-1, log-spectral distance, increment
autocorrelation over lags 1-10) + one windowed MMD.
- All distances are computed on sample-to-sample INCREMENTS, so constant offsets
  (datums, biases) are invisible by construction. State this in Methods.
- Each sim-vs-real distance is judged against the real-vs-real distances (reference
  floor). Pass rule: conformal leave-one-out margin at alpha = 0.95.

**Why conformal, with evidence** [Fidelity round v8-v10, threshold study]: held-out
REAL bags scored as if they were sim, 20 splits.

| criterion | rejects genuine real data (want 5%) | detects white noise | detects time-shuffled real |
|---|---|---|---|
| Mann-Whitney (difference test) | 18% (miscalibrated) | 100% | 20% |
| MWU + FDR | 13.5% | 100% | 17% |
| Conformal (adopted) | 5.5% | 87% | 5% |

Shuffle-blindness of the original metrics is why the ACF metric was added.

**The floor-size finding** (strong methodological point) [Widened floor]: the same twin
scored 17/31 against a 5-bag floor and 6/31 against a 13-bag floor. An under-sampled
floor is wrong in both directions. Fidelity scores are meaningless without a
well-sampled real-to-real floor.

**Verdict-rule selection** [Method-selection study]: 8 candidate methods scored by ROC
AUC against a graded ladder of negatives (including the project's own superseded twin
versions as "nearly right" labeled negatives). Winner: Wasserstein-1 + ACF max-z
composite, mean AUC 0.928. Start-transient trimming was tested and REJECTED (AUC
worse). Limitation found: swapping channels fools every method (~0.5 AUC).

**Final result (v18, 2026-10-27)**, 4 seeds each, matched venue (tub floor, 6 bags):
- Physics only, all optional flags off: **40/41 in every seed** (only fail
  accel_x.w1, consistently, so it is a real gap, not noise).
- With measured tub noise floors, the measured depth datum (-0.0326 m) and a 1.25 s
  low-pass modeling the flight controller's filtered accel reporting: **41/41 in every
  seed**.
- Honest framing (agreed, please keep): quote the per-seed spread and the tightest
  margins, never a bare "41/41". Tightest: accel_z.w1 at 0.68 of its margin,
  dvl_z.acf at 0.92. The margin construction is lenient (n = 6 leave-one-out), and
  the tub floors and datum were calibrated on the same bags that are scored. Say so.
- v19 re-check (2026-11-13): with the DVL payload and the measured 12.7 degree trim in
  the model, the same configuration scores **41/41 in every seed again** (MWU
  40/40/41/41). The trim adds a constant accelerometer offset, which the increment-
  based checks cannot see, and the driving dynamics stayed inside the real-to-real
  floor. Say this: the fidelity result survived a physically motivated model change.
- Score history for the narrative: 14/31 (v1) -> honest 6/31 after widening the floor
  -> 33/41 after calibrated criterion + rotational trajectory matching -> 36/41 (30-bag
  floor) -> 38/41 matched venue after modeling the DVL duplicate stream -> 40/41
  physics-only, 41/41 full (v18).

**Two-level reporting** [Ablation discrimination check]: turning off trajectory servos
leaves the pass count unchanged (40/41 both) but continuous distances degrade 25-73%
exactly on the trajectory-sensitive channels. Binary verdict answers "within real
variability?"; continuous distance answers "which twin is closer?". This defuses the
"you copied the answer" review.

**The v18 audit is a good honesty paragraph** [Simulator audit + v18]: a run of
tuning iterations was flip-flopping between 39 and 40/41. The audit found the runs
were unseeded (score noise straddling margins), the apparent sim "bobbing" was the
engine's own depth-sensor noise, and the fix was physics (damping), not more tuning
flags. After it: seeded, bit-reproducible runs, and the extra correction stack retired.

## 5. The controller benchmark (Methods)

Shared code for sim and real: `real/tracker.py` (no ROS inside the laws),
`real/course_runner.py`, `real/run_one.sh` (real) / `real/sim_run.sh` (sim). The
interface is deliberately derivative-free and velocity-free (the real pose estimate is
bursty with duplicate timestamps).

Seven entrants (n = 7 -> 21 controller pairs):

| law | family | source |
|---|---|---|
| `p` | proportional heading | baseline |
| `pursuit` | pure pursuit | geometric |
| `los` | line-of-sight | Fossen |
| `ilos` | integral LOS | Borhaug et al. 2008 |
| `alos` | adaptive LOS (crab-angle estimation) | Fossen, Pettersen & Galeazzi, IEEE TCST 2015 |
| `smc` | boundary-layer sliding-mode heading | von Benzon et al. 2022, JMSE 10(12):1898, sec 7.1, Table 6 gains |
| `fuzzy_lab` | the lab's fuzzy path tracker | lab |

The first five were written by Sumanth (pilot-controllers branch). `smc` and `alos`
were added 2026-10-28 because the learned controllers (ANFIS-DDPG, PPO) were not
available; chosen from the literature for mechanism diversity (ranking-validation
papers want controllers that span mechanism families). Backstepping and PD were ruled
out because the interface has no velocity; MPC/RL ruled out by implementation time.

Courses: rectangle 2.0 x 0.6 m, two laps (~45 s); straight line 2.4 m starting 0.3 m
off the path (~9 s). Metric: RMSE of cross-track error (von Benzon 2022 metrics).
Matched observable: real vehicle has no ground truth, so the primary comparison is
dead-reckoning vs dead-reckoning (sim ground truth reported as secondary).

**Ranking sensitivity, a motivation table** [Benchmark moved INTO the pool twin;
4-seed benchmarks]: with earlier stand-in controllers, the "aggressive" controller was
1st in open water (0.106 m), LAST in the pool (0.194), and 1st again once measured
drag corrections went in (0.089). Rankings are acutely course- and dynamics-sensitive,
which is exactly why sim rankings need real validation.

## 6. Pre-registration chain (cite commit hashes)

Public repo github.com/ChideraIbe123/Holo-for-CACMS, `docs/preregistration.md`,
append-only. Commit timestamps prove each freeze precedes the data.

| date | commit | what |
|---|---|---|
| 2026-09-30 | `8acc9ec` (v1.0.0) | original freeze, hypothesis H1, interpretation rule |
| 2026-10-10 | `acc5760` | Amendment 1: real entrants (5 laws) + prediction |
| 2026-10-12 | `955f59b` | Amendment 2: pairwise-separable primary stat; fuzzy fence prediction |
| 2026-10-22 | `300f2e1` | Amendment 3: pooled across both courses; Pearson added |
| 2026-10-28 | `c4debd2` | Amendment 4: 7 entrants, v18 re-freeze, n = 7 power, 42-run plan |
| 2026-11-12 | `f218716` | Amendment 5: real runs start at depth and still, like the sim (dive gate) |
| 2026-11-13 | `5ba54ff` | Amendment 6: v19 vehicle (DVL payload + measured 12.7° trim), real start-up reproduced in the twin, predictions re-frozen |
| 2026-11-14 | `257c850`.. `0730d34` | Amendment 6b: real floor height, depth sensitivity set, scorer fixes (predictions unchanged) |
| 2026-11-19 | `8344670` | Amendment 7: course heading from the operator's placement, not the saved compass heading (protocol only, before any rectangle data) |

**Binding prediction (Amendment 6, v19 twin, 4 repeats, RMSE m):**

| rank | rectangle | | line | |
|---|---|---|---|---|
| 1 | pursuit | 0.1119 | alos | 0.1337 |
| 2 | los | 0.1155 | los | 0.1366 |
| 3 | smc | 0.1159 | ilos | 0.1434 |
| 4 | alos | 0.1163 | fuzzy_lab | 0.1470 |
| 5 | p | 0.1222 | pursuit | 0.1801 |
| 6 | ilos | 0.1331 | smc | 0.2168 |
| 7 | fuzzy_lab | 0.1663 | p | 0.2221 |

Superseded Amendment-4 (v18) table, for the "prediction history" figure: rect pursuit
0.143 < los 0.150 < alos 0.159 < smc 0.168 < ilos 0.171 < p 0.177 < fuzzy 0.218; line los
0.134 < alos 0.143 < fuzzy 0.147 < ilos 0.151 < pursuit 0.178 < smc 0.217 < p 0.224.
Spearman v18 vs v19 = +0.93 on both courses. The v19 re-fit was driven entirely by
measurements (float attitude, DVL datasheet, lab mount), and an attribution set (v19
model on the old protocol) shows the shift is the vehicle model, not the new start-up
or pool depth. Absolute rect RMSE fell 20-25 %, toward the real October values, with
unchanged lap times.

Seed std 0.9-3.7 mm. Wording caution (found 2026-11-12): `--seed` fixes the sensor
noise streams but NOT the closed loop, because ROS wall-clock pacing changes which
estimator messages the controller acts on; two runs with identical config and seed
differed by 4.5 mm RMSE. So call these "4 repeats", not "4 deterministic seeds", and
describe the spread as the simulator's own run-to-run floor (which is exactly what the
pairwise-separability statistic needs on the sim side). Points worth making: the v18 twin changes moved NO incumbent pair
(the re-freeze confirmed rather than revised); the two courses rank nearly inversely
(fuzzy 7th vs 3rd, pursuit 1st vs 5th), which makes the pooled pairwise test sharper;
the new laws behave as their mechanisms predict (smc, like p, has no cross-track
feedback and drops to near-last on the offset line). Near-tie pairs the separability
stat will handle: smc-ilos rect (3.1 mm), alos-fuzzy line (3.7 mm).

Disclosures that must appear: the original freeze says any twin change voids it;
Amendment 4 is the documented re-freeze under v18. A shakedown session (Oct 8) was
seen before Amendments 1-4; disclosed in each.

**Statistics plan** (`sim/score_correlation.py`, `sim/power_analysis.py`):
- PRIMARY: of controller pairs whose RMSE gap exceeds the summed repeat spreads on both
  sides, the fraction the sim orders correctly, pooled across courses (pooling
  rationale from H2RBench, Section 2b), one-sided binomial test vs chance.
- Also: Spearman rho with exact-permutation p, Kendall tau, Pearson r, MMRV (SIMPLER
  convention), and a Monte-Carlo noise-floor band.
- Chance threshold at n = 7: rho >= 0.679 (alpha 0.05). With the v19 spreads (the
  rectangle's top four are within 4 mm): rectangle-only power at 3 repeats is
  0.98/0.83/0.55 for 5/10/20 mm real spread, 4 repeats 0.99/0.88/0.61; line >= 0.96.
  The primary statistic pools both courses. Session: 3 repeats each course, 4th
  rectangle repeat strongly recommended.
- Interpretation rule fixed in advance: above chance and inside the noise band = H1
  supported; below the band = rejected; otherwise inconclusive, reported as such.

## 7. Preliminary real data: Oct 8 shakedown (disclose; not confirmatory)

One clean run per law, rectangle only [FIRST SIM-VS-REAL RANKING COMPARISON; Tether
diagnostics].
- Real: ilos 0.085 < los 0.091 < p 0.101 < pursuit 0.118 < fuzzy 0.140.
- Sim:  pursuit < los < ilos < p < fuzzy.
- rho = +0.30, chance p = 0.34: inconclusive, exactly as the power analysis predicts
  for 1 repeat. Agrees: fuzzy last on both sides (and its fence crossing), los 2nd on
  both. Disagrees: pursuit and ilos swap.
- Diagnosing the swap (all in sim, each a controlled A/B):
  - constant tether pull: FALSIFIED (0.6 N leaves pursuit and ilos unchanged);
  - sensor-noise differences: FALSIFIED (noise off gives an identical ranking, RMSEs
    within 1-2 mm);
  - control-loop rate (sim 18 Hz vs real ~10.5 Hz): FALSIFIED (rate-matched ranking
    identical, max shift 4 mm);
  - CONFIRMED: fuzzy falls off a stability cliff at ~0.5 N lateral pull (the sim
    mechanism for its intermittent real fence crash).
  These double as a robustness paragraph: the frozen prediction is invariant to 2x
  loop rate, noise on/off, and sub-cliff tether pull. Leading explanation for the swap:
  single-run noise; the confirmatory session decides.
- Sim absolute RMSEs (0.14-0.23 m) are higher than real (0.05-0.13 m). Shown NOT to
  be noise (noise-off changes nothing); it is trajectory/dynamics driven (real laps
  are faster: p-law 33.2 s real vs 40.5 s sim). Ranking validation does not need
  absolute agreement, but say it.

## 7b. Line-course confirmatory session (2026-11-13): preliminary read

Collected per protocol: 7 laws x 4 repeats, Sumanth's start-up before every run, all
28 start-ups settled. Raw runs: Box "2026-11-13 Swimming tub test - LINE"; on the lab
server `~/data/real_1113_line` (bags not yet copied). THE PRE-REGISTERED PRIMARY
STATISTIC IS POOLED ACROSS BOTH COURSES and waits for the rectangle session; the
numbers below are the line course alone, run with the pre-registered script after two
tool defects were fixed (recorded in Amendment 6b).

| rank | predicted (v19, frozen) | real (4 runs) |
|---|---|---|
| 1 | alos 0.134 | alos 0.088 |
| 2 | los 0.137 | ilos 0.109 |
| 3 | ilos 0.143 | los 0.112 |
| 4 | fuzzy_lab 0.147 | pursuit 0.133 |
| 5 | pursuit 0.180 | fuzzy_lab 0.140 |
| 6 | smc 0.217 | smc 0.156 |
| 7 | p 0.222 | p 0.162 |

- Pairwise (primary form, line only): 6 of 6 separable pairs correctly ordered,
  binomial p = 0.016. Spearman +0.93, Pearson +0.88, Kendall +0.81, MMRV 0.003;
  exact chance p = 0.003; inside the perfect-agreement noise band [+0.36, +0.96].
  On this course alone, H1 is supported.
- The two adjacent swaps: los/ilos was predicted as a 3 mm near-tie and is a 3 mm
  near-tie in reality; pursuit/fuzzy is the one real disagreement (sim 33 mm apart,
  real 7 mm, overlapping ranges, not separable on the real side).
- Real absolute RMSE is ~30 % below the sim on every controller (same direction as
  the October rectangle shakedown). Report; ranking validation does not need it.
- Protocol facts from `start_gate.json` for Methods/Limitations: the floor guard
  capped the real depth to 0.28 m (0.21-0.40) because the floating DVL altitude was
  0.58 m, not the 0.81 m the 38-in assumption gave the twin; heading error at course
  start +4 to +8 deg (a steady push; twin ~0); settle 4-37 s. HEADING DRIFT (for
  Methods and Limitations, Amendment 7): the autopilot heading read at placement
  ranged -81 to -142 deg against one saved target (-70.4) across the 28 line runs,
  i.e. the compass wandered ~60 deg between runs for the same hand placement, and the
  start-up turned the vehicle 10-71 deg (mean 27) before every run. Within a run the
  heading was stable (results and ranking sane); the line course is defined in the
  vehicle's own frame so the data stand, but the runs set off at different physical
  angles across the pool. For the rectangle session the course heading is taken from
  the operator's placement instead (`--heading placed`, 2026-11-19): no turn, aim by
  sighting the long wall. Say in the paper that the two sessions differ in this one
  protocol detail and why. A pre-specified
  sensitivity set at the matching water depth (Amendment 6b, `pred_v19_d074`, done
  2026-11-14) reproduces the real start-up exactly (floating altitude 0.586 vs 0.583 m,
  depth target -0.285 vs -0.280 m) and moves the predictions by 1-4 mm only (Spearman
  +0.96 vs the frozen set on both courses; one near-tie pair swaps per course). So the
  depth difference does not explain anything, including the ~30 % absolute gap; say
  so in one sentence and keep the frozen predictions as the binding ones.

## 7c. Rectangle confirmatory session (2026-11-19) and the PRE-REGISTERED PRIMARY RESULT

Collected per protocol: 7 laws x 4 repeats, `--heading placed` on every run (Amendment 7
pushed 13:53, first run 14:17; heading error at course start -7.8..+7.8 deg, mean -0.2,
vs +4..+8 one-sided on the line day). Box "2026-11-19 Swimming tub test - RECTANGLE";
server `~/data/real_1119_rect` (no bags). Three runs tripped the 1.0 m safety box at the
same far-left corner and did not finish: fuzzy_lab r3 (both attempts; the first was
deleted by the lab and redone, disclosed in their NOTES.txt), p r2, smc r4. Every law has
>= 3 completed repeats. Handling of tripped runs was NOT pre-specified; the result entry
in `docs/preregistration.md` says so and reports all three handlings. Primary = the 25
completed runs.

| rank | predicted (v19, frozen) | real (completed runs) |
|---|---|---|
| 1 | pursuit 0.112 | smc 0.088 |
| 2 | los 0.116 | pursuit 0.088 |
| 3 | smc 0.116 | alos 0.089 |
| 4 | alos 0.116 | ilos 0.091 |
| 5 | p 0.122 | los 0.091 |
| 6 | ilos 0.133 | p 0.113 (sd 0.029; one near-miss at 0.97 m) |
| 7 | fuzzy_lab 0.166 | fuzzy_lab 0.120 |

- **HEADLINE (pre-registered primary, pooled rect + line, Amendments 2-3): 12 of 12
  separable pairs correctly ordered, binomial p = 0.0002. H1 supported.** With all 28
  runs as logged: 6/6, p = 0.016 (the rectangle adds no separable pair because the
  tripped runs inflate three spreads); tripped runs truncated at the fence: 11/11,
  p = 0.0005. The conclusion does not depend on the handling.
- Secondary, rectangle alone: Spearman +0.643 (exact chance p = 0.069, under the 0.679
  bar), Pearson +0.766, Kendall +0.524, MMRV 0.0075 m, inside the noise band
  [+0.21, +0.96]. Line alone stays Spearman +0.929 (p = 0.003). Write the rectangle
  honestly: reality tied five laws within 3 mm where the twin spread them over 21 mm;
  a rank correlation cannot resolve a five-way tie, and the pairwise statistic
  correctly declares those pairs inseparable. This is exactly why the primary was
  pre-registered as pairwise-separable and pooled (H2RBench/SIMPLER precedent).
- Both ends of the rectangle ranking are right: fuzzy_lab last on both sides, p sixth
  on both sides, pursuit top-two on both sides.
- Failure-mode prediction (Amendments 2, 6e): half confirmed, half falsified. fuzzy_lab
  did have the largest excursion (0.82-0.94 m among completed runs) and tripped both
  attempts of one repeat. But p and smc each tripped once (sim max y for them 0.77-0.80).
  All trips at the same corner, same leg, lap 0. Amendment 6e pre-stated the reading: a
  lateral disturbance the twin does not model (tether; fuzzy_lab's 0.5 N stability
  cliff). Report as a limitation; twin and predictions were NOT touched.
- Absolute RMSE: real 20-25 % below the twin for five laws (same as line and shakedown);
  p and fuzzy_lab closer to the twin.
- Scorer outputs for all handlings: `docs/results_pooled_2026-11-19.txt` in the repo.

**Verdict and how to frame it (2026-11-19).** The outcome is GOOD: the one statistic
committed to before any real data came out perfect (12/12, p = 0.0002), so the paper
is written on the "H1 supported" branch of the pre-registration.

- Why it is credible, not lucky: the result survives all three handlings of the tripped
  runs; the two courses agree with each other (line 6/6, rect 6/6); the pre-registration
  chain (Section 6) shows the predictions were frozen and pushed before each session,
  so a reviewer cannot attribute it to tuning.
- The places the twin was WRONG are the paper's best Discussion material, and they are
  the kind a reviewer expects and respects. (1) It could not tell apart five controllers
  that are genuinely within 3 mm of each other in the pool; it spread them over 21 mm.
  (2) It did not foresee the p and smc fence trips at the corner where the tether
  pulls. (3) It over-predicts absolute error by 20-25 %. Present all three as findings
  about the twin, not threats to the claim.
- What is NOT perfect and must be said plainly: the rectangle Spearman alone (+0.64) is
  under the 0.68 chance bar; on its own that course would read "inconclusive". Say it
  in Results, next to the pooled number, and explain the five-way tie. Do not let the
  pooled statistic hide it.
- What it means for scope: nothing is left to run. The paper has a yes answer to its
  question, 56 confirmatory runs over two courses, a fidelity scorecard (41/41) and a
  failure-mode prediction that was half right. The remaining work is writing.
- One-sentence version for the abstract: "Under a pre-registered protocol, the twin
  correctly ordered every separable controller pair (12/12, p = 0.0002) across two
  courses and 56 real runs, while its two misses, a five-way near-tie it over-separated
  and two unpredicted boundary excursions, localise the unmodelled physics to lateral
  tether disturbance."

## 8. Limitations (collected; all known, all should be in the paper)

1. **Validated path vs used path.** The scorecard validates the replay path (recorded
   thruster PWMs -> thruster model, 1 s command low-pass; sim heave lags real by
   0.6-1.0 s). The controllers drive through a force-setpoint path with no thruster
   model or lag. Write this explicitly.
2. No motion-capture ground truth; comparisons are dead-reckoning vs dead-reckoning
   (Marinarium has mocap, a genuine facility advantage; acknowledge).
3. No slow bias state in the noise model (bias time constants are minutes; runs are
   30-120 s). Cite the deep-sea simulation paper (arXiv 2608.26888) on white-noise DVL
   underestimating drift.
4. Scorecard margins are lenient at n = 6 bags; tub floors and datum calibrated on the
   scored bags.
5. Per-channel metrics cannot detect channel swaps.
6. Sway/yaw drag corrections fit from one command level.
7. One vehicle, one pool, classical controllers only (learned controllers unavailable).
8. Shakedown data seen before the amendments (disclosed).
9. Sim absolute RMSE higher than real (Section 7).

## 9. What still has to happen

1. DONE 2026-11-13 and 2026-11-19: both confirmatory sessions, 7 laws x 4 repeats each
   (56 runs, 3 rectangle fence trips). Section 7c has the result.
2. DONE 2026-11-19: analysis run exactly as pre-registered (pooled `--sim2/--real2`,
   `traj.csv` = runner log on both sides). Outcome: H1 SUPPORTED (12/12, p = 0.0002).
3. Write the Results/Discussion on the "supported" branch, with the rectangle Spearman
   caveat and the two unpredicted fence trips (p, smc) stated plainly.
3b. Optional: pull the 2026-11-13 and 2026-11-19 bags from Box to the server and run the
   fidelity scorecard on them as a held-out check (protocol item 5); not needed for H1.
4. Optional presentation upgrade: formal Allan variance (IEEE Std 952) on the float
   recordings, so the noise section speaks IMU-community vocabulary.

## 10. Figures and assets

Exists: `media/indoor_pool_chase.png` (twin render); `pool_view/decision_view.mp4` +
`decision_frame.png` (camera + truth-vs-DR overhead + controller telemetry);
`pool_view/chase_still_*.png`. Stale: `media/benchmark_ranking_5laws.png` (5 laws,
pre-v18; regenerate for 7). To make: scorecard per-check margin plot (per-seed spread),
floor-size figure (17/31 vs 6/31), threshold-calibration table, prediction table vs
real with error bars, the fence-excursion figure. Raw data for all of these is on the
lab GPU server under `~/data/` (scorecard_tub4lp*.json, pred_v18_{rect,line},
real_1008_clean); ask Chidera for access.

## 11. Where everything lives

- `~/Documents/Ocean/work.md`: full chronological log (sections named in brackets
  above). **The HANDOFF and "Current state" blocks at its top are from Sept 16 and
  out of date** (they say 13/31 and "stand-in controllers"). Same for the "Next steps"
  list in the middle. Trust the dated sections, latest wins.
- `docs/preregistration.md` in the public repo: authoritative pre-registration. (The
  copy in `~/Documents/Ocean/paper/` is an older snapshot; ignore it.)
- `paper/related_work_table.md`: positioning matrix + exact novelty wording.
- `reports/Underwater sim fidelity landscape.md` + `research_notes/`: cited literature
  review behind the related-work section (every claim has a link).
- `paper/ral_draft/references.bib`: 10 entries started (von Benzon, HoloOcean, SRCC,
  SIMPLER, Wu thesis, Fossen handbook, Borhaug ILOS, Robinson TOST, UCL survey,
  ACOUSIM). Still to add: H2RBench (Xiao et al., CoRL 2026, arXiv:2609.24778; most important),
  Fossen TCST 2015 (ALOS), PolaRiS, Marinarium,
  arXiv 2607.10597 and 2608.26888 (Gauss-Markov error shape), IEEE Std 952.
- `docs/real_ranking_protocol.md`, `real/CONTROLLER_RUN_INSTRUCTIONS.md`: the session
  procedure (Methods: experimental protocol).
- `INSTRUCTIONS.md` in the repo: onboarding to the code.
