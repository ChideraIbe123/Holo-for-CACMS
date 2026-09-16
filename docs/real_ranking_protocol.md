# Real-Vehicle Controller-Ranking Session — Protocol

**Purpose.** Produce the real half of the paper's headline experiment: run the same
controllers on the real BlueROV2 that we ranked in sim, on the same course, score them with
the same metrics, and compute the sim-vs-real rank correlation (`score_correlation.py`).
The claim under test: *the sim ranks controllers in the same order reality does, within
run-to-run noise.*

## Prerequisites (before water time)

1. **Controllers as ROS 2 drop-ins.** Each controller must subscribe `/deadreckon/odom`
   (nav_msgs/Odometry) and publish `/cmd_vel` (geometry_msgs/TwistStamped). Lab laws
   (ANFIS-DDPG / fuzzy / PPO) plug into `lab_controller_adapter.py` by implementing one
   `compute(obs) -> yaw_rate` class — no ROS code needed (see the adapter docstring).
   The adapter applies the yaw-sign convention (default −1); a law that already outputs
   vehicle-convention yaw-rate should run with `--yaw-sign 1`. **Decide per controller
   BEFORE the session** (bench-test: positive commanded yaw-rate should turn the vehicle
   left/CCW viewed from above).
2. **Dry run in sim first.** Every controller that will get water time must first complete
   the sim benchmark (`run_benchmark.sh` with the adapter in place of
   `waypoint_controller.py`). The sim ranking must be frozen and archived BEFORE the real
   session (pre-registration — avoids any appearance of tuning the sim to match).
3. Vehicle checks: DVL lock in the pool, `dead_reckon` running (same
   `st_car_ekf_deployable.json` policy as the bags), depth sensor zeroed at surface.

## Course

- **3×3 m square**, counter-clockwise, waypoints (0,0)→(3,0)→(3,3)→(0,3)→(0,0), matching
  `WAYPOINTS` in `waypoint_controller.py` and `SQUARE` in `score_ranking.py`. Shrink to
  2×2 m ONLY if pool clearance requires it — then change both constants everywhere (sim
  reruns included) so sim and real stay identical.
- Start **ON the path** at (0,0), pointing along +x (the first leg). The origin is wherever
  the vehicle is when its controller starts — the course is relative, no pool survey needed.
- Depth: constant target (sim uses −0.6 m; use the pool's safe equivalent), held by the
  vehicle's depth controller. The ranking is planar.
- Duration ≥ 90 s per run (≈ sim `DUR`; multiple laps).

## Per-run procedure

For each controller × repeat (script this order; RANDOMIZE controller order across repeats
to decorrelate battery drain from controller identity):

1. Position vehicle at the start corner, heading down the first leg, at target depth.
2. Publish `/deadreckon/reset` (std_msgs/Empty) **at the surface datum procedure the lab
   normally uses** so the EKF depth datum is aligned (work.md: st_car_ekf zeroes its depth
   datum at startup).
3. Start recording (both, belt and suspenders):
   - `ros2 bag record /mavros/imu/data /dvl/twist /mavros/global_position/rel_alt
     /dvl/fom /dvl/velocity_valid /deadreckon/odom /cmd_vel /mavros/rc/out
     /dvl/dead_reckoning/pose -o <controller>_s<rep>_bag`
   - `python3 sim_traj_recorder.py real_session/<controller>_s<rep>` — records
     `/deadreckon/odom` → `sim_dr.csv` live (the ground-truth topic simply stays empty on
     the real vehicle; same tool as sim, same CSV format).
4. Start the controller (adapter or lab node). Let it run the full duration. Ctrl-C the
   recorder (it saves on SIGINT).
5. Log: battery voltage, tether behavior, any pilot intervention (a run with intervention
   is VOID — rerun it).

**Repeats: ≥ 2 per controller (3 preferred).** The repeat-to-repeat spread IS the real-side
reference floor; with 1 run per controller the floor is unmeasurable and the correlation
verdict loses its error bar.

## Scoring (same code both sides)

```bash
# real-side ranking (DR trajectory vs commanded square):
python3 score_ranking.py real_session/            # uses sim_gt.csv|— falls back per run
# sim-vs-real rank correlation — the headline number. Matched observable (DR vs DR):
python3 score_correlation.py ~/data/benchmark real_session --csv-a sim_dr.csv --csv-b sim_dr.csv
# secondary (sim ground truth vs real DR):
python3 score_correlation.py ~/data/benchmark real_session --csv-b sim_dr.csv
```

**Why DR-vs-DR is primary:** the real vehicle has no ground truth; its trajectory is only
observable through dead reckoning (3–4% of distance vs vendor DR, per the replay baseline).
Scoring BOTH sides on the DR estimate compares like with like; the sim gt-vs-DR gap (~5%)
is reported separately as the estimator's contribution.

Outputs: Spearman ρ + Kendall τ, exact chance p-value, and the noise-floor band (Monte
Carlo under "perfect agreement + measured repeat spreads"). Verdict logic: ρ inside the
band ⇒ sim ordering consistent with reality within noise; below it ⇒ genuine mis-ordering.

## Bring-home checklist

- [ ] `real_session/<controller>_s<rep>/sim_dr.csv` for every valid run
- [ ] all rosbags (backup + future fidelity-floor material — these are also new DR bags)
- [ ] run log (order, battery, interventions)
- [ ] QGC `MOT_*_DIRECTION` param screenshot (2-min side quest; formally closes the
      yaw-sign finding)
- [ ] pool model/SKU + measured fill depth (for the spec-exact pool twin)
