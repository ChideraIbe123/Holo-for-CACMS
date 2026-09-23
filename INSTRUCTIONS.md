# Building on the BlueROV2 Digital Twin — Team Instructions

For lab teammates picking this up (written with the controller work in mind).
The [README](README.md) is the general reference — setup, validation summary,
file map. THIS file is the "how do I actually build on it" guide.

## What you're inheriting, in one paragraph

A validated HoloOcean twin of our BlueROV2: it publishes the vehicle's exact
ROS 2 topics (IMU / DVL / depth, field-for-field identical, statistically
indistinguishable from real recordings on 38 of 41 calibrated checks), runs our
unmodified `bluerov_dr` dead-reckoning on them, swims in a spec-exact copy of
the lab's 4×2 m Intex pool, and has a controller-ranking benchmark (von Benzon
2022 metrics) with ground truth that reality can't provide. The missing piece
is YOUR controllers — the whole point is ranking the lab's real controllers in
sim, then checking the ranking against a real pool session.

## First 15 minutes

1. Follow **Setup** in the README (HoloOcean needs the Epic-linked GitHub
   account; python venv must see ROS 2).
2. Sanity check — vehicle swims the pool and publishes real-format topics:
   ```bash
   source /opt/ros/humble/setup.bash && cd sim
   PY=~/holoocean-env/bin/python        # your venv
   $PY mavros_bridge.py --headless --move --pool intex --duration 60
   ros2 topic echo /mavros/imu/data     # another terminal
   ```
3. Full closed-loop check (sim → sensors → dead_reckon → controller → sim):
   ```bash
   bash scripts/run_closedloop_test.sh
   ```
   Needs `bluerov_dr` cloned next door; the script header says where.

## The mental model

```
                    mavros_bridge.py  (HoloOcean + physics + sensor emulation)
                          │ publishes the vehicle's exact topics
      /mavros/imu/data  /dvl/twist  /mavros/global_position/rel_alt  ...
                          ▼
                    bluerov_dr dead_reckon        (the lab's code, unmodified)
                          │ /deadreckon/odom      (the robot's belief)
                          ▼
                    YOUR CONTROLLER               (the part you add)
                          │ /cmd_vel              (TwistStamped velocity setpoint)
                          ▼
                    bridge --control  (inner velocity loop → thrusters → physics)
```

Ground truth is on `/holoocean/ground_truth` (sim-only). Controllers must
steer from `/deadreckon/odom`, never from truth — same information diet as the
real vehicle.

## Plugging in a controller (the main event)

You do NOT write ROS code. `sim/lab_controller_adapter.py` owns topics, the
course, depth hold, and sign conventions. You implement one class:

```python
# my_anfis.py  (anywhere on PYTHONPATH; weights file next to it)
class AnfisController:
    cruise = 0.3                      # m/s, optional (else --cruise)

    def __init__(self):
        self.net = load_my_weights("anfis_weights.pt")

    def reset(self):                  # optional, called once at start
        pass

    def compute(self, obs) -> float:  # return yaw-rate, + = turn left (CCW)
        return float(self.net(obs.dist_error, obs.theta_far, obs.theta_near))
```

`obs` carries the anfis_rl-style state — `dist_error` (signed cross-track, m,
+ = left of path), `theta_near` (heading error to path tangent), `theta_far`
(heading error to current waypoint), plus raw `x, y, yaw, t`. Full contract in
the adapter's docstring.

Run it:
```bash
$PY lab_controller_adapter.py --law my_anfis:AnfisController
```

**Yaw sign:** the adapter applies `--yaw-sign -1` by default (the vehicle's
yaw convention is inverted — measured, confirmed in our own EKF source). If
your controller was trained ON the real vehicle it already speaks vehicle
convention → run it with `--yaw-sign 1`. Decide per controller with a bench
test before trusting any results.

**Add it to the benchmark:** edit `CONTROLLERS` in
`scripts/run_benchmark_intex.sh` (swap the `waypoint_controller.py` line for
the adapter invocation), then:
```bash
bash scripts/run_benchmark_intex.sh     # ~2.5 min per controller per seed
```
Ranking table + figure land in the output dir. Use ≥4 seeds for anything you
intend to quote.

## The standard workflows

| I want to… | run |
|---|---|
| rank controllers in the pool twin | `scripts/run_benchmark_intex.sh` |
| score sim realism vs real recordings | `scripts/run_tub_campaign.sh` (fidelity scorecard, conformal criterion) |
| re-derive the verdict rule / thresholds | `sim/method_selection_study.py` |
| watch a run (camera + truth-vs-DR overhead + live controller decisions) | bridge `--capture DIR` + `waypoint_controller.py --log dec.csv` + `sim_traj_recorder.py`, then `sim/compose_decision_video.py RUN_DIR out.mp4` |
| pool footage / containment check | `sim/indoor_pool_capture.py out 100` |
| sim-vs-real ranking correlation (the paper number) | `sim/score_correlation.py <sim_dir> <real_dir>` |

## Things that WILL bite you (read once, save hours)

- **One bridge at a time.** Concurrent bridges publish to the same topics and
  poison every consumer. Check `pgrep -f "[m]avros_bridge"` first — and note
  the bracket: a plain pgrep matches your own ssh/script text.
- **Run campaigns detached** (`nohup … &` on the sim machine). A dropped ssh
  kills foreground runs; it has eaten whole campaigns.
- **Never judge a change by eye.** Every physics/noise edit → rerun the
  campaign → scorecard decides. Sign-gate scripts abort before wasting an hour
  if a convention is wrong — keep that pattern.
- **Conventions** (all measured, all already handled — don't "fix" them):
  IMU gravity sign-flip; acceleration control scheme is engine index 2; DVL
  wire format is FRD with a −1.848° mount yaw and a lever arm; yaw command
  convention inverted (MAVLink CW+ vs ROS CCW+); this vehicle's motor
  directions live in ESC config so logged PWMs are direction-less; the DVL
  stream publishes each measurement twice. Details + provenance: README
  "Conventions & gotchas" and constants' comments in `sim/mavros_bridge.py`.
- **The physics engine sleeps slow bodies** and then ignores forces; keep the
  teleport watchdog if you write new control loops.
- **The course is 2.0 × 0.6 m** (`WAYPOINTS` in `sim/waypoint_controller.py`,
  `SQUARE` in `sim/score_ranking.py`). It's the largest course that fits the
  real pool with clearance. Change it in BOTH places or nowhere — sim and the
  real session must run the identical course.
- Keep server hostnames / netids out of this public repo.

## Data (ask a teammate for access paths)

Real recordings live in the lab Box ("Deliverables-underwater"); working
copies + campaign outputs on the lab GPU server under `~/data/` (real bags as
npz in `real_npz*`/`tubtest_npz`, campaign runs in `mc_runs*`, scorecards as
`scorecard*.json`). `tools/bag_to_npz.py` converts a new bag; the fidelity
scorecard consumes npz.

## What the project needs from you specifically

1. **The controllers**: ANFIS-DDPG / fuzzy / PPO code + trained weights, and
   which versions are current (is `anfis_rl` the ANFIS base?). Each becomes
   one adapter class as above.
2. Then: sim benchmark with the real controllers (≥4 seeds) → freeze that
   ranking (pre-registration) → **real pool session** per
   [docs/real_ranking_protocol.md](docs/real_ranking_protocol.md) (≥3 repeats
   per controller) → `score_correlation.py` → the paper's headline number.
3. Two-minute favors when near the vehicle: QGroundControl screenshots of
   `MOT_*_DIRECTION` + `AHRS_ORIENTATION`; re-upload the two truncated Box
   bags (Pipeline `15_28_11`, River Test 1 `15_39_49`); log `rc/out` at
   ≥10 Hz in future recordings.

Questions: git blame is thorough — every constant's comment says where its
value came from and what run verified it.
