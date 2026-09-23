# Running the BlueROV2 Digital Twin — Operations Guide

How to run everything in this repo, end to end. The [README](README.md) covers
what the twin is and how it was validated; this file is purely "type this, get
that."

## Setup (once)

Follow **Setup** in the README (HoloOcean needs an Epic-linked GitHub account;
the python venv must be created with `--system-site-packages` so it sees ROS 2).
Everything below assumes:

```bash
source /opt/ros/humble/setup.bash
cd sim
PY=~/holoocean-env/bin/python          # your venv's python
```

## The mental model (30 seconds)

```
   mavros_bridge.py  — runs HoloOcean + physics, publishes the vehicle's exact topics
        │   /mavros/imu/data   /dvl/twist   /mavros/global_position/rel_alt ...
        ▼
   bluerov_dr dead_reckon  — the lab's estimator, unmodified
        │   /deadreckon/odom          (the robot's belief)
        ▼
   a controller (waypoint_controller.py)  — steers from the belief
        │   /cmd_vel                  (velocity setpoint)
        ▼
   bridge --control  — inner velocity loop → thrusters → physics
```

Ground truth is published on `/holoocean/ground_truth` (sim-only). Controllers
read `/deadreckon/odom`, never truth — same information the real vehicle has.

## 1. Just run the vehicle

```bash
# swims a scripted path in the Intex pool twin, publishing real-format topics
$PY mavros_bridge.py --headless --move --pool intex --duration 60

# watch the data (another terminal):
ros2 topic echo /mavros/imu/data
```

Useful flags: `--pool intex` (spawn in the pool twin) · `--no-noise` (clean
data) · `--replay prof.npz` (replay a real recording's commands + measured
rates + depth) · `--control` (listen on /cmd_vel) · `--capture DIR` (save
chase-camera frames) · `--duration N`.

## 2. End-to-end verification suites

```bash
bash scripts/run_holoocean_verify.sh    # bridge + topic checks + dead_reckon (simple mode)
bash scripts/run_ekf_verify.sh          # same, EKF mode (deployed vehicle config)
bash scripts/run_closedloop_test.sh     # full loop: sensors → DR → controller → sim
bash scripts/run_replay_baseline.sh     # replay a real bag, compare DR vs vendor estimate
```

Each script's header says what it needs (e.g. `bluerov_dr` cloned next door)
and where results land.

## 3. Controller-ranking benchmark (in the pool twin)

```bash
bash scripts/run_benchmark_intex.sh
# then rank:
$PY score_ranking.py <output_dir>       # table + ranking.png
```

6 reference controllers × seeds, closed-loop on DR feedback, ranked by the von
Benzon RMSE/IAE metrics against the seed-to-seed noise floor. ~2.5 min per run.

## 4. Fidelity scoring (how real is the sim data?)

```bash
# campaign: replay real recordings through the twin, then score vs the
# real-to-real reference floor (conformal criterion, 41 metrics):
bash scripts/run_tub_campaign.sh
# or score any sim-run dir against any real npz dir directly:
$PY fidelity_scorecard.py --real <real_npz_dir>/*.npz --sim <sim_dir>/*.npz --out card
```

New real bag → npz first: `python3 tools/bag_to_npz.py <bag_dir> out.npz`.
Current state of the art: 38/41. If you change ANY physics or noise constant,
rerun the campaign — the scorecard decides, never your eyes.

## 5. The decision video (camera + truth-vs-DR + live controller decisions)

```bash
OUT=~/data/decision_run; mkdir -p $OUT/frames
$PY mavros_bridge.py --headless --control --pool intex --capture $OUT/frames --duration 100 &
# (wait for topics) then, each in its own terminal:
python3 <bluerov_dr>/dead_reckon.py --ros-args -p estimator_mode:=legacy_integrator &
$PY waypoint_controller.py --law p --yaw-sign -1 --log $OUT/decisions.csv &
$PY sim_traj_recorder.py $OUT &
# when the bridge exits, compose the three-panel mp4:
$PY compose_decision_video.py $OUT $OUT/decision_view.mp4
```

Output: chase camera | overhead with TRUTH vs DR-belief trails | live telemetry
of every decision the controller made.

## 6. Footage / containment check

```bash
$PY indoor_pool_capture.py out_frames 100   # chase gif/stills + track CSV
                                            # prints containment + wall clearance
```

## 7. Statistical tooling (reproduce the paper numbers)

```bash
$PY threshold_study.py <real_npz_dirs>       # criterion calibration (conformal vs MWU)
$PY method_selection_study.py --real <dirs> --hist <old_campaign_dirs> --final <sim_dir>
                                             # AUC bracket + close/unsure/divergent verdicts
$PY score_correlation.py <sim_dir> <real_dir>  # sim-vs-real ranking correlation
$PY tubtest_floats.py unarmed.npz armed.npz    # sensor-vs-vibration noise decomposition
$PY tub_sysid.py <bag> surge|sway|heave|yaw    # steady-state dynamics check
```

## Things that WILL bite you (read once, save hours)

- **One bridge at a time.** Concurrent bridges publish to the same topics and
  poison every consumer. Check first — with the bracket trick, since a plain
  pgrep matches your own command text: `pgrep -f "[m]avros_bridge"`.
- **Run long campaigns detached** (`nohup … &` on the sim machine). A dropped
  ssh kills foreground runs; it has eaten whole campaigns.
- **Conventions are measured — don't "fix" them:** IMU gravity sign-flip;
  acceleration control scheme is engine index 2 (docs list it wrong); DVL wire
  format is FRD with a −1.848° mount yaw and a lever arm; yaw command
  convention inverted (controllers run `--yaw-sign -1`); this vehicle's motor
  directions live in ESC config so logged PWMs are direction-less; the DVL
  stream publishes each measurement twice. Provenance for every constant is in
  the comments at the top of `sim/mavros_bridge.py` / `sim/bluerov2_standard_model.py`.
- **The physics engine sleeps slow bodies** and then ignores forces — the pool
  scripts carry a teleport watchdog; keep it in any new control loop.
- **The course is 2.0 × 0.6 m** (`WAYPOINTS` in `sim/waypoint_controller.py`,
  `SQUARE` in `sim/score_ranking.py`) — the largest that fits the real 4×2 m
  pool. Change it in both places or neither.
- Keep server hostnames / netids out of this public repo.

## Data locations (ask a teammate for access)

Real recordings: the lab Box ("Deliverables-underwater"). Working copies +
outputs: the lab GPU server under `~/data/` — real bags as npz in
`real_npz*` / `tubtest_npz`, sim campaigns in `mc_runs*`, scorecards as
`scorecard*.json`, benchmarks in `benchmark_*`.
