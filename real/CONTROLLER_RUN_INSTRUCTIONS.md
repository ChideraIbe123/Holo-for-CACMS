# Controller Run Instructions (topside laptop)

For the person operating the BlueROV2 from the topside laptop. Follow the steps in order.

**Status:** this code has passed simulator tests and a fake-autopilot test. It has never
driven the real vehicle. The first session is a shakedown. Do the bench check in Step 4
before any controller runs, and keep one hand near the SPACE key.

**What it does:** runs five path-following controllers, one at a time. Each drives the
ROV along a short course and stops by itself.

---

## Test description

```
Controller test: (Duration ~ 45 minutes. Short version: Bags 3 and 14-28 only, ~ 20 minutes)
Sensors: camera, DVL, autopilot IMU + depth (MAVROS), dead reckoning. Sonar optional.
ROV: MANUAL mode, battery above 15 V at start, tether floated with 2 m of slack.
Pool: pump off for 10 minutes before the first bag. Same lighting as Bag 1.

Bag 3: empty pool, bench check. ROV held at the wall. Arm, jog forward, yaw both ways,
       disarm. (1 minute)

Bags 4-13: empty pool, rectangle course. ROV starts 1.0 m from a short wall and 0.7 m
       from the long wall on its RIGHT, nose pointing down the long axis of the pool.
       The controller drives 2.0 m ahead, 0.6 m to the left, and back, for two laps
       (~ 45 s). Five controllers x 2 runs.

Bags 14-28: pipeline at an angle in the middle of the pool (same placement as Bag 2).
       ROV starts level with one end of the pipe, 0.3 m to the RIGHT of it, nose
       parallel to the pipe. The controller converges onto the pipe and follows it
       for 2.4 m (~ 9 s). Five controllers x 3 passes.
       Needs 2.4 m of straight pipe with at least 0.8 m of clear water past its far end.
```

Put a strip of tape on the pool rim at each start position so every run starts from the
same place.

---

## Step 1. Get the code (once)

The code must sit in a folder your ROS 2 terminal can see.

- **Docker workflow** (the `ros2` branch of BlueROV-Tools): only the BlueROV-Tools folder is
  shared with the container, so clone inside it. On the host:
  ```bash
  cd <path to>/BlueROV-Tools
  git clone -b pilot-controllers https://github.com/ChideraIbe123/Holo-for-CACMS.git pilot_kit
  ```
  Git will list `pilot_kit/` as untracked in BlueROV-Tools. Do not commit it there.
  Inside the container it appears at `/workspaces/ros2_ws/src/BlueROV-Tools/pilot_kit`.
- **Native ROS 2:** clone anywhere, for example `~/pilot_kit`, with the same command.

Everything you run is in `pilot_kit/real/`.

## Step 2. Start the vehicle stack

1. Power the ROV and wait for BlueOS.
2. Open the Mission Control GUI the usual way:
   ```bash
   ros2 run bluerov_gui bluerov_gui
   ```
3. **Launch** tab: click **Launch All**. Wait until MAVROS, DVL Driver, Camera show as alive.
4. In the Launch tab, **stop "Dead Reckoning"** (and "EKF Odometry" if it started).
   We start dead reckoning by hand in Step 3 so its mode is known.
5. **Do NOT click "Connect" in the Teleop panel**, and do not run `keyboard_teleop`.
   Both send stick commands continuously on the same link this code uses. If the GUI
   teleop is already connected, disconnect it now.
6. Confirm the vehicle is in **MANUAL** mode.

## Step 3. Open three ROS 2 terminals

Open each the way you normally open a ROS 2 terminal (Docker: `docker/scripts/ros2-shell.sh`
then `source /workspaces/ros2_ws/install/setup.bash`. Native: `source ~/ros2_ws/install/setup.bash`).
In each one:
```bash
cd <pilot_kit>/real
python3 -c "import rclpy, pymavlink; print('ok')"      # must print ok
```

**Terminal A, dead reckoning:**
```bash
ros2 run bluerov_dr dead_reckon --ros-args -p estimator_mode:=legacy_integrator
```
Check it from another terminal:
```bash
ros2 param get /dvl_dead_reckon estimator_mode     # legacy_integrator
ros2 topic hz /deadreckon/odom                     # about 35 Hz
```

**Terminal B, the stick converter.** Leave this terminal in focus during runs. It holds
the emergency stop.
```bash
python3 cmdvel_to_manual.py --depth-mode passthrough
```
For the pipe (line) course, start it with a longer fence instead:
```bash
python3 cmdvel_to_manual.py --depth-mode passthrough --fence=-0.4,2.9,-0.4,0.8
```
It should print `MAVLink connected` and then `vehicle: disarmed, mode MANUAL`.
If it prints `no heartbeat from the vehicle`, something else is holding port 14550
(GUI teleop, keyboard teleop, QGroundControl). Close that and retry.

Keys in Terminal B:

| Key | Action |
|---|---|
| `a` | arm |
| **SPACE** or `d` | **disarm and latch a stop** |
| `c` | clear a latched stop |
| `i` `k` | jog forward, back (low power, between runs only) |
| `j` `l` | jog yaw left (counter-clockwise), right (clockwise) |

**Terminal C** is for starting runs (Step 5).

## Step 4. Bench check (Bag 3). Do not skip.

ROV in the water, held at the wall or on a short tether.

1. Terminal B: press `a`. Expect `vehicle: ARMED`.
2. Tap `i` a few times. The ROV must push **forward**.
3. Tap `l`. Seen from above, the nose must swing **clockwise**. Tap `j`: counter-clockwise.
4. Press SPACE. Expect `DISARM sent` and the thrusters stop.
5. Press `c` to clear the stop.

If forward or yaw goes the wrong way, or SPACE does not disarm, **stop here** and report it.

## Step 5. One run

1. Place the ROV on its start mark, nose pointing along the course (see Test description).
   The course is laid out from wherever the ROV is and whichever way it faces at the
   moment the run starts, so placement matters.
2. Terminal B: `c` if a stop is latched, then `a` to arm.
3. Terminal C:
   ```bash
   ./run_one.sh <law> <course> <rep>
   ```
   | `<law>` | Controller |
   |---|---|
   | `p` | proportional heading |
   | `pursuit` | pure pursuit |
   | `los` | line-of-sight |
   | `ilos` | integral line-of-sight |
   | `fuzzy_lab` | the lab's fuzzy path tracker |

   `<course>` is `rect` or `line`. `<rep>` is the repeat number: 1, 2, 3.
   Example: `./run_one.sh los line 1`

   The script resets dead reckoning, starts a bag, runs the controller, and stops the bag.
4. The ROV stops by itself. Terminal B prints `run ended: holding neutral`.
5. Write on the run sheet: run name, battery voltage, and anything unusual.
6. Bring the ROV back to the start mark with `i k j l` or by hand. Repeat from 1.

If the pipe is shorter than 2.4 m, set the length to follow, leaving 0.3 m spare:
```bash
LINE_LENGTH=1.8 ./run_one.sh los line 1
```

### Run order

Interleave the controllers so battery drain is spread evenly. Do the whole list once
before starting the next repeat.

```
Rectangle, empty pool:    p  pursuit  los  ilos  fuzzy_lab      (rep 1), then again (rep 2)
Pipe line:                los  p  fuzzy_lab  pursuit  ilos      (rep 1), (rep 2), (rep 3)
```

## Stopping

| Situation | Do this |
|---|---|
| Anything looks wrong | **SPACE** in Terminal B. Disarms and latches. |
| Terminal B frozen or lost | Ctrl-C in Terminal B, or pull the ROV in by the tether. |
| Converter prints `STOP LATCHED: geofence` | The estimate left the allowed box. Ctrl-C in Terminal C, reposition, press `c`, re-arm. |
| ROV drifts toward a wall with no latch | The estimate has drifted. SPACE, reposition, and note it on the run sheet. |

A run that needed a stop still counts as data. Keep its folder and note what happened.
Then repeat it with the next rep number.

## If depth misbehaves

By default the controller holds 0.3 m depth through the throttle stick (set `DEPTH=-0.4 ./run_one.sh ...` to change it). If the ROV sinks,
surfaces repeatedly, or porpoises:

1. Ctrl-C Terminal B and restart it with `--depth-mode neutral`.
2. The ROV now rides at the surface, where it is slightly buoyant. Carry on.

## After the session

Each run leaves a folder `real/runs/<law>_<course>_r<rep>/` containing `bag/`, `runner.csv`
and `bag.log`.

```bash
python3 score_runs.py runs/
```
prints time, path length and tracking error per run.

Upload the whole `runs/` folder and a photo of the run sheet to Box under
`2026-MM-DD Swimming tub test/controller_runs/`.

## Optional: the LLM pilot (camera in, actions out)

`llm_pilot.py` lets a vision-language model drive from the camera. It works in steps:
take a frame, ask the model for one small action, carry it out for up to 2 seconds, stop,
take the next frame. The ROV stands still while the model thinks. Its commands go through
the same converter, so the stick limits, geofence and SPACE stop all still apply.

It needs internet on the topside laptop and a model API key. Each step uploads one 768 px
JPEG of the camera view to the model provider. Two providers work:

| Provider | Default model | Key variable | Install |
|---|---|---|---|
| OpenAI | `gpt-6-astra` | `OPENAI_API_KEY` or `OPENAI_KEY` | `python3 -m pip install openai` |
| Anthropic | `claude-opus-5-5` | `ANTHROPIC_API_KEY` | `python3 -m pip install anthropic` |

**Setup, once, in a ROS 2 terminal:**
```bash
python3 -m pip install openai                 # or anthropic
echo 'OPENAI_KEY=...' > ../.env               # ask the project lead for the key
chmod 600 ../.env                             # .env is git-ignored. Never commit a key.
python3 llm_pilot.py --check                  # one test call, no motion. Must print CHECK OK
```
The pilot picks the provider from whichever key it finds; force one with `--provider`.
`--check` also prints how long one reply takes. Measured with `gpt-6-astra`: 5 to 8 seconds
and about 1,250 input tokens per step.

**What to expect (simulator trials, 2026-10-06):** the model found the pipe and steered
onto it, holding about 0.1 m to one side. It judged distance badly: it stopped after
0.5 m, reporting the end of the pipe half a metre ahead when it was 2.3 m ahead. Without
being told what the far wall was, it refused to move at all. Treat it as an experiment,
start with `--dry-run`, and expect to word the goal carefully.

**Run it:**
1. Terminal B, converter with a longer run limit and a fence sized for free movement
   (the box is measured from where the ROV starts, in metres: back, ahead, right, left):
   ```bash
   python3 cmdvel_to_manual.py --depth-mode passthrough --max-run 300 --fence=-0.6,3.0,-0.8,0.8
   ```
2. Dry run first. The model decides and logs, but the ROV does not move:
   ```bash
   python3 llm_pilot.py --dry-run --goal "Follow the pipe on the pool floor to its far end, then stop." --out llm_runs/dry1
   ```
   Read the printed decisions. If they make sense for what the camera shows, continue.
3. Arm (`a` in Terminal B), then run it live, with a bag recording in another terminal:
   ```bash
   python3 llm_pilot.py --depth-target -0.3 --goal "Follow the pipe on the pool floor to its far end, then stop." --out llm_runs/pipe1
   ```
4. It stops when the model says it is done, after 40 steps, or after 4 minutes.
   **SPACE in Terminal B stops it at any time.**

Each run folder holds every frame the model saw (`frame_0001.jpg`, ...) and `steps.jsonl`
with each decision, its stated reason, the reply time and the token count.

The goal is plain English. Other examples: `"Drive toward the far wall and stop about one
metre before it."`, `"Turn on the spot until you can see the pipe, then stop."`

## Known limits

- **Estimator drift.** Dead reckoning drifts a few percent of distance travelled. In a
  pool 2 m wide that can reach tens of centimetres within a minute. The geofence uses
  the estimate, so it cannot see that drift.
- **Stick gains are open-loop.** They were measured at 15.4 V in MANUAL mode. On a low
  battery the ROV runs slower than commanded.
- **MANUAL mode only.** In ALT_HOLD or STABILIZE the autopilot reinterprets the yaw stick
  and the gains are wrong.

## What is in this folder

| File | Purpose |
|---|---|
| `tracker.py` | The five steering laws and the course logic. No ROS. |
| `course_runner.py` | Runs one law: reads `/deadreckon/odom`, publishes `/cmd_vel`. |
| `cmdvel_to_manual.py` | Turns `/cmd_vel` into stick commands. Owns arm, stop, jog. |
| `run_one.sh` | One recorded run on the vehicle. |
| `llm_pilot.py` | A vision-language model drives from the camera, one small step at a time. |
| `score_runs.py` | Scores finished runs. |
| `sim_run.sh`, `test_tracker.py`, `test_mavlink_fake.py`, `test_llm_fake.py` | Simulator and offline tests. |
