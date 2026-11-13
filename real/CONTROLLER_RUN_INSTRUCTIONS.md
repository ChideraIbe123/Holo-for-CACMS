# Controller Session Instructions

A checklist for running the controller test on the BlueROV2 from the topside laptop.
Do the steps in order. Every command can be copied and pasted.

**What happens:** the ROV sinks to 0.5 m, drives a short course by itself, then stops. You do this
42 times: 7 controllers, 2 courses, 3 times each. It takes about 2 hours in the water.

**Safety:** keep one hand near the **SPACE** key in Terminal B. SPACE stops the
thrusters immediately.

---

## Before you start

- [ ] Battery above 15 V
- [ ] Tether floating, with 2 m of slack
- [ ] Pool pump OFF for 10 minutes before the first run
- [ ] Tape marks on the pool rim at both start positions (see "The two courses" below)
- [ ] A printed run sheet (template at the bottom of this page)

---

## Step 1. Get the code (first time only)

In a normal terminal on the laptop:
```bash
cd <path to>/BlueROV-Tools
git clone -b pilot-controllers https://github.com/ChideraIbe123/Holo-for-CACMS.git pilot_kit
echo pilot_kit/ >> .git/info/exclude
```
Already have it? Update it instead:
```bash
cd <path to>/BlueROV-Tools/pilot_kit && git pull
```
Not sure where BlueROV-Tools is? It is usually `~/ros2_ws/src/BlueROV-Tools`. Go into it
and type `pwd` to see its full path.

---

## How to open a ROS 2 terminal

You need four of these (one for the GUI, three for Terminals A, B, C). Open each the
way you normally do for the lab GUI. On the lab laptop ROS 2 is installed directly (no
Docker), so in each new terminal:
```bash
source /opt/ros/jazzy/setup.bash        # if `ls /opt/ros` shows humble instead, use humble
source ~/ros2_ws/install/setup.bash
cd <path to>/BlueROV-Tools/pilot_kit/real
python3 -c "import rclpy, pymavlink; print('ok')"      # must print ok
```
If `~/ros2_ws/install/setup.bash` does not exist, the lab workspace has not been built:
`cd ~/ros2_ws && colcon build` once, then try again. (Docker setups instead use
`docker/scripts/ros2-shell.sh` from the BlueROV-Tools folder.)

---

## Step 2. Start the vehicle

1. Power on the ROV. Wait for BlueOS to come up.
2. In your first ROS 2 terminal, start the lab GUI:
   ```bash
   ros2 run bluerov_gui bluerov_gui
   ```
3. Go to the **Launch** tab and click **Launch All**. Wait until MAVROS, DVL Driver and
   Camera show as alive.
4. In the same tab, **stop "Dead Reckoning"** (and "EKF Odometry" if it is running).
   You will start it yourself in Step 3.
5. **Do NOT click "Connect" in the Teleop panel.** If it is already connected,
   disconnect it. It fights with our code for control of the ROV.
6. Check the ROV is in **MANUAL** mode.

---

## Step 3. Open three more ROS 2 terminals

Open them as described in "How to open a ROS 2 terminal" above.

**Terminal A: position tracking.** Paste this and leave it running:
```bash
ros2 run bluerov_dr dead_reckon --ros-args -p estimator_mode:=legacy_integrator
```
Check it from Terminal C:
```bash
ros2 param get /dvl_dead_reckon estimator_mode     # must say legacy_integrator
ros2 topic hz /deadreckon/odom                     # must show messages arriving (Ctrl-C to stop)
```

**Terminal B: control and emergency stop.** For the **rectangle** runs:
```bash
python3 cmdvel_to_manual.py --depth-mode passthrough
```
It should print `MAVLink connected` and then `vehicle: disarmed, mode MANUAL`.
If it says `no heartbeat from the vehicle`, the GUI teleop (or QGroundControl) is still
connected. Close it and try again.

Terminal B keys:

| Key | What it does |
|---|---|
| `a` | arm (thrusters on) |
| **SPACE** | **STOP: thrusters off** |
| `c` | clear a stop, so you can arm again |
| `i` / `k` | nudge forward / back |
| `j` / `l` | turn left / right |

**Terminal C: starting runs.** You type the run commands here.

---

## Step 4. Thruster check (do not skip)

Hold the ROV at the wall.

1. Terminal B: press `a`. It should say `vehicle: ARMED`.
2. Tap `i` a few times. The ROV should push **forward**.
3. Tap `l`. The nose should turn **right** (clockwise from above). Tap `j`: it turns **left**.
4. Press **SPACE**. The thrusters should stop.
5. Press `c`.

If anything goes the wrong way, or SPACE does not stop it, **stop and report it.**
Do not continue.

---

## Step 5. How depth works (read once)

The ROV drives **0.5 m below the surface**, the same depth the simulator used. You do
not set anything. When you start a run:

1. The script zeroes the depth reading **where the ROV is at that moment**. So the ROV
   must be **floating freely** when you press Enter: not held, not touching the wall,
   not still sinking from the last run.
2. The ROV sinks straight down with no forward motion and waits until its depth sensor
   has read 0.5 m (within 5 cm) for a full second. Terminal C prints
   `at depth: -0.50 m after 4.1 s (settled)`.
3. Only then does the course start, laid out from where the ROV is at that moment.

If the ROV was not floating (depth did not read about 0), the script prints
`NOT STARTING` and does nothing. Let the ROV float up, then run the same command again.

Each run folder gets a `depth_gate.json` with the depth before, during and after the
dive, and the DVL's height above the floor, so depth can be checked afterwards.

---

## Step 6. Do one run

1. Put the ROV on the start mark, facing the right way (see "The two courses").
   The ROV lays out the course from wherever it starts, so the start position matters.
2. Terminal B: press `c`, then `a` (arm).
3. Terminal C: type the run command, for example:
   ```bash
   ./run_one.sh ilos rect 1
   ```
   That means controller `ilos`, course `rect`, repeat `1`.
   It prints `diving to -0.5 m`, then a few seconds later `at depth: -0.50 m ... (settled)`,
   then the ROV starts driving. `NOT STARTING` means the ROV was not floating freely:
   let it float up and run the same command again.
4. Wait. The ROV drives the course and stops by itself. Terminal B says
   `run ended: holding neutral`.
5. Fill in the run sheet: run name, battery voltage, the `at depth` line, anything odd.
6. Bring the ROV back to the start mark (keys `i` `k` `j` `l`, or by hand).

Then do the next run on the list.

---

## The two courses

**Rectangle (`rect`).** Empty pool. Start 1.0 m from a short wall and 0.7 m from the
long wall on the ROV's **right**, nose pointing down the long side. The ROV drives a
2.0 m by 0.6 m rectangle twice (about 45 seconds).

**Line (`line`).** Pipe in the middle of the pool. Start level with one end of the pipe,
0.3 m to its **right**, nose parallel to the pipe. The ROV moves onto the pipe and
follows it for 2.4 m (about 9 seconds). You need 2.4 m of straight pipe plus 0.8 m of
clear water past the far end. If the pipe is shorter, put `LINE_LENGTH=` in front of
the command, with the pipe length minus 0.3 (for example a 2.1 m pipe):
```bash
LINE_LENGTH=1.8 ./run_one.sh los line 1
```

**Before the line runs, restart Terminal B** with a bigger safety box. Press Ctrl-C in
Terminal B, then:
```bash
python3 cmdvel_to_manual.py --depth-mode passthrough --fence=-0.4,2.9,-0.4,0.8
```

---

## Run list (do them in exactly this order)

The order is shuffled on purpose. Do not change it. Copy each command into Terminal C.
Tick each one off on the run sheet as you go.

**Rectangle: 21 runs**
```bash
./run_one.sh ilos rect 1
./run_one.sh los rect 1
./run_one.sh fuzzy_lab rect 1
./run_one.sh alos rect 1
./run_one.sh smc rect 1
./run_one.sh pursuit rect 1
./run_one.sh p rect 1

./run_one.sh smc rect 2
./run_one.sh pursuit rect 2
./run_one.sh ilos rect 2
./run_one.sh fuzzy_lab rect 2
./run_one.sh los rect 2
./run_one.sh alos rect 2
./run_one.sh p rect 2

./run_one.sh p rect 3
./run_one.sh alos rect 3
./run_one.sh ilos rect 3
./run_one.sh smc rect 3
./run_one.sh los rect 3
./run_one.sh pursuit rect 3
./run_one.sh fuzzy_lab rect 3
```

**Now restart Terminal B for the line course** (see "The two courses").

**Line: 21 runs**
```bash
./run_one.sh pursuit line 1
./run_one.sh alos line 1
./run_one.sh p line 1
./run_one.sh fuzzy_lab line 1
./run_one.sh los line 1
./run_one.sh smc line 1
./run_one.sh ilos line 1

./run_one.sh los line 2
./run_one.sh ilos line 2
./run_one.sh pursuit line 2
./run_one.sh fuzzy_lab line 2
./run_one.sh smc line 2
./run_one.sh p line 2
./run_one.sh alos line 2

./run_one.sh smc line 3
./run_one.sh los line 3
./run_one.sh fuzzy_lab line 3
./run_one.sh p line 3
./run_one.sh pursuit line 3
./run_one.sh ilos line 3
./run_one.sh alos line 3
```

**Extra, only if there is time left after all 42** (rectangle again, so restart
Terminal B with the rectangle command first):
```bash
./run_one.sh los rect 4
./run_one.sh alos rect 4
./run_one.sh smc rect 4
./run_one.sh pursuit rect 4
./run_one.sh ilos rect 4
./run_one.sh fuzzy_lab rect 4
./run_one.sh p rect 4
```
If time is short, finish all 42 before doing any extras.

---

## If something goes wrong

| What happened | What to do |
|---|---|
| Anything looks wrong | Press **SPACE** in Terminal B. |
| Terminal B is frozen | Ctrl-C in Terminal B, or pull the ROV in by the tether. |
| Terminal B says `STOP LATCHED: geofence` | The ROV left its safety box. Ctrl-C in Terminal C, move the ROV back, press `c`, arm again. |
| ROV heads for a wall and nothing stops it | Press **SPACE**. Note it on the run sheet. |
| Terminal C prints `NOT STARTING: depth reads ...` | The ROV was not floating freely when you pressed Enter. Let it float up, wait 5 s, run the same command again. |
| Terminal C prints `TIMED OUT, starting anyway` | The ROV could not reach 0.5 m in 12 s. The run continues. Note it on the run sheet. Check Terminal B was started with `--depth-mode passthrough` and the battery is not low. |
| ROV keeps sinking, popping up, or bobbing | Ctrl-C in Terminal B, restart it with `--depth-mode neutral` instead of `--depth-mode passthrough`. The ROV now stays at the surface and every run waits 12 s before driving. Write **surface mode** on the run sheet for every run after this. |

**If you had to stop a run:** that run does not count, but keep its folder. Redo it at
the end of the same block of 7, adding `b` to the repeat number:
```bash
./run_one.sh smc rect 2b
```

`run_one.sh` refuses to overwrite a run that already exists. If it says
`already exists`, you typed a run you already did. Check the run sheet.

---

## After the session

Check the runs:
```bash
python3 score_runs.py runs/
```
Then upload the whole `runs/` folder and a photo of the run sheet to Box:
`2026-MM-DD Swimming tub test/controller_runs/`

---

## Run sheet template

```
Date: ____________   Operator: ____________

Water depth, floor to surface, middle of the pool: ______ (simulator: 1.05 m)

Run name         | Battery V | at depth after | Done? | Notes
-----------------+-----------+----------------+-------+-----------------------
ilos rect 1      |           |                |       |
los rect 1       |           |                |       |
fuzzy_lab rect 1 |           |                |       |
...              |           |                |       |
```

---

## Notes for whoever maintains this

- Depth: the lab's `dead_reckon` node zeroes depth at `/deadreckon/reset`
  (`pos[2] = baro_depth - baro_depth_at_reset`), and `run_one.sh` sends that reset
  right before each run, so the controller's depth is metres below the FLOATING
  position and `DEPTH=-0.5` (the default) is 0.5 m below it. The autopilot's own
  depth zero is irrelevant. `--dive-first` makes the runner refuse unless depth reads
  within 0.10 m of zero at start, sink with zero surge/yaw until the sensor is within
  0.05 m of the target for 1.0 s (12 s timeout, flagged), then lay out the course.
  Tested offline: `python3 test_depth_gate.py`.
- Simulator comparison: the sim starts each run at depth with no reset; its depth
  sensor sits 0.50 m below the surface. The real sensor floats about 6 cm below the
  surface, so after a 0.5 m dive it sits about 0.56 m down: a 6 cm difference,
  recorded in `depth_gate.json`, negligible for the dynamics at that depth. The sim
  does NOT use `--dive-first` (its predictions are unchanged by this feature).
- The run order was generated with a fixed seed (Python `random.Random(20261103)`), as
  the pre-registration requires a randomized order.
- Position drifts a few percent of distance travelled. The safety box uses the estimated
  position, so it cannot see that drift.
- Stick gains were measured at 15.4 V in MANUAL mode. On a low battery the ROV is slower.
  MANUAL only: other modes change how the sticks behave.
- The `alos` and `smc` controllers have only run in the simulator. The other five ran on
  the real ROV in the 2026-10-08 session.
- Optional LLM camera pilot: see `LLM_PILOT.md`.

| File | What it is |
|---|---|
| `tracker.py` | The seven controllers and the course shapes |
| `course_runner.py` | Runs one controller |
| `cmdvel_to_manual.py` | Turns controller output into stick commands; arm, stop, nudge |
| `run_one.sh` | One recorded run |
| `test_depth_gate.py` | Offline test of the dive-then-start logic |
| `score_runs.py` | Scores finished runs |
| `llm_pilot.py` | Optional camera pilot (see `LLM_PILOT.md`) |
