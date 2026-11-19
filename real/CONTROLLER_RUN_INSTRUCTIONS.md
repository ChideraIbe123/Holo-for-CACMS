# Controller Session Instructions

A checklist for running the controller test on the BlueROV2 from the topside laptop.
Do the steps in order. Every command can be copied and pasted.

**What happens:** the ROV sinks, holding the direction you aimed it, drives a short course by
itself, then stops. You do this
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

## Step 5. Aim the ROV yourself (every run)

The course goes in the direction the ROV is **pointing when you let go of it**. The
start-up holds that heading while it sinks; it does not turn. So before every run:

1. Put the ROV on the start mark, floating, nose pointing **straight down the long side
   of the pool**. Sight along the long wall to get it parallel. Two or three degrees
   off is fine.
2. Let go and keep your hands off it until it starts driving.

Why not the compass: on the line day (2026-11-13) the ROV's compass heading for the
same placement wandered by up to 60 degrees between runs, so turning to a saved
compass heading sent the runs off at different angles (that is the left turn you saw
at the start of every run). A rectangle set off by 27 degrees would swing its far
corners 0.9 m sideways, into the wall. Your eyes along the wall are more reliable.

`save_heading.py` is no longer needed. (The old behaviour is still there if ever
wanted: put `HEADING=saved` in front of a run command.)

### What a run does before it drives (read once)

1. The script zeroes the depth reading **where the ROV is at that moment**, so the ROV
   must be **floating freely** when you press Enter: not held, not touching the wall, not
   still sinking from the last run.
2. It reads the DVL's height above the floor and picks the depth: 0.5 m, or less if that
   would leave under 0.30 m of water beneath the DVL (in a 38 in pool that is about
   0.35 to 0.45 m). Terminal C prints the depth it chose.
3. It sinks with no forward motion, holding the heading you placed it at, and waits
   until depth (within 5 cm) and heading (within 6 degrees) have both held for a
   second. Terminal C prints `initialised after N s ... Course starts.`
4. The course is laid out from where you placed the ROV, pointing the way you aimed
   it. Then it drives.

If it cannot settle within 45 s it does **not** drive: Terminal C prints `NOT STARTING`,
the run folder is renamed `..._ABORTED_<time>`, and you simply run the same command
again. Each run folder gets `start_gate.json` (depth, heading, altitude before and
during the run) and `init.csv` (the start-up trace).

---

## Step 6. Do one run

1. Put the ROV on the start mark, nose straight down the long side of the pool (see
   Step 5 and "The two courses"). The ROV lays out the course from where it starts and
   in the direction it points, so both matter.
2. Terminal B: press `c`, then `a` (arm).
3. Terminal C: type the run command, for example:
   ```bash
   ./run_one.sh ilos rect 1
   ```
   That means controller `ilos`, course `rect`, repeat `1`.
   It prints the depth it chose, then after a few seconds `initialised after N s: depth
   ..., heading error ... Course starts.` and the ROV drives. `NOT STARTING` means it
   could not settle (or was not floating freely): let it float up, check Terminal B is
   still in passthrough mode, and run the same command again.
4. Wait. The ROV drives the course and stops by itself. Terminal B says
   `run ended: holding neutral`.
5. Fill in the run sheet: run name, battery voltage, the `initialised after` line, anything odd.
6. Bring the ROV back to the start mark (keys `i` `k` `j` `l`, or by hand).

Then do the next run on the list.

---

## The two courses

**Rectangle (`rect`).** Empty pool. Start 1.0 m from a short wall and 0.7 m from the
long wall on the ROV's **right**, nose pointing straight down the long side (sight
along the wall). The ROV drives a 2.0 m by 0.6 m rectangle twice (about 45 seconds):
2.0 m ahead, 0.6 m to its left. With the nose straight, the nearest it should come to
any wall is about 0.2 m (fuzzy, on the left side); the safety box stops it 0.3 m
before every wall. Expect fuzzy to trip the box on some runs: that is a recorded
result, not a fault. Do not move the mark to give it more room.

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
| Terminal C prints `NOT STARTING: not settled after 45 s` | It could not hold depth and heading together. The folder is renamed `_ABORTED`; run the same command again. If it happens twice in a row: check Terminal B is in `--depth-mode passthrough`, the battery is above 15 V, and the tether is slack. |
| Terminal C prints `no saved heading` | Only happens with `HEADING=saved`. Drop that from the command (the default uses the heading you placed it at), or run `python3 save_heading.py` first. |
| The ROV turns before it drives | Only normal with `HEADING=saved`. By default it must not turn: check the run command and that Terminal C printed `heading=placed`. |
| Terminal C warns `depth target reduced` | Normal in a shallow pool: it keeps 0.30 m of water under the DVL. Note the depth on the run sheet. |
| ROV keeps sinking, popping up, or bobbing | Ctrl-C in Terminal B, restart it with `--depth-mode neutral` instead of `--depth-mode passthrough`. The ROV now stays at the surface. Runs will NOT start on their own in this mode (the start-up needs depth); only use it to recover the ROV. |

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

Heading: placed by hand (default) / saved compass heading ______ (circle one)

Run name         | Battery V | initialised after | Done? | Notes
-----------------+-----------+-------------------+-------+--------------------
ilos rect 1      |           |                   |       |
los rect 1       |           |                   |       |
fuzzy_lab rect 1 |           |                   |       |
...              |           |                   |       |
```

---

## Notes for whoever maintains this

- Start-up (`course_runner.py --init`, Sumanth, pushed 2026-11-12; the commits carry
  Shrey Mathur's git identity): the lab's
  `dead_reckon` node zeroes depth at `/deadreckon/reset`, which `run_one.sh` sends while
  the vehicle floats, so depth is metres below the floating position. The runner refuses
  unless depth reads within 0.10 m of zero, locks the depth target from five floating
  DVL altitudes (requested 0.5 m, capped to keep 0.30 m under the DVL), then sinks and
  holds the course heading with PI loops (depth damped by the depth-sensor rate, yaw
  by the gyro, 0.5 rad/s minimum yaw outside tolerance because smaller sticks sit in
  the thruster dead band), and starts only when depth (0.05 m) and heading (6 deg) hold
  for 1.0 s; 45 s timeout aborts (exit 3). The course is anchored at the placement
  point; the stick converter re-anchors its geofence there.
  Offline tests: `python3 test_start_gate.py`.
- Course heading (`--heading`, 2026-11-19): `placed` (default in `run_one.sh`) uses the
  estimator heading averaged while the vehicle floats on the mark, so there is no turn
  and the operator's aim defines the course. `saved` turns to `heading_ref.json`
  (`save_heading.py`), the 2026-11-13 line-session behaviour. Why: the 28 line-session
  `start_gate.json` files show `heading_at_start_deg` from -81 to -142 for the same
  hand placement against a fixed target of -70.4, i.e. the autopilot heading wandered
  about 60 deg between runs and the vehicle turned 10-71 deg before every run. The
  course start heading error was also +4 to +8 deg on every run (the yaw loop stops
  just inside its tolerance when arriving from one side); `placed` starts at ~0.
  `start_gate.json` records `heading_mode`.
- Simulator comparison: the same start-up runs in the twin (`sim_run.sh INIT=1`:
  surface spawn, reset at the float point, `--init` with heading 0, 0.965 m water), so
  the predictions include the floor-guarded depth. The twin's vehicle model carries the
  DVL A50 payload and the measured 12.7 deg nose-down float trim (v19).
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
| `save_heading.py` | Saves a compass heading for `HEADING=saved` runs (not used by default) |
| `test_start_gate.py` | Offline test of the start-up (sink + turn) logic |
| `test_depth_gate.py` | Offline test of the older dive-only start (`--dive-first`) |
| `score_runs.py` | Scores finished runs |
| `llm_pilot.py` | Optional camera pilot (see `LLM_PILOT.md`) |
