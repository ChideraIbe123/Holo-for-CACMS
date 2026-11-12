# LLM pilot (optional, not part of the controller session)

Moved out of CONTROLLER_RUN_INSTRUCTIONS.md. Set up the vehicle with Steps 1-4 there first.

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

**What to expect (2026-10-06).** In the simulator, with the goal worded as below, the
model followed a 1.8 m pipe in 8 steps and 52 seconds, stayed within 0.09 m of it, and
stopped within 0.1 m of the intended point. Shown 11 real frames from the March pipe
recording, it chose to advance along the pipe in 8, and correctly chose to turn and look in
the frames where no pipe was visible. It has not driven the real vehicle.

**Word the goal with something visible, not a distance.** The model judges distance badly.
Told "stop half a metre before the end" it stopped 2.3 m short. Told "stop when the far end
of the pipe reaches the bottom edge of the frame" it stopped in the right place.

Recommended goal for the pipe:
```
Follow the dark pipe lying on the pool floor to its far end, staying over it or right
beside it. Stop when the far end of the pipe reaches the bottom edge of the frame or passes
out of view below it.
```

**Three stages. Do them in order, and stop at the first one that looks wrong.**

*Stage 1, offline, no ROV.* Show the model recorded frames and read what it would do:
```bash
python3 llm_pilot.py --frames <folder of .png or .jpg> --goal "<goal>" --out llm_runs/offline1
```
Each frame is judged on its own. Use this the moment the first camera frames of the day exist.

*Stage 2, live dry run, ROV in the water but not moving.* Step 2 below.

*Stage 3, live.* Step 3 below.

**Run it:**
1. Terminal B, converter with a longer run limit and a fence sized for free movement
   (the box is measured from where the ROV starts, in metres: back, ahead, right, left):
   ```bash
   python3 cmdvel_to_manual.py --depth-mode passthrough --max-run 300 --fence=-0.6,3.0,-0.8,0.8
   ```
2. Dry run first. The model decides and logs, but the ROV does not move:
   ```bash
   python3 llm_pilot.py --dry-run --goal "<goal>" --out llm_runs/dry1
   ```
   Read the printed decisions. Move the ROV by hand to a few spots (beside the pipe, facing a
   wall, pipe out of view) and check the decision is sensible at each. If so, continue.
3. Arm (`a` in Terminal B), then run it live, with a bag recording in another terminal:
   ```bash
   python3 llm_pilot.py --depth-target -0.3 --goal "<goal>" --out llm_runs/pipe1
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

