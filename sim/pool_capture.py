"""Simulate the UIUC CRCE leisure pool and capture footage + trajectory data.

Pool geometry (see work.md):
- Depth: official CRCE spec, 3'6" to 4'0" (1.07-1.22 m) with zero-depth entry.
  Main test area floor at -1.22 m (4 ft); a sloped entry ramp at the far end.
- Footprint: the leisure pool's plan dimensions are not published; the main swim
  area is approximated as 25 m x 12 m and labeled as an approximation.

The vehicle (standard 6-thruster BlueROV2, validated dynamics) runs a waypoint
lap inside the pool at ~0.6 m/s with depth-hold at 0.6 m — matching the real
pool tests (the lab's bag shows DVL altitude ~0.57 m, i.e. mid-depth in a 4 ft
pool). Two cameras record: a chase cam and an overhead cam.

Usage: python3 pool_capture.py <out_dir> [duration_s]
"""
import json
import math
import os
import sys

import numpy as np
import holoocean
from PIL import Image

from bluerov2_standard_model import BlueROV2StandardModel
from mavros_bridge import SCENARIO_JSON, AGENT_NAME, WATER_SURFACE_Z, quat_to_rot_matrix_xyzw, DYN_QUAT

FPS = 5
WIDTH, HEIGHT = 960, 540

# ---- CRCE pool model (meters, water surface at z = 0) ----
POOL_LEN = 25.0          # approximation — footprint not published
X_OFF = 35.0             # place pool on open seabed, away from world terrain
POOL_WID = 12.0          # y: -6 .. 6
FLOOR_Z = -1.22          # 4 ft, official CRCE max depth
RAMP_LEN = 4.0           # zero-entry slope at the far (x=25) end
WALL_T = 0.4
WALL_TOP = 1.6           # tall walls hide the outside world (indoor look)

DEPTH_TARGET = -0.6      # depth-hold setpoint (mid-column, like the real tests)
CRUISE_CMD = 0.33        # ~0.6 m/s
WAYPOINTS = [(X_OFF + 5.0, -3.0), (X_OFF + 19.0, -3.0), (X_OFF + 19.0, 3.0), (X_OFF + 5.0, 3.0)]


def spawn_pool(env):
    cx = X_OFF + POOL_LEN / 2
    # floor (white tile), top surface at FLOOR_Z
    env.spawn_prop("box", location=[cx, 0, FLOOR_Z - 0.2], scale=[POOL_LEN, POOL_WID, 0.4],
                   material="white")
    # lane stripes on the floor
    for y in (-2.0, 0.0, 2.0):
        env.spawn_prop("box", location=[cx, y, FLOOR_Z + 0.006], scale=[POOL_LEN - 6, 0.15, 0.012],
                       material="black")
    # zero-entry ramp at the far end, sloping up from the floor
    ramp_pitch = math.degrees(math.atan2(0.9, RAMP_LEN))
    env.spawn_prop("box", location=[X_OFF + POOL_LEN - RAMP_LEN / 2, 0, FLOOR_Z + 0.35],
                   rotation=[0, -ramp_pitch, 0], scale=[RAMP_LEN + 0.6, POOL_WID, 0.3],
                   material="white")
    # side walls (long)
    for y in (-POOL_WID / 2 - WALL_T / 2, POOL_WID / 2 + WALL_T / 2):
        env.spawn_prop("box", location=[cx, y, (WALL_TOP + FLOOR_Z - 0.4) / 2],
                       scale=[POOL_LEN + 2 * WALL_T, WALL_T, WALL_TOP - FLOOR_Z + 0.4],
                       material="white")
    # end walls
    for x in (X_OFF - WALL_T / 2, X_OFF + POOL_LEN + WALL_T / 2):
        env.spawn_prop("box", location=[x, 0, (WALL_TOP + FLOOR_Z - 0.4) / 2],
                       scale=[WALL_T, POOL_WID, WALL_TOP - FLOOR_Z + 0.4],
                       material="white")


def yaw_of(quat):
    x, y, z, w = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def lap_command(dyn, wp_state):
    """Waypoint-following: cruise toward the current waypoint, depth-hold verticals."""
    pos = dyn[6:9]
    quat = dyn[DYN_QUAT]
    wp = WAYPOINTS[wp_state["i"]]
    dx, dy = wp[0] - pos[0], wp[1] - pos[1]
    if math.hypot(dx, dy) < 1.2:
        wp_state["i"] = (wp_state["i"] + 1) % len(WAYPOINTS)
        wp = WAYPOINTS[wp_state["i"]]
        dx, dy = wp[0] - pos[0], wp[1] - pos[1]

    err = math.atan2(dy, dx) - yaw_of(quat)
    err = math.atan2(math.sin(err), math.cos(err))
    r = dyn[14]   # yaw rate; damping prevents the turn limit-cycle
    d = float(np.clip(1.6 * err - 0.9 * r, -0.45, 0.45))
    fwd = CRUISE_CMD if abs(err) < 0.9 else 0.18   # slow down for sharp turns

    cmd = np.zeros(6)
    cmd[[0, 2]] = np.clip(fwd + d, -0.8, 0.8)      # raising T1,T3 turns CCW
    cmd[[1, 3]] = np.clip(fwd - d, -0.8, 0.8)
    # -0.26 feedforward trims out net buoyancy (P-only control leaves a steady offset)
    cmd[4:6] = float(np.clip(-0.26 + 0.8 * (DEPTH_TARGET - pos[2]) - 0.8 * dyn[5], -0.6, 0.6))
    return cmd


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "pool_frames"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 75.0
    os.makedirs(out, exist_ok=True)

    with open(SCENARIO_JSON) as f:
        scenario = json.load(f)
    scenario["world"] = "SimpleUnderwater"
    agent = scenario["agents"][0]
    agent["control_scheme"] = 2
    agent["location"] = [X_OFF + 5.0, -3.0, -0.6]
    agent["rotation"] = [0, 0, 0]
    for s in agent["sensors"]:
        if s["sensor_type"] == "DynamicsSensor":
            s["Hz"] = scenario["ticks_per_sec"]
    agent["sensors"].append({
        "sensor_type": "RGBCamera", "sensor_name": "ChaseCam",
        "location": [-2.6, 0.0, 0.15], "rotation": [0.0, 8.0, 0.0],
        "Hz": FPS, "configuration": {"CaptureWidth": WIDTH, "CaptureHeight": HEIGHT},
    })
    # stationary wide-view camera: a second agent parked in the pool corner
    # (raw-dynamics scheme + zero commands = frozen in place)
    scenario["agents"].append({
        "agent_name": "cam0", "agent_type": "BlueROV2",
        "control_scheme": 2,
        "location": [X_OFF + 1.6, -4.6, -0.45], "rotation": [0, 0, 18],
        "sensors": [{
            "sensor_type": "RGBCamera", "sensor_name": "WideCam",
            "location": [0.9, 0.0, 0.12], "rotation": [0.0, 2.0, 0.0],
            "Hz": FPS, "configuration": {"CaptureWidth": WIDTH, "CaptureHeight": HEIGHT},
        }],
    })

    model = BlueROV2StandardModel()
    ticks = float(scenario["ticks_per_sec"])
    chase, top, track = [], [], []
    freeze_hist = []
    t, last_dyn = 0.0, None
    wp_state = {"i": 0}

    with holoocean.make(scenario_cfg=scenario, show_viewport=False) as env:
        spawn_pool(env)
        while t < duration:
            if last_dyn is not None:
                quat = last_dyn[DYN_QUAT]
                R = quat_to_rot_matrix_xyzw(quat)
                nu = np.concatenate([R.T @ last_dyn[3:6], R.T @ last_dyn[12:15]])
                cmd = lap_command(last_dyn, wp_state)
                nu_dot = model.step(cmd, quat, nu, z_world=float(last_dyn[8]),
                                    surface_z=WATER_SURFACE_Z)
                env.act(AGENT_NAME, np.concatenate([R @ nu_dot[:3], R @ nu_dot[3:]]))
                # physics-sleep watchdog: engine sleeps slow bodies and then
                # ignores forces; a tiny teleport wakes it
                pos = last_dyn[6:9]
                freeze_hist.append(pos.copy())
                if len(freeze_hist) > 100:
                    freeze_hist.pop(0)
                    if np.linalg.norm(pos - freeze_hist[0]) < 0.002 and np.abs(cmd[:4]).max() > 0.05:
                        env.agents[AGENT_NAME].teleport(location=pos + np.array([0, 0, 0.003]))
                        freeze_hist.clear()
            env.act("cam0", np.zeros(6))
            state = env.tick()
            t = float(state.get("t", t + 1.0 / ticks))
            a = state.get(AGENT_NAME, state)
            c = state.get("cam0", {})
            if "DynamicsSensor" in a:
                last_dyn = np.asarray(a["DynamicsSensor"], dtype=float)
                track.append([t, last_dyn[6], last_dyn[7], last_dyn[8]])
            if "ChaseCam" in a:
                img = np.asarray(a["ChaseCam"])[:, :, :3][:, :, ::-1]
                chase.append(Image.fromarray(img.astype(np.uint8)))
            if "WideCam" in c:
                img = np.asarray(c["WideCam"])[:, :, :3][:, :, ::-1]
                top.append(Image.fromarray(img.astype(np.uint8)))
            if len(chase) % 50 == 1:
                pass

    with open(os.path.join(out, "pool_track.csv"), "w") as f:
        f.write("t,x,y,z\n")
        for r in track[::4]:
            f.write(",".join(f"{v:.4f}" for v in r) + "\n")

    for name, buf in (("chase", chase), ("top", top)):
        if not buf:
            continue
        for i in (len(buf) // 4, len(buf) // 2, 3 * len(buf) // 4):
            buf[i].save(os.path.join(out, f"{name}_still_{i:04d}.png"))
        buf[0].save(os.path.join(out, f"pool_{name}.gif"), save_all=True,
                    append_images=buf[1:], duration=int(1000 / FPS), loop=0)
    print(f"[pool] saved {len(chase)} chase + {len(top)} top frames, "
          f"{len(track)} track points -> {out}")


if __name__ == "__main__":
    main()
