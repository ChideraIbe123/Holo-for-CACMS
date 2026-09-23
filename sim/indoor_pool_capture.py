"""Simulate the lab's indoor Intex pool and capture footage + trajectory data.

Pool: Intex Clearview Prism Frame Rectangular, model 26770 — the lab's actual
workshop pool (manual photographed 2026-09-17). Exact geometry:
- Inner footprint 400 x 200 cm; frame/wall height 122 cm.
- Design capacity 8,418 L = 90% fill line -> water depth 8.418/(4.0*2.0) = 1.05 m.
  Water surface at z=0, floor at -1.05 m; walls rise 0.17 m above the surface.
- Liner approximated as white (real is light blue/gray; Clearview window strips
  in the walls are cosmetic and not modeled).

The vehicle (standard 6-thruster BlueROV2, validated dynamics) runs a tight
2.4 x 0.8 m rectangle with depth-hold at 0.5 m — mid-column, giving DVL
altitude ~0.55 m, the same bottom-lock regime as the real CRCE bags (~0.57 m).
This is the course scale the REAL ranking session must also use: a 3x3 m
square does not fit a 4x2 m pool (2.0 x 0.6 m verified: overshoot stays clear).

Usage: python3 indoor_pool_capture.py <out_dir> [duration_s]
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

# ---- Intex 26770 pool model (meters, water surface at z = 0) ----
POOL_LEN = 4.0           # x: inner footprint, exact
POOL_WID = 2.0           # y: -1 .. 1, exact
FLOOR_Z = -1.05          # 8418 L / (4.0 x 2.0 m) — the 90% design fill line
WALL_TOP = 0.17          # 1.22 m wall minus 1.05 m water
WALL_T = 0.15            # thin liner-on-frame walls (vs concrete CRCE)
X_OFF = 35.0             # place on open seabed, away from world terrain

DEPTH_TARGET = -0.5      # mid-column; DVL altitude ~0.55 m like the real bags
CRUISE_CMD = 0.17        # ~0.3 m/s — pool is 4 m long, keep speeds gentle
REACH = 0.35
WAYPOINTS = [(X_OFF + 1.0, -0.3), (X_OFF + 3.0, -0.3),
             (X_OFF + 3.0, 0.3), (X_OFF + 1.0, 0.3)]  # 2.0 x 0.6 m: first cut (2.4x0.8) grazed a wall at corner overshoot


def spawn_pool(env):
    cx = X_OFF + POOL_LEN / 2
    # floor, top surface at FLOOR_Z
    env.spawn_prop("box", location=[cx, 0, FLOOR_Z - 0.2], scale=[POOL_LEN, POOL_WID, 0.4],
                   material="white")
    # side walls (long)
    for y in (-POOL_WID / 2 - WALL_T / 2, POOL_WID / 2 + WALL_T / 2):
        env.spawn_prop("box", location=[cx, y, (WALL_TOP + FLOOR_Z - 0.3) / 2],
                       scale=[POOL_LEN + 2 * WALL_T, WALL_T, WALL_TOP - FLOOR_Z + 0.3],
                       material="white")
    # end walls
    for x in (X_OFF - WALL_T / 2, X_OFF + POOL_LEN + WALL_T / 2):
        env.spawn_prop("box", location=[x, 0, (WALL_TOP + FLOOR_Z - 0.3) / 2],
                       scale=[WALL_T, POOL_WID, WALL_TOP - FLOOR_Z + 0.3],
                       material="white")


def yaw_of(quat):
    x, y, z, w = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def lap_command(dyn, wp_state):
    """Waypoint-following scaled to the small pool: gentle cruise, tight reach."""
    pos = dyn[6:9]
    quat = dyn[DYN_QUAT]
    wp = WAYPOINTS[wp_state["i"]]
    dx, dy = wp[0] - pos[0], wp[1] - pos[1]
    if math.hypot(dx, dy) < REACH:
        wp_state["i"] = (wp_state["i"] + 1) % len(WAYPOINTS)
        wp = WAYPOINTS[wp_state["i"]]
        dx, dy = wp[0] - pos[0], wp[1] - pos[1]

    err = math.atan2(dy, dx) - yaw_of(quat)
    err = math.atan2(math.sin(err), math.cos(err))
    r = dyn[14]
    d = float(np.clip(1.6 * err - 0.9 * r, -0.4, 0.4))
    fwd = CRUISE_CMD if abs(err) < 0.9 else 0.08   # near-stop pivots at corners

    cmd = np.zeros(6)
    cmd[[0, 2]] = np.clip(fwd + d, -0.7, 0.7)
    cmd[[1, 3]] = np.clip(fwd - d, -0.7, 0.7)
    cmd[4:6] = float(np.clip(-0.26 + 0.8 * (DEPTH_TARGET - pos[2]) - 0.8 * dyn[5], -0.6, 0.6))
    return cmd


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "indoor_pool_frames"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 75.0
    os.makedirs(out, exist_ok=True)

    with open(SCENARIO_JSON) as f:
        scenario = json.load(f)
    scenario["world"] = "SimpleUnderwater"
    agent = scenario["agents"][0]
    agent["control_scheme"] = 2
    agent["location"] = [X_OFF + 1.0, -0.3, -0.5]
    agent["rotation"] = [0, 0, 0]
    for s in agent["sensors"]:
        if s["sensor_type"] == "DynamicsSensor":
            s["Hz"] = scenario["ticks_per_sec"]
    agent["sensors"].append({
        "sensor_type": "RGBCamera", "sensor_name": "ChaseCam",
        "location": [-0.75, 0.0, 0.32], "rotation": [0.0, 18.0, 0.0],  # short high boom: stays inside the pool at corner pivots
        "Hz": FPS, "configuration": {"CaptureWidth": WIDTH, "CaptureHeight": HEIGHT},
    })
    # stationary corner cam — KNOWN FLAKY in this tight pool (the tripod agent
    # ends up displaced; chase cam is the reliable footage source)
    scenario["agents"].append({
        "agent_name": "cam0", "agent_type": "BlueROV2",
        "control_scheme": 2,
        "location": [X_OFF + 0.5, -0.65, -0.55], "rotation": [0, 0, 18],
        "sensors": [{
            "sensor_type": "RGBCamera", "sensor_name": "WideCam",
            "location": [0.4, 0.0, 0.05], "rotation": [0.0, -5.0, 0.0],  # deeper + down-pitch: shallow cam saw only the Snell window/sky
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
        # prop spawning can shove the tripod agent — put it back afterwards
        env.agents["cam0"].teleport(location=[X_OFF + 0.5, -0.65, -0.55],
                                    rotation=[0, 0, 18])
        while t < duration:
            if last_dyn is not None:
                quat = last_dyn[DYN_QUAT]
                R = quat_to_rot_matrix_xyzw(quat)
                nu = np.concatenate([R.T @ last_dyn[3:6], R.T @ last_dyn[12:15]])
                cmd = lap_command(last_dyn, wp_state)
                nu_dot = model.step(cmd, quat, nu, z_world=float(last_dyn[8]),
                                    surface_z=WATER_SURFACE_Z)
                env.act(AGENT_NAME, np.concatenate([R @ nu_dot[:3], R @ nu_dot[3:]]))
                # physics-sleep watchdog (engine sleeps slow bodies; see work.md)
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

    with open(os.path.join(out, "indoor_pool_track.csv"), "w") as f:
        f.write("t,x,y,z\n")
        for r in track[::4]:
            f.write(",".join(f"{v:.4f}" for v in r) + "\n")

    # containment + regime stats against the EXACT pool geometry
    tr = np.array(track)
    if len(tr):
        inside = ((tr[:, 1] > X_OFF) & (tr[:, 1] < X_OFF + POOL_LEN) &
                  (np.abs(tr[:, 2]) < POOL_WID / 2)).mean()
        clear = min((tr[:, 1] - X_OFF).min(), (X_OFF + POOL_LEN - tr[:, 1]).max() and
                    (X_OFF + POOL_LEN - tr[:, 1]).min(), (POOL_WID / 2 - np.abs(tr[:, 2])).min())
        print(f"[indoor] containment {inside*100:.1f}%  min wall clearance {clear:.2f} m  "
              f"depth mean {tr[:, 3].mean():.2f} m  altitude ~{(tr[:, 3] - FLOOR_Z).mean():.2f} m")

    for name, buf in (("chase", chase), ("top", top)):
        if not buf:
            continue
        for i in (len(buf) // 4, len(buf) // 2, 3 * len(buf) // 4):
            buf[i].save(os.path.join(out, f"{name}_still_{i:04d}.png"))
        buf[0].save(os.path.join(out, f"indoor_{name}.gif"), save_all=True,
                    append_images=buf[1:], duration=int(1000 / FPS), loop=0)
    print(f"[indoor] saved {len(chase)} chase + {len(top)} top frames, "
          f"{len(track)} track points -> {out}")


if __name__ == "__main__":
    main()
