"""Capture chase-cam footage of the simulated BlueROV2 (headless — HoloOcean
renders camera sensors offscreen). Saves PNG frames + an animated GIF.

Usage: python3 capture_video.py <out_dir> [duration_s]
"""
import copy
import json
import os
import sys

import numpy as np
import holoocean
from PIL import Image

from bluerov2_standard_model import BlueROV2StandardModel
from mavros_bridge import (SCENARIO_JSON, AGENT_NAME, WATER_SURFACE_Z,
                           scripted_command6, quat_to_rot_matrix_xyzw, DYN_QUAT)

FPS = 5
WIDTH, HEIGHT = 960, 540


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "frames"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
    os.makedirs(out, exist_ok=True)

    with open(SCENARIO_JSON) as f:
        scenario = json.load(f)
    agent = scenario["agents"][0]
    agent["control_scheme"] = 2
    for s in agent["sensors"]:
        if s["sensor_type"] == "DynamicsSensor":
            s["Hz"] = scenario["ticks_per_sec"]
    # chase camera: behind and above the vehicle, pitched down
    agent["sensors"].append({
        "sensor_type": "RGBCamera",
        "sensor_name": "ChaseCam",
        "location": [-2.2, 0.0, 1.0],
        "rotation": [0.0, 25.0, 0.0],
        "Hz": FPS,
        "configuration": {"CaptureWidth": WIDTH, "CaptureHeight": HEIGHT},
    })

    model = BlueROV2StandardModel()
    ticks_per_sec = float(scenario["ticks_per_sec"])
    frames = []
    t, last_dyn = 0.0, None

    with holoocean.make(scenario_cfg=scenario, show_viewport=False) as env:
        while t < duration:
            if last_dyn is not None:
                quat = last_dyn[DYN_QUAT]
                R = quat_to_rot_matrix_xyzw(quat)
                nu = np.concatenate([R.T @ last_dyn[3:6], R.T @ last_dyn[12:15]])
                cmd6 = scripted_command6(t, z=float(last_dyn[8]), w_vert=float(last_dyn[5]))
                nu_dot = model.step(cmd6, quat, nu, z_world=float(last_dyn[8]),
                                    surface_z=WATER_SURFACE_Z)
                env.act(AGENT_NAME, np.concatenate([R @ nu_dot[:3], R @ nu_dot[3:]]))
            state = env.tick()
            t = float(state.get("t", t + 1.0 / ticks_per_sec))
            if "DynamicsSensor" in state:
                last_dyn = np.asarray(state["DynamicsSensor"], dtype=float)
            if "ChaseCam" in state:
                img = np.asarray(state["ChaseCam"])[:, :, :3][:, :, ::-1]  # BGRA->RGB
                frames.append(Image.fromarray(img.astype(np.uint8)))
                if len(frames) % 25 == 0:
                    print(f"[capture] {len(frames)} frames, sim t={t:.1f}s")

    for i in (0, len(frames) // 3, 2 * len(frames) // 3, len(frames) - 1):
        frames[i].save(os.path.join(out, f"still_{i:04d}.png"))
    frames[0].save(os.path.join(out, "chase_cam.gif"), save_all=True,
                   append_images=frames[1:], duration=int(1000 / FPS), loop=0)
    print(f"[capture] saved {len(frames)} frames -> {out}/chase_cam.gif + stills")


if __name__ == "__main__":
    main()
