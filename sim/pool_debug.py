"""Minimal corner-turn diagnostic: spawn at the lap corner, command one waypoint
90 degrees to the left, log heading dynamics every tick. No cameras."""
import json
import math
import sys

import numpy as np
import holoocean

from bluerov2_standard_model import BlueROV2StandardModel
from mavros_bridge import SCENARIO_JSON, AGENT_NAME, WATER_SURFACE_Z, quat_to_rot_matrix_xyzw, DYN_QUAT
from pool_capture import spawn_pool

WPS = [(40.0,-3.0),(54.0,-3.0),(54.0,3.0),(40.0,3.0)]


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


with open(SCENARIO_JSON) as f:
    scenario = json.load(f)
agent = scenario["agents"][0]
agent["control_scheme"] = 2
agent["location"] = [40.0, -3.0, -0.65]
agent["rotation"] = [0, 0, 0]
for s in agent["sensors"]:
    if s["sensor_type"] == "DynamicsSensor":
        s["Hz"] = scenario["ticks_per_sec"]

model = BlueROV2StandardModel()
rows, t, last_dyn, wpi = [], 0.0, None, [0]
freeze_hist, wakes = [], 0
with holoocean.make(scenario_cfg=scenario, show_viewport=False) as env:
    if "--props" in sys.argv:
        spawn_pool(env)
    while t < 70.0:
        if last_dyn is not None:
            quat = last_dyn[DYN_QUAT]
            R = quat_to_rot_matrix_xyzw(quat)
            nu = np.concatenate([R.T @ last_dyn[3:6], R.T @ last_dyn[12:15]])
            pos = last_dyn[6:9]
            wp = WPS[wpi[0]]
            if math.hypot(wp[0]-pos[0], wp[1]-pos[1]) < 1.2:
                wpi[0] = (wpi[0] + 1) % len(WPS)
                wp = WPS[wpi[0]]
            err = math.atan2(wp[1] - pos[1], wp[0] - pos[0]) - yaw_of(quat)
            err = math.atan2(math.sin(err), math.cos(err))
            r = last_dyn[14]
            d = float(np.clip(1.6 * err - 0.9 * r, -0.45, 0.45))
            fwd = 0.33 if abs(err) < 0.9 else 0.18
            cmd = np.zeros(6)
            cmd[[0, 2]] = np.clip(fwd + d, -0.8, 0.8)
            cmd[[1, 3]] = np.clip(fwd - d, -0.8, 0.8)
            cmd[4:6] = float(np.clip(-0.26 + 0.8 * (-0.6 - pos[2]) - 0.8 * last_dyn[5], -0.6, 0.6))
            nu_dot = model.step(cmd, quat, nu, z_world=float(pos[2]), surface_z=WATER_SURFACE_Z)
            env.act(AGENT_NAME, np.concatenate([R @ nu_dot[:3], R @ nu_dot[3:]]))
            # physics-sleep watchdog: if commanded to move but frozen, wake with a teleport
            freeze_hist.append(pos.copy())
            if len(freeze_hist) > 100:
                freeze_hist.pop(0)
                if np.linalg.norm(pos - freeze_hist[0]) < 0.002 and abs(fwd) > 0.05:
                    env.agents[AGENT_NAME].teleport(location=pos + np.array([0, 0, 0.003]))
                    wakes += 1
                    freeze_hist.clear()
            rows.append([t, pos[0], pos[1], math.degrees(yaw_of(quat)), math.degrees(err), r, wpi[0]])
        state = env.tick()
        t = float(state.get("t", t))
        if "DynamicsSensor" in state:
            last_dyn = np.asarray(state["DynamicsSensor"], dtype=float)

a = np.array(rows)
np.savetxt(sys.argv[1] if len(sys.argv) > 1 else "corner_debug.csv", a,
           delimiter=",", header="t,x,y,yaw_deg,err_deg,r,wp", comments="")
print(f'wake-ups: {wakes}')
for i in range(0, len(a), 400):
    print(f"t={a[i,0]:5.1f}  pos=({a[i,1]:5.1f},{a[i,2]:5.1f})  yaw={a[i,3]:7.1f}  err={a[i,4]:7.1f}  wp={int(a[i,6])}")
