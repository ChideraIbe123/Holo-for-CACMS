"""Infer the vehicle's EFFECTIVE per-axis sign (surge/sway/heave/yaw) from real
data, without the Pixhawk params. For each bag: take the real thruster commands
(rc/out via the profile npz), map to commanded body-axis force with the standard
ArduSub mixer, and correlate against the measured motion (DVL velocity, gyro yaw).

A positive correlation => the sim's assumed direction for that axis is correct.
A negative correlation => that axis is effectively flipped on the real vehicle.

Usage: python3 infer_motor_signs.py   (reads ~/data/profiles + ~/data/real_npz)
"""
import glob
import os

import numpy as np
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bluerov2_standard_model import ARDUSUB_DIR, THRUSTER_POS, t200_force

# body-axis mixer: tau[surge,sway,heave, roll,pitch,yaw] = T @ per-thruster-force
T = np.zeros((6, 6))
for i in range(6):
    T[:3, i] = ARDUSUB_DIR[i]
    T[3:, i] = np.cross(THRUSTER_POS[i], ARDUSUB_DIR[i])

PROF = os.path.expanduser("~/data/profiles")
NPZ = os.path.expanduser("~/data/real_npz")

axes = {"surge (DVL x)": (0, "dvl", 1),
        "sway (DVL y)":  (1, "dvl", 2),
        "heave (DVL z)": (2, "dvl", 3),
        "yaw (gyro z)":  (5, "imu", 3)}

agg = {k: [] for k in axes}
for prof in sorted(glob.glob(os.path.join(PROF, "*.npz"))):
    name = os.path.basename(prof)
    npz = os.path.join(NPZ, name)
    if not os.path.exists(npz):
        continue
    p = np.load(prof); d = np.load(npz)
    tcmd, cmd = p["t"], p["cmd"]
    # commanded body-axis force at each command row
    tau = np.array([T @ t200_force(c) for c in cmd])   # N x 6
    for label, (col, arr_key, mcol) in axes.items():
        arr = d[arr_key]
        if len(arr) < 20:
            continue
        tm = arr[:, 0] - arr[0, 0]
        meas = arr[:, mcol]
        # command is zero-order-hold; sample commanded tau at each measurement time
        idx = np.clip(np.searchsorted(tcmd, tm, side="right") - 1, 0, len(tau) - 1)
        c_axis = tau[idx, col]
        if np.std(c_axis) < 1e-6 or np.std(meas) < 1e-6:
            continue
        r = np.corrcoef(c_axis, meas)[0, 1]
        agg[label].append(r)

print(f"{'axis':<16}{'n bags':>7}{'mean corr':>11}{'verdict':>26}")
for label, rs in agg.items():
    if not rs:
        print(f"{label:<16}{'0':>7}   (no excitation)")
        continue
    m = float(np.mean(rs))
    pos = sum(1 for r in rs if r > 0)
    verdict = "sign OK" if m > 0.15 else ("FLIPPED" if m < -0.15 else "inconclusive (weak)")
    print(f"{label:<16}{len(rs):>7}{m:>+11.2f}   {verdict:>23} ({pos}/{len(rs)} bags +)")
