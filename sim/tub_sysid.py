#!/usr/bin/env python3
"""Steady-state magnitude validation of the dynamics model on the 2026-09-17
single-axis tub runs — the first data that excites sway, heave, and yaw.

Method: find windows where the thruster PWMs (rc/out, 2 Hz — fine for the
sustained holds these runs were flown for) are roughly constant for >= MIN_HOLD
seconds. For each window: measured speed = mean |DVL / gyro_z| on the excited
axis; predicted speed = the model's steady state at that command level (thrust
through the ArduSub mixer balanced against quadratic drag). Signs were already
validated (surge +0.83 / sway +0.89 / yaw -0.97); this checks MAGNITUDES.

Usage: tub_sysid.py <bag.mcap> <axis>   # axis: surge|sway|heave|yaw
"""
import glob
import os
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bluerov2_standard_model import (BlueROV2StandardModel, t200_force,
                                     THRUST_SCALE, DRAG_SCALE, D_LIN, D_QUAD)

MIN_HOLD = 2.5     # s of ~constant command
PWM_TOL = 30       # PWM counts considered "constant"
MIN_LEVEL = 0.08   # normalized |cmd| worth analyzing

AXIS = {"surge": ("dvl", 0, 0), "sway": ("dvl", 1, 1),
        "heave": ("dvl", 2, 2), "yaw": ("gyro", 2, 5)}   # (meas src, col, tau idx)


def read_bag(path):
    if os.path.isdir(path):
        path = glob.glob(os.path.join(path, "*.mcap"))[0]
    pwm, dvl, gyro = [], [], []
    with open(path, "rb") as f:
        r = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, m, msg in r.iter_decoded_messages(
                topics=["/mavros/rc/out", "/dvl/twist", "/mavros/imu/data"]):
            t = m.log_time * 1e-9
            if ch.topic == "/mavros/rc/out":
                pwm.append([t] + [float(msg.channels[i]) for i in range(6)])
            elif ch.topic == "/dvl/twist":
                dvl.append([t, msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            else:
                gyro.append([t, msg.angular_velocity.x, msg.angular_velocity.y,
                             msg.angular_velocity.z])
    return np.array(pwm), np.array(dvl), np.array(gyro)


def steady_windows(pwm):
    """[(t0, t1, cmd6)] where all 6 PWMs stay within PWM_TOL for >= MIN_HOLD."""
    out, i = [], 0
    while i < len(pwm):
        j = i
        while j + 1 < len(pwm) and np.abs(pwm[j + 1, 1:] - pwm[i, 1:]).max() < PWM_TOL:
            j += 1
        if pwm[j, 0] - pwm[i, 0] >= MIN_HOLD:
            cmd = np.clip((pwm[i:j + 1, 1:].mean(0) - 1500.0) / 400.0, -1, 1)
            out.append((pwm[i, 0], pwm[j, 0], cmd))
        i = j + 1
    return out


# THIS vehicle's motor directions live in its ESC/param config, so logged PWMs
# are direction-less: pure surge logs EQUAL + on all four vectored thrusters
# (AP_Motors6DOF's signed map cancels them), and pure heave logs OPPOSITE signs
# on the vertical pair (one vertical is flipped in config). Derived from the
# 2026-09-17 single-axis runs — this IS the MOT_n_DIRECTION info, from data.
VEHICLE_DIR = np.array([1, 1, 1, 1, 1, -1])


def predict_steady(model, cmd6, tau_idx):
    """Model steady-state speed on one axis: mixer thrust vs quadratic drag."""
    tau = THRUST_SCALE * (model.T @ t200_force(cmd6 * VEHICLE_DIR))
    f = tau[tau_idx]
    a = DRAG_SCALE * D_QUAD[tau_idx]
    b = DRAG_SCALE * D_LIN[tau_idx]
    if abs(f) < 1e-6:
        return 0.0
    # solve f = (b + a*|v|)*v  ->  a v^2 + b v - |f| = 0 (v >= 0)
    v = (-b + np.sqrt(b * b + 4 * a * abs(f))) / (2 * a) if a > 0 else abs(f) / b
    return float(np.sign(f) * v)


def main():
    path, axis = sys.argv[1], sys.argv[2]
    src, col, tau_idx = AXIS[axis]
    pwm, dvl, gyro = read_bag(path)
    meas_t, meas_v = (dvl[:, 0], dvl[:, 1 + col]) if src == "dvl" else \
                     (gyro[:, 0], gyro[:, 1 + col])
    model = BlueROV2StandardModel()
    print(f"{os.path.basename(path.rstrip('/'))} [{axis}]: {len(pwm)} pwm, "
          f"{len(steady_windows(pwm))} steady windows")
    print(f"{'t0':>7} {'hold':>5} {'|cmd|':>6} {'measured':>9} {'model':>8} {'err%':>7}")
    rows = []
    for t0, t1, cmd in steady_windows(pwm):
        if np.abs(cmd).max() < MIN_LEVEL:
            continue
        sel = (meas_t >= t0 + 0.8) & (meas_t <= t1)   # skip transient
        if sel.sum() < 5:
            continue
        m = float(np.mean(np.abs(meas_v[sel])))
        p = abs(predict_steady(model, cmd, tau_idx))
        if p < 1e-4 and m < 0.02:
            continue
        err = 100 * (p - m) / max(m, 1e-6)
        rows.append((m, p))
        print(f"{t0 % 1000:7.1f} {t1 - t0:5.1f} {np.abs(cmd).max():6.2f} "
              f"{m:9.3f} {p:8.3f} {err:+6.0f}%")
    if rows:
        m, p = np.array(rows).T
        keep = m > 0.03
        if keep.sum() >= 2:
            ratio = np.median(p[keep] / m[keep])
            print(f"median model/measured ratio (excited windows): {ratio:.2f}")


if __name__ == "__main__":
    main()
