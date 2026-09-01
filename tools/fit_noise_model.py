"""Fit the excitation-dependent noise model from real bags.

For every 5 s window in every bag: thruster effort E = mean |pwm-1500|/400 over
channels 1-6 (zero-order hold), and per-channel noise floor (successive-diff
std / sqrt(2)). Fits sigma(E) = a + b*E per channel by least squares.

Usage: python3 fit_noise_model.py <bag_dir_or_mcap> [...]
Prints the coefficient table to paste into mavros_bridge.py.
"""
import glob
import os
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

WIN = 5.0
CHANNELS = ["gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z",
            "dvl_x", "dvl_y", "dvl_z"]


def load(path):
    if os.path.isdir(path):
        path = glob.glob(os.path.join(path, "*.mcap"))[0]
    imu, dvl, pwm = [], [], []
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, m, msg in reader.iter_decoded_messages(
                topics=["/mavros/imu/data", "/dvl/twist", "/mavros/rc/out"]):
            t = m.log_time * 1e-9
            if ch.topic == "/mavros/imu/data":
                imu.append([t, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            elif ch.topic == "/dvl/twist":
                dvl.append([t, msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            else:
                pwm.append([t] + [float(msg.channels[i]) for i in range(6)])
    return np.array(imu), np.array(dvl), np.array(pwm)


def main():
    E_all, sig_all = [], {c: [] for c in CHANNELS}
    for path in sys.argv[1:]:
        imu, dvl, pwm = load(path)
        if len(imu) < 30 or len(pwm) < 3:
            continue
        t0, t1 = imu[0, 0], imu[-1, 0]
        for ws in np.arange(t0, t1 - WIN, WIN / 2):
            mi = imu[(imu[:, 0] >= ws) & (imu[:, 0] < ws + WIN)]
            md = dvl[(dvl[:, 0] >= ws) & (dvl[:, 0] < ws + WIN)]
            if len(mi) < 15:
                continue
            # effort: ZOH pwm at window midpoint region
            idx = np.searchsorted(pwm[:, 0], ws + WIN / 2, side="right") - 1
            eff = np.abs(pwm[max(idx, 0), 1:7] - 1500.0).mean() / 400.0
            E_all.append(eff)
            d = np.diff(mi[:, 1:7], axis=0).std(axis=0) / np.sqrt(2)
            for k, c in enumerate(CHANNELS[:6]):
                sig_all[c].append(d[k])
            if len(md) > 10:
                dd = np.diff(md[:, 1:4], axis=0).std(axis=0) / np.sqrt(2)
                for k, c in enumerate(CHANNELS[6:]):
                    sig_all[c].append(dd[k])
            else:
                for c in CHANNELS[6:]:
                    sig_all[c].append(np.nan)

    E = np.array(E_all)
    print(f"windows: {len(E)}, effort range {E.min():.3f}..{E.max():.3f}, median {np.median(E):.3f}")
    print(f"{'channel':<9}{'a (sigma@0)':>12}{'b (slope)':>12}{'corr(E,sig)':>12}")
    print("NOISE_MODEL = {")
    for c in CHANNELS:
        s = np.array(sig_all[c])
        ok = ~np.isnan(s)
        e, sv = E[ok], s[ok]
        A = np.vstack([np.ones_like(e), e]).T
        (a, b), *_ = np.linalg.lstsq(A, sv, rcond=None)
        r = np.corrcoef(e, sv)[0, 1]
        print(f"    '{c}': ({max(a,0):.5f}, {b:.5f}),   # corr {r:+.2f}")
    print("}")


if __name__ == "__main__":
    main()
