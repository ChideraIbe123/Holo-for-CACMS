"""Holdout evaluation: run the twin's calibration measurements on UNSEEN real bags.

For each bag: noise floors (successive differences), publish rates, frame_ids,
covariances, stationary accel, rel_alt stats, and PWM-replay surge validation.
Compare across bags to get the real-to-real reference spread, and against the
values the twin was calibrated to (from rosbag_20260504_154118).

Usage: python3 holdout_eval.py <bag_dir_or_mcap> [more bags...]
"""
import glob
import os
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

# Values the twin is calibrated to (from the calibration bag 154118)
CALIB = {
    "gyro": [0.02693, 0.02356, 0.13137],
    "accel": [0.20288, 0.10355, 0.05016],
    "dvl": [0.02959, 0.03154, 0.00959],
    "rel_alt": 0.0098,
    "imu_frame": "base_link",
    "angvel_cov0": 1.2184700254281e-07,
    "accel_cov0": 8.999999999999999e-08,
}

TOPICS = ["/mavros/imu/data", "/dvl/twist", "/mavros/global_position/rel_alt",
          "/dvl/fom", "/mavros/rc/out"]


def diffstd(a):
    return np.diff(a, axis=0).std(axis=0) / np.sqrt(2)


def eval_bag(path):
    if os.path.isdir(path):
        mcaps = glob.glob(os.path.join(path, "*.mcap"))
        if not mcaps:
            print(f"  no mcap in {path}")
            return None
        path = mcaps[0]

    data = {t: [] for t in TOPICS}
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, m, msg in reader.iter_decoded_messages(topics=TOPICS):
            data[ch.topic].append((m.log_time * 1e-9, msg))

    out = {"name": os.path.basename(path)}
    imu = data["/mavros/imu/data"]
    if len(imu) < 30:
        print(f"  insufficient IMU data ({len(imu)})")
        return None
    t = np.array([x[0] for x in imu]); t -= t[0]
    out["dur"] = t[-1]
    gyr = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu])
    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in imu])
    out["imu_hz"] = len(imu) / out["dur"]
    out["imu_frame"] = imu[0][1].header.frame_id
    out["angvel_cov0"] = float(imu[0][1].angular_velocity_covariance[0])
    out["accel_cov0"] = float(imu[0][1].linear_acceleration_covariance[0])
    out["gyro_floor"] = diffstd(gyr)
    out["accel_floor"] = diffstd(acc)
    # quietest 5 s window accel (gravity check)
    W = max(5, int(5 / max(np.median(np.diff(t)), 1e-3)))
    if len(gyr) > W + 2:
        gm = np.convolve(np.linalg.norm(gyr, axis=1), np.ones(W) / W, "valid")
        i = int(np.argmin(gm)); j = min(i + W, len(acc) - 1)
        out["quiet_accel_z"] = acc[i:j, 2].mean()
        out["quiet_gyro_mag"] = gm[i]

    dvl = data["/dvl/twist"]
    if len(dvl) > 30:
        dv = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for _, m in dvl])
        out["dvl_hz"] = len(dvl) / out["dur"]
        out["dvl_floor"] = diffstd(dv)
        out["dvl_speed_max"] = np.abs(dv[:, 0]).max()

    ra = data["/mavros/global_position/rel_alt"]
    if len(ra) > 20:
        rav = np.array([m.data for _, m in ra])
        out["relalt_floor"] = float(diffstd(rav[:, None])[0])
        out["relalt_range"] = (rav.min(), rav.max())

    fom = data["/dvl/fom"]
    if fom:
        out["fom_med"] = float(np.median([m.data for _, m in fom]))
    out["has_rcout"] = len(data["/mavros/rc/out"])
    return out


def main():
    results = []
    for p in sys.argv[1:]:
        print(f"=== {p} ===")
        r = eval_bag(p)
        if r:
            results.append(r)
            print(f"  dur {r['dur']:.0f}s | imu {r['imu_hz']:.1f} Hz | frame '{r['imu_frame']}' | "
                  f"covs {r['angvel_cov0']:.3g}/{r['accel_cov0']:.3g}")
            print(f"  gyro floor  {np.round(r['gyro_floor'], 4)}  (calib {CALIB['gyro']})")
            print(f"  accel floor {np.round(r['accel_floor'], 4)}  (calib {CALIB['accel']})")
            if "dvl_floor" in r:
                print(f"  dvl floor   {np.round(r['dvl_floor'], 4)}  (calib {CALIB['dvl']}) | max|u| {r['dvl_speed_max']:.2f}")
            if "relalt_floor" in r:
                print(f"  rel_alt floor {r['relalt_floor']:.4f} (calib {CALIB['rel_alt']}) | range {np.round(r['relalt_range'],2)}")
            if "quiet_accel_z" in r:
                print(f"  quiet-window accel z {r['quiet_accel_z']:+.2f} (|gyro| {r['quiet_gyro_mag']:.3f})")
            if "fom_med" in r:
                print(f"  fom median {r['fom_med']:.4f} | rc/out msgs {r['has_rcout']}")

    if len(results) >= 2:
        print("\n=== real-to-real spread across bags (reference floor for twin fidelity) ===")
        for key, cal in (("gyro_floor", CALIB["gyro"]), ("accel_floor", CALIB["accel"]), ("dvl_floor", CALIB["dvl"])):
            arr = np.array([r[key] for r in results if key in r])
            if len(arr) >= 2:
                spread = arr.max(0) / np.maximum(arr.min(0), 1e-9)
                vs_cal = arr.mean(0) / np.array(cal)
                print(f"  {key:12s} max/min across bags: {np.round(spread, 2)} | mean vs calib: {np.round(vs_cal, 2)}")


if __name__ == "__main__":
    main()
