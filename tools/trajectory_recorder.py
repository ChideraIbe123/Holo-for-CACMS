#!/usr/bin/env python3
"""Record trajectories to CSV: our pipeline's /deadreckon/odom and the DVL
vendor's onboard /dvl/dead_reckoning/pose. On SIGINT/SIGTERM (rclpy shuts spin
down internally; do NOT rely on custom signal handlers), saves CSVs and prints a
comparison: both trajectories aligned to their own starting pose (translation,
then optimal yaw about z), displacement stats and error.

Usage: python3 trajectory_recorder.py <out_dir>
"""
import sys
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else '.')
out_dir.mkdir(parents=True, exist_ok=True)

ours, vendor = [], []


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    rclpy.init()
    node = rclpy.create_node('trajectory_recorder')
    node.create_subscription(
        Odometry, '/deadreckon/odom',
        lambda m: ours.append((stamp_s(m.header), m.pose.pose.position.x,
                               m.pose.pose.position.y, m.pose.pose.position.z)), 50)
    node.create_subscription(
        PoseStamped, '/dvl/dead_reckoning/pose',
        lambda m: vendor.append((stamp_s(m.header), m.pose.position.x,
                                 m.pose.position.y, m.pose.position.z)), 50)

    try:
        rclpy.spin(node)          # returns when rclpy handles SIGINT/SIGTERM
    except (KeyboardInterrupt, Exception):
        pass

    for name, rows in (('ours', ours), ('vendor', vendor)):
        with open(out_dir / f'{name}.csv', 'w') as f:
            f.write('t,x,y,z\n')
            for r in rows:
                f.write(','.join(f'{v:.6f}' for v in r) + '\n')
    print(f'\n[recorder] saved {len(ours)} ours / {len(vendor)} vendor samples to {out_dir}', flush=True)
    analyze()


def analyze():
    if len(ours) < 10 or len(vendor) < 10:
        print('[recorder] not enough data to compare')
        return
    a = np.array(ours, dtype=float)
    b = np.array(vendor, dtype=float)
    # zero each trajectory at its own start
    a[:, 1:] -= a[0, 1:]
    b[:, 1:] -= b[0, 1:]
    # resample vendor onto our timestamps (clip to overlap)
    t0, t1 = max(a[0, 0], b[0, 0]), min(a[-1, 0], b[-1, 0])
    am = a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]
    bi = np.stack([np.interp(am[:, 0], b[:, 0], b[:, k]) for k in (1, 2, 3)], axis=1)
    ai = am[:, 1:]
    # optimal yaw alignment about z (xy Procrustes, rotation only)
    num = float(np.sum(ai[:, 0] * bi[:, 1] - ai[:, 1] * bi[:, 0]))
    den = float(np.sum(ai[:, 0] * bi[:, 0] + ai[:, 1] * bi[:, 1]))
    yaw = np.arctan2(num, den)
    c, s = np.cos(yaw), np.sin(yaw)
    a_rot = ai.copy()
    a_rot[:, 0] = c * ai[:, 0] - s * ai[:, 1]
    a_rot[:, 1] = s * ai[:, 0] + c * ai[:, 1]

    err_xy = np.linalg.norm(a_rot[:, :2] - bi[:, :2], axis=1)
    path_len = float(np.sum(np.linalg.norm(np.diff(bi[:, :2], axis=0), axis=1)))
    print(f'[compare] overlap {t1 - t0:.1f} s, {len(ai)} samples, vendor xy path length {path_len:.2f} m')
    print(f'[compare] yaw offset between frames: {np.degrees(yaw):+.1f} deg')
    print(f'[compare] ours final displacement xy: {np.linalg.norm(ai[-1, :2]):.2f} m | vendor: {np.linalg.norm(bi[-1, :2]):.2f} m')
    print(f'[compare] xy error after yaw alignment: rms {err_xy.std() + err_xy.mean():.3f} m '
          f'(mean {err_xy.mean():.3f}, max {err_xy.max():.3f})')
    print(f'[compare] final z: ours {ai[-1, 2]:+.3f} vendor {bi[-1, 2]:+.3f}')


if __name__ == '__main__':
    main()
