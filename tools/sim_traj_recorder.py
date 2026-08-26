#!/usr/bin/env python3
"""Record sim dead-reckon odom vs sim ground truth to CSVs (both nav_msgs/Odometry).

Usage: python3 sim_traj_recorder.py <out_dir> — Ctrl-C/SIGINT to stop & save.
"""
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry

out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else '.')
out_dir.mkdir(parents=True, exist_ok=True)
dr, gt = [], []


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    rclpy.init()
    node = rclpy.create_node('sim_traj_recorder')
    node.create_subscription(Odometry, '/deadreckon/odom',
                             lambda m: dr.append((stamp_s(m.header), m.pose.pose.position.x,
                                                  m.pose.pose.position.y, m.pose.pose.position.z)), 50)
    node.create_subscription(Odometry, '/holoocean/ground_truth',
                             lambda m: gt.append((stamp_s(m.header), m.pose.pose.position.x,
                                                  m.pose.pose.position.y, m.pose.pose.position.z)), 50)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    for name, rows in (('sim_dr', dr), ('sim_gt', gt)):
        with open(out_dir / f'{name}.csv', 'w') as f:
            f.write('t,x,y,z\n')
            for r in rows:
                f.write(','.join(f'{v:.6f}' for v in r) + '\n')
    print(f'[recorder] saved {len(dr)} dr / {len(gt)} gt samples to {out_dir}', flush=True)


if __name__ == '__main__':
    main()
