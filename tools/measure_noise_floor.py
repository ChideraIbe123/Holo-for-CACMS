#!/usr/bin/env python3
"""Measure per-sample noise floors of the live sim topics with the SAME
successive-difference estimator used on the real bag (std(diff)/sqrt(2) —
cancels smooth motion, keeps white noise), and compare against the real-vehicle
targets. Run alongside the bridge, ideally with the vehicle idle.

Usage: python3 measure_noise_floor.py [seconds]   (default 60)
"""
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

# Real-vehicle targets (successive-diff floors from rosbag_20260504_154118)
TARGET = {
    'gyro (rad/s)': [0.027, 0.024, 0.131],
    'accel (m/s^2)': [0.203, 0.104, 0.050],
    'rpy (deg)': [0.32, 0.34, 1.39],
    'dvl vel (m/s)': [0.030, 0.032, 0.010],
    'rel_alt (m)': [0.0098],
}

imu_rows, dvl_rows, alt_rows = [], [], []


def on_imu(m):
    q = m.orientation
    imu_rows.append([
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
        q.x, q.y, q.z, q.w,
    ])


def diffstd(a):
    return np.diff(a, axis=0).std(axis=0) / np.sqrt(2)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    node = rclpy.create_node('noise_floor_meter')
    node.create_subscription(Imu, '/mavros/imu/data', on_imu, 50)
    node.create_subscription(TwistStamped, '/dvl/twist',
                             lambda m: dvl_rows.append([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]), 50)
    node.create_subscription(Float64, '/mavros/global_position/rel_alt',
                             lambda m: alt_rows.append([m.data]), 50)

    deadline = time.time() + duration
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    imu = np.array(imu_rows)
    if len(imu) < 20:
        print(f'not enough IMU data ({len(imu)} msgs)')
        return
    gyro, accel, quat = imu[:, 0:3], imu[:, 3:6], imu[:, 6:10]
    x, y, z, w = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    measured = {
        'gyro (rad/s)': diffstd(gyro),
        'accel (m/s^2)': diffstd(accel),
        'rpy (deg)': np.degrees([diffstd(roll[:, None])[0], diffstd(pitch[:, None])[0], diffstd(yaw[:, None])[0]]),
        'dvl vel (m/s)': diffstd(np.array(dvl_rows)) if len(dvl_rows) > 20 else None,
        'rel_alt (m)': diffstd(np.array(alt_rows)) if len(alt_rows) > 20 else None,
    }

    print(f'\nsamples: imu {len(imu_rows)}, dvl {len(dvl_rows)}, rel_alt {len(alt_rows)}')
    print(f'{"quantity":<16}{"sim measured":<28}{"real target":<28}ratio')
    for k, tgt in TARGET.items():
        m = measured[k]
        if m is None:
            print(f'{k:<16}(insufficient data)')
            continue
        m = np.atleast_1d(m)
        ratio = ' '.join(f'{a / b:.2f}' for a, b in zip(m, tgt))
        print(f'{k:<16}{np.round(m, 5)!s:<28}{tgt!s:<28}{ratio}')


if __name__ == '__main__':
    main()
