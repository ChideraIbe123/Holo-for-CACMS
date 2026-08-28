#!/usr/bin/env python3
"""Record one run's sensor streams (IMU, DVL, rel_alt) to a compressed .npz,
in the same layout bag_to_npz.py produces for real bags.

Usage: python3 record_run.py <out.npz> <seconds>
"""
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

imu_rows, dvl_rows, alt_rows = [], [], []


def main():
    out, dur = sys.argv[1], float(sys.argv[2])
    rclpy.init()
    node = rclpy.create_node("run_recorder")

    def on_imu(m):
        q = m.orientation
        imu_rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                         m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                         q.x, q.y, q.z, q.w])

    node.create_subscription(Imu, "/mavros/imu/data", on_imu, 50)
    node.create_subscription(TwistStamped, "/dvl/twist",
                             lambda m: dvl_rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                                        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]), 50)
    node.create_subscription(Float64, "/mavros/global_position/rel_alt",
                             lambda m: alt_rows.append([time.time(), m.data]), 50)

    deadline = time.time() + dur
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    np.savez_compressed(out, imu=np.array(imu_rows), dvl=np.array(dvl_rows), alt=np.array(alt_rows))
    print(f"[record] {out}: imu {len(imu_rows)}, dvl {len(dvl_rows)}, alt {len(alt_rows)}")


if __name__ == "__main__":
    main()
