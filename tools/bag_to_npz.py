#!/usr/bin/env python3
"""Convert a real mcap bag's sensor streams to the scorecard .npz layout.

Usage: python3 bag_to_npz.py <bag_dir_or_mcap> <out.npz>
"""
import glob
import os
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def main():
    path, out = sys.argv[1], sys.argv[2]
    if os.path.isdir(path):
        path = glob.glob(os.path.join(path, "*.mcap"))[0]
    imu_rows, dvl_rows, alt_rows = [], [], []
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, m, msg in reader.iter_decoded_messages(
                topics=["/mavros/imu/data", "/dvl/twist", "/mavros/global_position/rel_alt"]):
            t = m.log_time * 1e-9
            if ch.topic == "/mavros/imu/data":
                q = msg.orientation
                imu_rows.append([t, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                 msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                                 q.x, q.y, q.z, q.w])
            elif ch.topic == "/dvl/twist":
                dvl_rows.append([t, msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            else:
                alt_rows.append([t, msg.data])
    np.savez_compressed(out, imu=np.array(imu_rows), dvl=np.array(dvl_rows), alt=np.array(alt_rows))
    print(f"[bag_to_npz] {out}: imu {len(imu_rows)}, dvl {len(dvl_rows)}, alt {len(alt_rows)}")


if __name__ == "__main__":
    main()
