"""Extract a real bag's thruster command profile for sim replay.

Saves: t (s, from 0), cmd (N x 6 normalized [-1,1], ArduSub channel order),
z0 (initial depth from rel_alt), and t_gz/gz (measured yaw rate on the same
clock — the replay yaw-rate servo tracks it because the geometric mixer can't
reproduce yaw from PWMs).

Usage: python3 extract_cmd_profile.py <bag_dir_or_mcap> <out.npz>
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
    pwm, relalt, gyro = [], [], []
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, m, msg in reader.iter_decoded_messages(
                topics=["/mavros/rc/out", "/mavros/global_position/rel_alt",
                        "/mavros/imu/data"]):
            t = m.log_time * 1e-9
            if ch.topic == "/mavros/rc/out":
                pwm.append([t] + [float(msg.channels[i]) for i in range(6)])
            elif ch.topic == "/mavros/imu/data":
                gyro.append([t, float(msg.angular_velocity.x),
                             float(msg.angular_velocity.y),
                             float(msg.angular_velocity.z)])
            else:
                relalt.append([t, msg.data])
    pwm = np.array(pwm)
    gyro = np.array(gyro)
    t0 = pwm[0, 0]
    cmd = np.clip((pwm[:, 1:7] - 1500.0) / 400.0, -1, 1)
    z0 = float(relalt[0][1]) if relalt else -0.5
    relalt = np.array(relalt) if relalt else np.zeros((0, 2))
    np.savez_compressed(out, t=pwm[:, 0] - t0, cmd=cmd, z0=z0,
                        t_gz=gyro[:, 0] - t0, gz=gyro[:, 3],
                        gx=gyro[:, 1], gy=gyro[:, 2],
                        t_alt=relalt[:, 0] - t0 if len(relalt) else relalt[:, :0],
                        alt=relalt[:, 1] if len(relalt) else relalt[:, :0])
    print(f"[profile] {out}: {len(pwm)} cmds over {pwm[-1,0]-t0:.0f}s, z0={z0:.2f}, "
          f"mean effort {np.abs(cmd).mean():.3f}, {len(gyro)} yaw-rate refs")


if __name__ == "__main__":
    main()
