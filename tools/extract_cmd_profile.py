"""Extract a real bag's thruster command profile for sim replay.

Saves: t (s, from 0), cmd (N x 6 normalized [-1,1], ArduSub channel order),
z0 (initial depth from rel_alt).

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
    pwm, relalt = [], []
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, m, msg in reader.iter_decoded_messages(
                topics=["/mavros/rc/out", "/mavros/global_position/rel_alt"]):
            t = m.log_time * 1e-9
            if ch.topic == "/mavros/rc/out":
                pwm.append([t] + [float(msg.channels[i]) for i in range(6)])
            else:
                relalt.append([t, msg.data])
    pwm = np.array(pwm)
    t0 = pwm[0, 0]
    cmd = np.clip((pwm[:, 1:7] - 1500.0) / 400.0, -1, 1)
    z0 = float(relalt[0][1]) if relalt else -0.5
    np.savez_compressed(out, t=pwm[:, 0] - t0, cmd=cmd, z0=z0)
    print(f"[profile] {out}: {len(pwm)} cmds over {pwm[-1,0]-t0:.0f}s, z0={z0:.2f}, "
          f"mean effort {np.abs(cmd).mean():.3f}")


if __name__ == "__main__":
    main()
