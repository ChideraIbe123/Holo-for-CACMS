#!/usr/bin/env python3
"""Decompose the 2026-09-17 tub-test float recordings into the noise model's
ingredients: SENSOR noise (unarmed float) vs sensor+VIBRATION (armed float).

For each channel (gyro/accel/dvl xyz): successive-difference std, increment
lag-1 autocorrelation, and the vibration component (quadratic difference of
armed vs unarmed stds). Compared against the sim's currently injected sigmas —
these numbers are the principled replacement for every guessed noise constant.

Usage: tubtest_floats.py <unarmed.npz> <armed.npz>
"""
import sys

import numpy as np

CH = [("gyro_x", "imu", 1), ("gyro_y", "imu", 2), ("gyro_z", "imu", 3),
      ("accel_x", "imu", 4), ("accel_y", "imu", 5), ("accel_z", "imu", 6),
      ("dvl_x", "dvl", 1), ("dvl_y", "dvl", 2), ("dvl_z", "dvl", 3)]

# sim's injected floors at zero excitation (NOISE_MODEL a-terms / accel consts)
SIM_A = {"gyro_x": 0.00635, "gyro_y": 0.01378, "gyro_z": 0.03313,
         "accel_x": 0.184, "accel_y": 0.15, "accel_z": 0.05,
         "dvl_x": 0.01983, "dvl_y": 0.01757, "dvl_z": 0.01061}


def stats(a, col):
    x = a[:, col]
    d = np.diff(x)
    sd = float(np.std(d) / np.sqrt(2))
    ac = float(np.corrcoef(d[:-1], d[1:])[0, 1]) if len(d) > 10 else float("nan")
    return sd, ac


def main():
    un = np.load(sys.argv[1])
    ar = np.load(sys.argv[2])
    print(f"unarmed: imu {len(un['imu'])} rows, dvl {len(un['dvl'])} | "
          f"armed: imu {len(ar['imu'])} rows, dvl {len(ar['dvl'])}")
    print(f"\n{'channel':<9}{'SENSOR sd':>11}{'ARMED sd':>10}{'VIBRATION':>11}"
          f"{'sim inj.':>10}{'sensor ACF1':>12}{'armed ACF1':>11}")
    for name, key, col in CH:
        s_u, ac_u = stats(un[key], col)
        s_a, ac_a = stats(ar[key], col)
        vib = float(np.sqrt(max(s_a ** 2 - s_u ** 2, 0.0)))
        print(f"{name:<9}{s_u:>11.5f}{s_a:>10.5f}{vib:>11.5f}"
              f"{SIM_A.get(name, float('nan')):>10.5f}{ac_u:>12.2f}{ac_a:>11.2f}")
    print("\nSENSOR = inject always. VIBRATION = inject only when thrusters active"
          "\n(scale with |cmd|). sim inj. = current zero-excitation injection for"
          "\ncomparison. ACF1 = increment lag-1 autocorr (noise-shape target).")


if __name__ == "__main__":
    main()
