#!/usr/bin/env python3
"""Compose the 'robot's inner monologue' video from a decision_run capture:

  [ chase camera ]  |  [ overhead: pool + TRUTH trail vs DR-belief trail ]
  [ live telemetry: waypoint, DR position, truth, heading error, commands ]

Inputs (all sim-time-stamped): frames/frame_*.png (camera), sim_gt.csv +
sim_dr.csv (sim_traj_recorder), decisions.csv (waypoint_controller --log).

Usage: compose_decision_video.py <run_dir> <out.mp4>
"""
import glob
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import image as mpimg

X0, L, W = 35.0, 4.0, 2.0     # intex pool in world coords
COURSE = [(1.0, -0.3), (3.0, -0.3), (3.0, 0.3), (1.0, 0.3), (1.0, -0.3)]


def main():
    run, out = sys.argv[1], sys.argv[2]
    frames = sorted(glob.glob(os.path.join(run, "frames", "frame_*.png")))
    ft = np.array([float(os.path.basename(f)[6:-4]) for f in frames])
    gt = np.genfromtxt(os.path.join(run, "sim_gt.csv"), delimiter=",", names=True)
    dr = np.genfromtxt(os.path.join(run, "sim_dr.csv"), delimiter=",", names=True)
    dec = np.genfromtxt(os.path.join(run, "decisions.csv"), delimiter=",", names=True)
    # DR odom is relative to its own start; anchor it at the truth start point
    dr_x = dr["x"] - dr["x"][0] + gt["x"][0]
    dr_y = dr["y"] - dr["y"][0] + gt["y"][0]
    t0 = gt["t"][0]

    tmp = os.path.join(run, "_comp")
    os.makedirs(tmp, exist_ok=True)
    wps = np.array(COURSE) + [X0, 0]

    for i, (f, tf) in enumerate(zip(frames, ft)):
        fig = plt.figure(figsize=(12.8, 5.4), facecolor="#101418")
        # --- camera panel
        axc = fig.add_axes([0.005, 0.02, 0.55, 0.96])
        axc.imshow(mpimg.imread(f))
        axc.axis("off")
        axc.set_title("chase camera", color="w", fontsize=10, pad=4)
        # --- overhead panel
        axo = fig.add_axes([0.585, 0.36, 0.40, 0.60], facecolor="#101418")
        axo.add_patch(plt.Rectangle((X0, -W/2), L, W, fill=False, lw=2, color="#4f8cc9"))
        axo.plot(wps[:, 0], wps[:, 1], ":", color="#777", lw=1.2)
        gsel = gt["t"] - t0 <= tf
        dsel = dr["t"] - t0 <= tf
        axo.plot(gt["x"][gsel], gt["y"][gsel], "-", color="#63d471", lw=1.6, label="TRUTH")
        axo.plot(dr_x[dsel], dr_y[dsel], "-", color="#e05c5c", lw=1.6, label="DR belief")
        if gsel.any():
            axo.plot(gt["x"][gsel][-1], gt["y"][gsel][-1], "o", color="#63d471", ms=8)
        if dsel.any():
            axo.plot(dr_x[dsel][-1], dr_y[dsel][-1], "o", color="#e05c5c", ms=8)
        axo.set_xlim(X0 - 0.25, X0 + L + 0.25)
        axo.set_ylim(-W/2 - 0.25, W/2 + 0.25)
        axo.set_aspect("equal")
        axo.tick_params(colors="#888", labelsize=7)
        for s in axo.spines.values():
            s.set_color("#444")
        axo.legend(loc="upper right", fontsize=7, facecolor="#181c22",
                   labelcolor="w", edgecolor="#444")
        axo.set_title("overhead — where it IS vs where it THINKS it is",
                      color="w", fontsize=9, pad=4)
        # --- telemetry panel (the controller's decisions, like live prints)
        axt = fig.add_axes([0.585, 0.02, 0.40, 0.30])
        axt.axis("off")
        k = np.searchsorted(dec["t"] - dec["t"][0], tf) - 1
        gap = ""
        if gsel.any() and dsel.any():
            g = np.hypot(gt["x"][gsel][-1] - dr_x[dsel][-1],
                         gt["y"][gsel][-1] - dr_y[dsel][-1])
            gap = f"{g:5.2f} m"
        if k >= 0:
            d = dec[k]
            txt = (f"t = {tf:6.1f} s      CONTROLLER (sees only DR)\n"
                   f"waypoint  #{int(d['wp'])}  ->  heading error {np.degrees(d['err']):+6.1f} deg\n"
                   f"decide:   forward {d['fwd']:.2f} m/s   yaw {d['yawcmd']:+.2f} rad/s\n"
                   f"DR pos    ({d['x']:+5.2f}, {d['y']:+5.2f})   truth-vs-belief gap {gap}")
        else:
            txt = f"t = {tf:6.1f} s   waiting for first fix..."
        axt.text(0.02, 0.9, txt, transform=axt.transAxes, color="#d8e6f2",
                 fontsize=11, family="monospace", va="top", linespacing=1.7)
        fig.savefig(os.path.join(tmp, f"c_{i:05d}.png"), dpi=100,
                    facecolor=fig.get_facecolor())
        plt.close(fig)
        if i % 100 == 0:
            print(f"  frame {i}/{len(frames)}", flush=True)

    subprocess.run(["ffmpeg", "-y", "-framerate", "5", "-i",
                    os.path.join(tmp, "c_%05d.png"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "24", out], check=True)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
