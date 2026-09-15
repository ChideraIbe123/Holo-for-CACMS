"""Recover the vehicle's EFFECTIVE per-axis sign directly from data, using the
pilot's teleop commands vs the measured motion — no motor-mixer assumptions.

/teleop/cmd (TwistStamped): linear.x=forward, linear.y=lateral, angular.z=yaw cmd.
Correlate each against the measured response (DVL vx=surge, vy=sway, gyro z=yaw
rate). Sign of the correlation = the effective axis sign the sim must reproduce.

Usage: python3 infer_signs_teleop.py <mcap> [<mcap> ...]
"""
import sys
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def load(mc):
    tele, dvl, gz = [], [], []
    with open(mc, "rb") as f:
        r = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, m, msg in r.iter_decoded_messages(
                topics=["/teleop/cmd", "/dvl/twist", "/mavros/imu/data"]):
            t = m.log_time * 1e-9
            if ch.topic == "/teleop/cmd":
                tele.append([t, msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z])
            elif ch.topic == "/dvl/twist":
                dvl.append([t, msg.twist.linear.x, msg.twist.linear.y])
            else:
                gz.append([t, msg.angular_velocity.z])
    return np.array(tele), np.array(dvl), np.array(gz)


def corr_at(cmd_t, cmd_v, meas_t, meas_v):
    """ZOH command at each measurement time (with a lag: velocity responds AFTER
    the command). Sweep small lags, return the best |corr| with its sign, plus the
    sign-agreement fraction on strong-command samples."""
    if len(cmd_t) < 5 or len(meas_t) < 5 or np.ptp(cmd_v) < 1e-6:
        return None, 0.0
    best = 0.0
    for lag in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        idx = np.clip(np.searchsorted(cmd_t, meas_t - lag, side="right") - 1, 0, len(cmd_v) - 1)
        c = cmd_v[idx]
        if np.std(c) < 1e-9 or np.std(meas_v) < 1e-9:
            continue
        r = float(np.corrcoef(c, meas_v)[0, 1])
        if abs(r) > abs(best):
            best = r; best_c = c
    if best == 0.0:
        return None, 0.0
    # sign agreement on strong-command samples (|cmd| in top 40%)
    thr = np.quantile(np.abs(best_c), 0.6)
    strong = np.abs(best_c) >= max(thr, 1e-6)
    agree = float(np.mean(np.sign(best_c[strong]) == np.sign(meas_v[strong]))) if strong.any() else 0.0
    return best, agree


def main():
    axes = {"surge (fwd cmd -> DVL vx)": [], "sway (lat cmd -> DVL vy)": [], "yaw (yaw cmd -> gyro z)": []}
    for mc in sys.argv[1:]:
        try:
            tele, dvl, gz = load(mc)
        except Exception as e:
            print(f"skip {mc.split('/')[-1]}: {type(e).__name__}")
            continue
        if len(tele) < 5:
            continue
        r_su, a_su = corr_at(tele[:, 0], tele[:, 1], dvl[:, 0], dvl[:, 1]) if len(dvl) else (None, 0)
        r_sw, a_sw = corr_at(tele[:, 0], tele[:, 2], dvl[:, 0], dvl[:, 2]) if len(dvl) else (None, 0)
        r_yaw, a_yaw = corr_at(tele[:, 0], tele[:, 3], gz[:, 0], gz[:, 1]) if len(gz) else (None, 0)
        name = mc.split("/")[-1][:26]
        def fmt(r,a): return "  none" if r is None else f"{r:+.2f}(ag{a:.0%})"
        print(f"{name:28} surge {fmt(r_su,a_su)}  sway {fmt(r_sw,a_sw)}  yaw {fmt(r_yaw,a_yaw)}")
        for k, r in zip(axes, (r_su, r_sw, r_yaw)):
            if r is not None:
                axes[k].append(r)

    print("\n=== effective axis sign (aggregate across bags) ===")
    for k, rs in axes.items():
        if not rs:
            print(f"{k:32} (no data)"); continue
        m = float(np.mean(rs))
        verdict = "POSITIVE (sim assumption OK)" if m > 0.2 else \
                  ("NEGATIVE (flip this axis)" if m < -0.2 else "weak/ambiguous")
        print(f"{k:32} mean {m:+.2f}  ({sum(r>0 for r in rs)}/{len(rs)} +)  -> {verdict}")


if __name__ == "__main__":
    main()
