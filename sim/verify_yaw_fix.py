"""Verify the yaw-sign fix: replay each real bag's per-thruster PWM through the
model (ardusub mixer) and correlate the model's predicted yaw torque with the
REAL measured yaw rate. Before the fix this was negative (sim turned the wrong
way); after EFFECTIVE_SIGN it should be positive (sim turns like the real vehicle).

Usage: python3 verify_yaw_fix.py <mcap> [...]
"""
import sys
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import bluerov2_standard_model as m

mdl = m.BlueROV2StandardModel()


def tau_yaw(pwm_row, sign):
    cmd = np.clip((np.array(pwm_row) - 1500.0) / 400.0, -1, 1)
    forces = m.THRUST_SCALE * m.t200_force(cmd)
    tau = mdl.T_ardusub @ forces
    return (tau * sign)[5]   # yaw torque


def run(mc, sign):
    pwm, gz = [], []
    with open(mc, "rb") as f:
        r = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, msg, d in r.iter_decoded_messages(topics=["/mavros/rc/out", "/mavros/imu/data"]):
            t = msg.log_time * 1e-9
            if ch.topic == "/mavros/rc/out":
                pwm.append([t] + [float(d.channels[i]) for i in range(6)])
            else:
                gz.append([t, d.angular_velocity.z])
    pwm, gz = np.array(pwm), np.array(gz)
    if len(pwm) < 5 or len(gz) < 5:
        return None
    ty = np.array([tau_yaw(pwm[np.clip(np.searchsorted(pwm[:, 0], t - 0.4) - 1, 0, len(pwm) - 1), 1:7], sign)
                   for t in gz[:, 0]])
    if np.std(ty) < 1e-9:
        return None
    return float(np.corrcoef(ty, gz[:, 1])[0, 1])


def main():
    print(f"{'bag':30}{'yaw corr NO fix':>16}{'yaw corr WITH fix':>18}")
    no, yes = [], []
    for mc in sys.argv[1:]:
        try:
            rn = run(mc, np.ones(6))
            ry = run(mc, m.EFFECTIVE_SIGN)
        except Exception as e:
            print(f"{mc.split('/')[-1][:28]:30} skip ({type(e).__name__})"); continue
        if rn is None:
            continue
        no.append(rn); yes.append(ry)
        print(f"{mc.split('/')[-1][:28]:30}{rn:>16.2f}{ry:>18.2f}")
    if no:
        print(f"\n{'MEAN':30}{np.mean(no):>16.2f}{np.mean(yes):>18.2f}")
        print(f"\n-> predicted yaw {'now MATCHES' if np.mean(yes) > 0.2 else 'does not match'} "
              f"the real vehicle's turn direction")


if __name__ == "__main__":
    main()
