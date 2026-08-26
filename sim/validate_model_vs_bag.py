"""Validate (and tune) the standard-BlueROV2 sim model against the LAB'S OWN
vehicle: replay the real bag's per-thruster PWMs (/mavros/rc/out ch1-6) through
the Fossen model and compare predicted velocity changes against the measured
DVL velocities + gyro yaw rate.

Method: one-step-ahead prediction (standard sysID validation). At each DVL
sample, initialize the model with the MEASURED state, integrate forward H
seconds with the recorded thruster commands, and compare the predicted velocity
at t+H with the measurement at t+H. Reports per-axis RMSE/correlation, then
grid-searches THRUST_SCALE and DRAG_SCALE for the best fit to this vehicle.

ArduSub standard vectored frame (AP_Motors6DOF.cpp): positive motor output
directions M1..M6 in body FLU:
  M1 (+0.14,-0.092): (-.707,+.707,0)   M2 (+0.14,+0.092): (-.707,-.707,0)
  M3 (-0.14,-0.092): (+.707,+.707,0)   M4 (-0.14,+0.092): (+.707,-.707,0)
  M5 (0,-0.109,.077): (0,0,-1)         M6 (0,+0.109,.077): (0,0,-1)
PWM -> normalized: (pwm - 1500) / 400.

Usage: python3 validate_model_vs_bag.py <bag.mcap> [--tune]
"""
import argparse
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

import bluerov2_standard_model as bm

# ArduSub-convention thruster directions converted FRD->FLU (ArduSub factors are
# forward/lateral-RIGHT/z-down; model frame is FLU, so lateral flips sign)
ARDUSUB_DIR = np.array([
    [-0.7071, -0.7071, 0.0],
    [-0.7071, 0.7071, 0.0],
    [0.7071, -0.7071, 0.0],
    [0.7071, 0.7071, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 0.0, -1.0],
])
DVL_LEVER_ARM = np.array([-0.171, -0.089, -0.013])   # from the lab's dead_reckon params
HORIZON = 1.0          # s, one-step-ahead prediction window
SUB_DT = 0.02          # s, integration substep


def load_bag(path):
    topics = ["/mavros/rc/out", "/dvl/twist", "/mavros/imu/data"]
    data = {t: [] for t in topics}
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, m, msg in reader.iter_decoded_messages(topics=topics):
            data[ch.topic].append((m.log_time * 1e-9, msg))
    t0 = min(v[0][0] for v in data.values())

    pwm = np.array([[t - t0] + [float(m.channels[i]) for i in range(6)]
                    for t, m in data["/mavros/rc/out"]])
    dvl = np.array([[t - t0, m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]
                    for t, m in data["/dvl/twist"]])
    imu = np.array([[t - t0,
                     m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                     m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]
                    for t, m in data["/mavros/imu/data"]])
    return pwm, dvl, imu


def interp_rows(table, t):
    """Zero-order hold on the most recent row at time t (commands are steps)."""
    idx = np.searchsorted(table[:, 0], t, side="right") - 1
    return table[max(idx, 0), 1:]


def predict(model, t_start, nu0, quat, pwm_table):
    nu = nu0.copy()
    steps = int(HORIZON / SUB_DT)
    for k in range(steps):
        tk = t_start + k * SUB_DT
        cmd = np.clip((interp_rows(pwm_table, tk) - 1500.0) / 400.0, -1, 1)
        nu_dot = model.step_ardusub(cmd, quat, nu, z_world=-0.4)
        nu = nu + nu_dot * SUB_DT
    return nu


def run(bag_path, thrust_scale, drag_scale, pwm, dvl, imu):
    bm.THRUST_SCALE = thrust_scale
    bm.DRAG_SCALE = drag_scale
    model = bm.BlueROV2StandardModel()

    preds, meas = [], []
    for row in dvl:
        t = row[0]
        t_end = t + HORIZON
        if t_end > dvl[-1, 0]:
            break
        gyro = interp_rows(imu[:, :4], t)
        quat = interp_rows(imu[:, [0, 4, 5, 6, 7]], t)
        # DVL measures at its lever arm; convert to COM velocity
        v_com0 = row[1:4] - np.cross(gyro, DVL_LEVER_ARM)
        nu0 = np.concatenate([v_com0, gyro])

        nu_pred = predict(model, t, nu0, quat, pwm)

        # measured state at t+H
        gyro1 = interp_rows(imu[:, :4], t_end)
        i1 = np.searchsorted(dvl[:, 0], t_end)
        if i1 >= len(dvl):
            break
        v_com1 = dvl[i1, 1:4] - np.cross(gyro1, DVL_LEVER_ARM)

        preds.append(np.concatenate([nu_pred[:3], [nu_pred[5]]]))
        meas.append(np.concatenate([v_com1, [gyro1[2]]]))

    preds, meas = np.array(preds), np.array(meas)
    rmse = np.sqrt(((preds - meas) ** 2).mean(axis=0))
    corr = [np.corrcoef(preds[:, i], meas[:, i])[0, 1] for i in range(4)]
    return rmse, corr, len(preds)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--tune", action="store_true")
    args = ap.parse_args()

    # ArduSub-convention mixing matrix for the model
    T = np.zeros((6, 6))
    for i in range(6):
        T[:3, i] = ARDUSUB_DIR[i]
        T[3:, i] = np.cross(bm.THRUSTER_POS[i], ARDUSUB_DIR[i])

    def step_ardusub(self, cmd, quat, nu, z_world=-0.4):
        forces = bm.THRUST_SCALE * bm.t200_force(cmd)
        saved = self.T
        self.T = T
        try:
            # reuse the model's own physics with the ArduSub mixing
            tau = self.T @ forces
            import numpy as _np
            R = bm._quat_to_rot(quat)
            submerged = _np.clip((0.0 - z_world + 0.125) / 0.25, 0.0, 1.0)
            g_w = _np.array([0.0, 0.0, -bm.WEIGHT])
            b_w = _np.array([0.0, 0.0, bm.BUOYANCY * submerged])
            tau = tau + _np.concatenate([
                R.T @ (g_w + b_w),
                _np.cross(bm.R_G, R.T @ g_w) + _np.cross(bm.R_B, R.T @ b_w)])
            tau = tau - bm._coriolis(nu) @ nu
            tau = tau - bm.DRAG_SCALE * (bm.D_LIN + bm.D_QUAD * _np.abs(nu)) * nu
            return self.M_inv @ tau
        finally:
            self.T = saved

    bm.BlueROV2StandardModel.step_ardusub = step_ardusub

    pwm, dvl, imu = load_bag(args.bag)
    print(f"loaded: {len(pwm)} rc/out, {len(dvl)} dvl, {len(imu)} imu samples")

    labels = ["u (m/s)", "v (m/s)", "w (m/s)", "r (rad/s)"]
    rmse, corr, n = run(args.bag, bm.THRUST_SCALE, bm.DRAG_SCALE, pwm, dvl, imu)
    print(f"\n=== current constants (THRUST_SCALE={bm.THRUST_SCALE}, DRAG_SCALE={bm.DRAG_SCALE}) "
          f"— {n} predictions, {HORIZON}s horizon ===")
    for i, lab in enumerate(labels):
        print(f"  {lab:10s} rmse={rmse[i]:.4f}  corr={corr[i]:+.3f}")

    if args.tune:
        best = (1e9, None)
        for ts in np.arange(0.30, 1.01, 0.05):
            for ds in np.arange(0.6, 2.61, 0.2):
                r, c, _ = run(args.bag, ts, ds, pwm, dvl, imu)
                score = r[0] / 0.1 + r[1] / 0.1 + r[3] / 0.2   # weighted u, v, r
                if score < best[0]:
                    best = (score, (ts, ds, r, c))
        ts, ds, r, c = best[1]
        print(f"\n=== best fit: THRUST_SCALE={ts:.2f}, DRAG_SCALE={ds:.2f} ===")
        for i, lab in enumerate(labels):
            print(f"  {lab:10s} rmse={r[i]:.4f}  corr={c[i]:+.3f}")


if __name__ == "__main__":
    main()
