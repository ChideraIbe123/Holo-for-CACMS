#!/usr/bin/env python3
"""HoloOcean -> lab-format ROS 2 bridge.

Runs HoloOcean and rclpy in ONE process (use a python matching the ROS distro,
e.g. system python3.10 with ROS 2 Humble sourced) and publishes the lab's exact
topic formats so the dead-reckoning stack consumes sim data unmodified:

  /mavros/imu/data                sensor_msgs/Imu        (imu_depth_reset.yaml)
  /dvl/twist                      geometry_msgs/TwistStamped  (dvl_twist.yaml)
  /mavros/global_position/rel_alt std_msgs/Float64

Also publishes sim ground truth on /holoocean/ground_truth (nav_msgs/Odometry)
for dead-reckoning drift evaluation. Message-building code is ported unchanged
from the mock publishers that were format-verified against the lab YAMLs and
closed-loop tested with dvl_dead_reckon.

Usage:
    source /opt/ros/humble/setup.bash
    python3 mavros_bridge.py [--move] [--headless] [--duration SEC]
"""
import argparse
import math
import os

import numpy as np
import holoocean
import rclpy

from bluerov2_standard_model import BlueROV2StandardModel
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64, Float32, Bool
from nav_msgs.msg import Odometry

# =====================================================================================
# CONFIG — tune these against the real data (see the redacted YAMLs / a real bag)
# =====================================================================================

SCENARIO_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scenario_bluerov.json')
AGENT_NAME = 'auv0'

# --- IMU: /mavros/imu/data (sensor_msgs/msg/Imu) ---
# Values below extracted from real vehicle bag rosbag_20260504_154118 (2026-05-04 pool test)
IMU_TOPIC = '/mavros/imu/data'
IMU_FRAME_ID = 'base_link'          # real vehicle publishes IMU in base_link
# Gravity convention (verified empirically on HoloOcean 2.3.0, Ocean package):
# HoloOcean's IMU reads -9.8 on the body up-axis at rest — the gravity term has
# the OPPOSITE sign of a real IMU (+9.8 up at rest), while the kinematic part has
# the true sign (checked: forward thrust -> positive x accel).
#   'flip_gravity' -> real = holoocean + R_body_from_world @ [0, 0, 2g]  (default)
#   'none'         -> passthrough (if a future HoloOcean version fixes the sign)
#   'add'          -> + rotated +g (for a gravity-free source)
GRAVITY_MODE = 'flip_gravity'
GRAVITY = 9.81
# Real-vehicle covariances (constant across the whole bag):
ORIENTATION_COV_DIAG = [1.0, 1.0, 1.0]
ANGULAR_VELOCITY_COV_DIAG = [1.2184700254281e-07] * 3
LINEAR_ACCELERATION_COV_DIAG = [8.999999999999999e-08] * 3

# --- DVL: /dvl/twist (geometry_msgs/msg/TwistStamped) ---
DVL_TOPIC = '/dvl/twist'
DVL_FRAME_ID = 'dvl_link'           # confirmed against the real bag
# Angular fields are all-zero in the real bag (confirmed) -> left zero.

# --- DVL quality topics (present in the real bag at ~14 Hz; published here with
#     each DVL message, which is what the dead-reckon node's gating consumes) ---
VELOCITY_VALID_TOPIC = '/dvl/velocity_valid'
FOM_TOPIC = '/dvl/fom'
DVL_FOM_VALUE = 0.002               # median FOM in the real bag (mean 0.0021, max 0.0057)

# --- Depth: /mavros/global_position/rel_alt (std_msgs/msg/Float64) ---
DEPTH_TOPIC = '/mavros/global_position/rel_alt'
# HoloOcean DepthSensor returns global z (z-up, negative underwater), which already
# behaves like a relative altitude. rel_alt = REL_ALT_SIGN * (z - WATER_SURFACE_Z).
REL_ALT_SIGN = 1.0
WATER_SURFACE_Z = 0.0

# --- Ground truth (sim-only, for drift evaluation) ---
GT_TOPIC = '/holoocean/ground_truth'
GT_FRAME_ID = 'map'
GT_CHILD_FRAME_ID = 'base_link'

# --- NOISE (all values MEASURED from the real bag rosbag_20260504_154118 via the
#     successive-difference estimator; the bag has no stationary segment, so these
#     floors include real thruster vibration — the lab's actual disturbance level).
#     HoloOcean-native noise (IMU accel/gyro, DVL beams, depth) lives in
#     scenario_bluerov.json; the values below cover what HoloOcean can't do.
#     --no-noise disables BOTH (bridge zeroes the scenario sigmas on load). ---
NOISE_ENABLED = True
# Excitation-dependent noise: sigma(E) = a + b*E, E = mean |normalized thruster cmd|.
# Fitted from 48 five-second windows across 5 real bags (fit_noise_model.py).
# Applied in the bridge (engine sigmas are zeroed) so sigma can follow effort.
# v12: gyro/dvl sigmas REVERTED to v10 — the v11 raise fought the ACF metrics
# (noisier increments = further from real's +0.7 lag-1). Missing variance is the
# low-freq rocking band, retained by the shorter ref LP instead. Accel kept at v11.
# (superseded note: v11 rescale below)
# v11 rescale: with rotational trajectory matching supplying real motion, sigmas
# were re-fit so sim per-axis increment totals match the 13-bag real means
# (measured on v10 campaign runs; e.g. dvl was -53% low, gyros -31..-38% low).
# accel_x deliberately NOT raised: its v10 excess (+109%) was servo-chatter
# gravity leak, fixed by the ref low-pass instead.
NOISE_MODEL = {
    'gyro_x': (0.00635, 0.13451),   # x1.45
    'gyro_y': (0.01378, 0.02021),   # x1.49
    'gyro_z': (0.04141, 0.48643),   # x1.50
    'accel_x': (0.15382, 0.21025),  # x1.00 (see note above)
    'accel_y': (0.11117, 0.16047),  # x1.12
    'accel_z': (0.05446, 0.03398),  # x1.34
    'dvl_x': (0.01983, 0.04191),    # x1.50
    'dvl_y': (0.01757, 0.08603),    # x1.50
    'dvl_z': (0.01061, 0.00000),    # x1.43
}


def noise_sigma(ch, effort):
    a, b = NOISE_MODEL[ch]
    return a + b * effort
ORIENTATION_NOISE_RPY_DEG = [0.32, 0.34, 1.39]   # DynamicsSensor is noiseless in-engine
REL_ALT_QUANTIZATION = 0.001                     # m; real topic has 1 mm granularity
ACCEL_LSB = 0.00980665                           # real accel is quantized at 1 milli-g
# The real 10 Hz accel topic is a SAMPLE-AND-HOLD of a ~2 Hz internal update
# (measured: 81% of consecutive samples identical, variance arrives in bursts).
# Scenario AccelSigma is scaled up by 1/sqrt(P) so the successive-diff floor
# still matches the real multi-bag floors.
ACCEL_UPDATE_P = 0.19
# HYBRID: constant per-axis accel sigmas (multi-bag means, the v3 config that passed
# all accel metrics). Excitation model kept for gyros only (helped there).
ACCEL_SIGMA_CONST = [0.19, 0.15, 0.05]
DVL_FOM_JITTER = True                            # fom ~ max(0, N(mean, std)) as in real bag
DVL_FOM_STD = 0.00185

# --- Timing ---
# 'sim'  -> stamp from HoloOcean sim time state['t'] (exact physics dt; what BYU's
#           own bridge does; correct even if the sim runs off real-time)
# 'wall' -> stamp from the node clock (matches the real vehicle; requires the sim
#           to run at real-time; needed if st_car_ekf mode is ever used, since it
#           mixes message stamps with wall-clock now())
# --- Closed-loop control (velocity autopilot) ---
# When --control is set, the bridge subscribes to a TwistStamped velocity setpoint
# (surge=linear.x, sway=linear.y, heave=linear.z, yaw_rate=angular.z) and runs an
# inner P velocity loop -> body-force setpoint, emulating ArduSub's velocity mode.
# This lets an external controller drive the sim vehicle closed-loop.
CONTROL_TOPIC = '/cmd_vel'
CTRL_KP_LIN = 60.0        # N per (m/s) surge/sway error
CTRL_KP_Z = 80.0          # N per (m/s) heave error
CTRL_KP_YAW = 20.0        # N*m per (rad/s) yaw-rate error
# Real vehicle's yaw command->motion is INVERTED (infer_signs_teleop.py on obstacle
# bags: teleop yaw vs gyro_z = -0.73, all runs). A controller written for the real
# vehicle expects that convention, so the sim autopilot matches it. Set +1 if the
# benchmark's MAVROS controllers turn out to use the standard convention (needs a
# velocity-setpoint bag or the QGC param to confirm which the setpoint path uses).
YAW_CMD_SIGN = -1.0
# Replay yaw-rate servo: the geometric mixer is degenerate for yaw (PWM replay
# gives ~0 yaw corr), so during --replay the model's yaw rate is servoed to the
# bag's MEASURED gyro_z (stored in the profile npz as t_gz/gz). Surge/heave stay
# PWM-driven (they validated against the real vehicle). Gain in N*m/(rad/s);
# 0 disables. Sign maps real gyro_z convention onto model body yaw rate —
# verify with a single replay run (sim gyro_z should correlate ~+1 with ref).
YAWREF_GAIN = 3.0
# -1 verified empirically: with +1 the published sim gyro_z came out ANTI-correlated
# with the bag reference (corr -0.768 on yaw-rich bag 151023) — same inverted yaw
# convention seen everywhere else on this vehicle; with -1 the published signal
# matches the real trace. (v9 sign-check gate caught this before the campaign.)
YAWREF_SIGN = -1.0
# v10: same servo on roll/pitch rates. Real gyro_x/y are MOTION-dominated (increment
# lag-1 autocorr +0.7; sim's was -0.25 = pure AR1 noise signature) — the model barely
# rocks during replay, so pitch/roll motion must be trajectory-matched like yaw.
# Signs empirically gated per axis before each campaign (run_v10 sign check).
ATTREF_GAIN = 1.5    # N*m/(rad/s); restoring moments are stiffer in roll/pitch
# Sign-gate findings (v10): x +0.63, y -0.86, z +0.77 -> x keeps +1, y flips.
# The pattern (x same, y and z inverted) = FLU-vs-FRD frame mismatch: the real
# vehicle's angular rates are FRD relative to the sim's FLU body frame. This
# EXPLAINS the yaw inversion seen all along, rather than just patching it.
ATTREF_SIGN_X = 1.0
ATTREF_SIGN_Y = -1.0
CTRL_TAU_MAX = np.array([90.0, 90.0, 120.0, 0.0, 0.0, 22.0])  # BlueROV2 axis force/torque limits
CTRL_TIMEOUT = 1.0        # s; zero the command if no setpoint arrives

STAMP_SOURCE = 'sim'
REAL_TIME_PACE = True               # sleep each tick so sim time tracks wall time

# Frame note: HoloOcean world is right-handed x-forward/y-left/z-up and the body
# frame is FLU — the same structure ROS uses. The dead-reckon stack only needs
# internal consistency (verified in closed-loop test); no axis remapping is done.

# =====================================================================================

# DynamicsSensor (UseRPY=false) 19-vector layout, global frame at COM:
#   [0:3] accel  [3:6] vel  [6:9] pos  [9:12] ang_accel  [12:15] ang_vel  [15:19] quat x,y,z,w
DYN_POS = slice(6, 9)
DYN_QUAT = slice(15, 19)


def _diag_to_cov9(diag):
    cov = [0.0] * 9
    cov[0], cov[4], cov[8] = float(diag[0]), float(diag[1]), float(diag[2])
    return cov


def quat_to_rot_matrix_xyzw(q):
    """Unit quaternion (x, y, z, w) -> body-to-world rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_quat_xyzw(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quat_multiply_xyzw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


# Colored (AR1) noise state per channel — real sensor noise rolls off at high
# frequency; white noise puts too much power in the top of the band and fails the
# spectral metric. AR1_A sets the corner (0=white, ->1 = heavier low-pass).
AR1_A = 0.5
_ar1_state = {}


def colored(key, sigma):
    """AR(1) low-pass noise with stationary std = sigma."""
    prev = _ar1_state.get(key, 0.0)
    innov = np.random.randn() * sigma * np.sqrt(1.0 - AR1_A * AR1_A)
    val = AR1_A * prev + innov
    _ar1_state[key] = val
    return val


def perturb_quat_xyzw(q):
    """Compose q with a small Gaussian RPY rotation (attitude-estimate noise)."""
    r, p, y = (np.random.randn(3) * np.radians(ORIENTATION_NOISE_RPY_DEG))
    return quat_multiply_xyzw(q, rpy_to_quat_xyzw(r, p, y))


class MavrosBridge:
    def __init__(self, node):
        self.node = node
        self.imu_pub = node.create_publisher(Imu, IMU_TOPIC, 10)
        self.dvl_pub = node.create_publisher(TwistStamped, DVL_TOPIC, 10)
        self.depth_pub = node.create_publisher(Float64, DEPTH_TOPIC, 10)
        self.vel_valid_pub = node.create_publisher(Bool, VELOCITY_VALID_TOPIC, 10)
        self.fom_pub = node.create_publisher(Float32, FOM_TOPIC, 10)
        self.gt_pub = node.create_publisher(Odometry, GT_TOPIC, 10)
        self.cmd_vel = np.zeros(4)   # [surge, sway, heave, yaw_rate] desired
        self.cmd_time = -1e9
        node.create_subscription(TwistStamped, CONTROL_TOPIC, self._on_cmd_vel, 10)
        self.last_quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0])
        self.sim_time = 0.0
        self.effort = 0.0      # mean |thruster cmd|, set by the main loop each tick
        self._gt_decim = 1     # publish GT every Nth dynamics sample
        self._gt_count = 0

    def _on_cmd_vel(self, msg):
        self.cmd_vel = np.array([msg.twist.linear.x, msg.twist.linear.y,
                                 msg.twist.linear.z, msg.twist.angular.z])
        self.cmd_time = self.sim_time

    def autopilot_tau(self, nu):
        """Inner P velocity loop: desired body velocity -> body-force setpoint."""
        des = self.cmd_vel if (self.sim_time - self.cmd_time) < CTRL_TIMEOUT else np.zeros(4)
        tau = np.zeros(6)
        tau[0] = CTRL_KP_LIN * (des[0] - nu[0])
        tau[1] = CTRL_KP_LIN * (des[1] - nu[1])
        tau[2] = CTRL_KP_Z * (des[2] - nu[2])
        tau[5] = YAW_CMD_SIGN * CTRL_KP_YAW * (des[3] - YAW_CMD_SIGN * nu[5])
        return np.clip(tau, -CTRL_TAU_MAX, CTRL_TAU_MAX)

    def _stamp(self):
        if STAMP_SOURCE == 'sim':
            from builtin_interfaces.msg import Time as TimeMsg
            msg = TimeMsg()
            msg.sec = int(self.sim_time)
            msg.nanosec = int((self.sim_time - msg.sec) * 1e9)
            return msg
        return self.node.get_clock().now().to_msg()

    def on_dynamics(self, dyn):
        self.last_quat_xyzw = np.asarray(dyn[DYN_QUAT], dtype=float)

        self._gt_count += 1
        if self._gt_count % self._gt_decim:
            return

        odom = Odometry()
        odom.header.stamp = self._stamp()
        odom.header.frame_id = GT_FRAME_ID
        odom.child_frame_id = GT_CHILD_FRAME_ID
        pos = dyn[DYN_POS]
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        q = self.last_quat_xyzw
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        self.gt_pub.publish(odom)

    def on_imu(self, imu_data):
        accel = np.asarray(imu_data[0], dtype=float)   # body frame, m/s^2
        gyro = np.asarray(imu_data[1], dtype=float)    # body frame, rad/s
        if NOISE_ENABLED:
            E = self.effort
            gyro = gyro + np.array([
                colored('gyro_x', noise_sigma('gyro_x', E)),
                colored('gyro_y', noise_sigma('gyro_y', E)),
                colored('gyro_z', noise_sigma('gyro_z', E))]) / np.sqrt(2)

        if GRAVITY_MODE == 'flip_gravity':
            # convert HoloOcean's flipped gravity term to real-IMU convention
            rot_m = quat_to_rot_matrix_xyzw(self.last_quat_xyzw)   # body -> world
            accel = accel + rot_m.T @ np.array([0.0, 0.0, 2.0 * GRAVITY])
        elif GRAVITY_MODE == 'add':
            # a real accelerometer at rest reads +g on the body up-axis
            rot_m = quat_to_rot_matrix_xyzw(self.last_quat_xyzw)   # body -> world
            accel = accel + rot_m.T @ np.array([0.0, 0.0, GRAVITY])

        msg = Imu()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = IMU_FRAME_ID
        # attitude-estimate noise on the published orientation only; the gravity
        # correction above uses the true attitude, and ground truth stays clean
        q = perturb_quat_xyzw(self.last_quat_xyzw) if NOISE_ENABLED else self.last_quat_xyzw
        msg.orientation.x = float(q[0])
        msg.orientation.y = float(q[1])
        msg.orientation.z = float(q[2])
        msg.orientation.w = float(q[3])
        msg.orientation_covariance = _diag_to_cov9(ORIENTATION_COV_DIAG)
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.angular_velocity_covariance = _diag_to_cov9(ANGULAR_VELOCITY_COV_DIAG)
        if NOISE_ENABLED:
            # HYBRID: accels use CONSTANT multi-bag sigmas (v3 config beat the fitted
            # excitation model on every accel metric) plus the sensor's real behavior
            # (2 Hz sample-and-hold, milli-g quantization).
            if not hasattr(self, "_held_accel") or np.random.rand() < ACCEL_UPDATE_P:
                sig = np.array(ACCEL_SIGMA_CONST) / np.sqrt(2 * ACCEL_UPDATE_P)
                self._held_accel = np.round((accel + np.random.randn(3) * sig) / ACCEL_LSB) * ACCEL_LSB
            accel = self._held_accel
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        msg.linear_acceleration_covariance = _diag_to_cov9(LINEAR_ACCELERATION_COV_DIAG)
        self.imu_pub.publish(msg)

    def on_dvl(self, dvl_data):
        v = np.asarray(dvl_data, dtype=float)[:3]      # body-frame velocity, m/s
        if np.isnan(v).any():
            return
        # v7: colored per-axis DVL noise in the bridge (real DVL noise rolls off at
        # high freq like the gyros; engine white noise failed the spectral metric).
        if NOISE_ENABLED:
            E = self.effort
            v = v + np.array([
                colored('dvl_x', noise_sigma('dvl_x', E)),
                colored('dvl_y', noise_sigma('dvl_y', E)),
                colored('dvl_z', noise_sigma('dvl_z', E))]) / np.sqrt(2)
        msg = TwistStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = DVL_FRAME_ID
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.linear.z = float(v[2])
        # angular left at zero — not populated by the real DVL driver
        self.dvl_pub.publish(msg)

        valid = Bool()
        valid.data = True               # HoloOcean DVL has no dropout model
        self.vel_valid_pub.publish(valid)
        fom = Float32()
        if NOISE_ENABLED and DVL_FOM_JITTER:
            fom.data = float(max(0.0, np.random.normal(DVL_FOM_VALUE, DVL_FOM_STD)))
        else:
            fom.data = float(DVL_FOM_VALUE)
        self.fom_pub.publish(fom)

    def on_depth(self, depth_data):
        z = float(np.asarray(depth_data, dtype=float).flatten()[0])
        rel_alt = REL_ALT_SIGN * (z - WATER_SURFACE_Z)
        if NOISE_ENABLED and REL_ALT_QUANTIZATION > 0:
            rel_alt = round(rel_alt / REL_ALT_QUANTIZATION) * REL_ALT_QUANTIZATION
        msg = Float64()
        msg.data = rel_alt
        self.depth_pub.publish(msg)


def scripted_command(t):
    """Scripted test motion, builtin (Heavy) dynamics: 8 thruster forces."""
    cmd = np.zeros(8)
    if t < 3.0:
        cmd[0:4] = -2.0        # vertical thrusters: descend
    else:
        cmd[4:8] = 6.0         # forward thrusters: cruise ~1 m/s
        cmd[[4, 6]] = 5.2      # slight asymmetry: slow yaw
    return cmd


SCRIPT_CRUISE = 0.50   # overridable via --cruise
SCRIPT_TURN = 0.44     # overridable via --turn


def scripted_command6(t, z=None, w_vert=0.0, z_target=-2.5):
    """Scripted test motion, standard-BlueROV2 dynamics: 6 normalized T200
    commands [T1..T4 vectored, T5..T6 vertical], each in [-1, 1].

    Verticals run a small depth-hold loop (like ArduSub's depth-hold mode,
    which the real vehicle uses) instead of open-loop trim."""
    cmd = np.zeros(6)
    if t >= 3.0:
        cmd[0:4] = SCRIPT_CRUISE   # vectored cruise
        cmd[[0, 2]] = SCRIPT_TURN  # asymmetry: yaw
    if z is None:
        cmd[4:6] = -0.45 if t < 3.0 else -0.105
    else:
        # P-D depth hold on the verticals
        cmd[4:6] = float(np.clip(0.8 * (z_target - z) - 0.8 * w_vert, -0.6, 0.6))
    return cmd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--move', action='store_true', help='drive a scripted path instead of sitting still')
    parser.add_argument('--headless', action='store_true', help='run HoloOcean without a viewport window')
    parser.add_argument('--duration', type=float, default=0.0, help='stop after N sim-seconds (0 = run forever)')
    parser.add_argument('--no-noise', action='store_true',
                        help='clean data: disables bridge-side noise AND zeroes scenario sensor sigmas')
    parser.add_argument('--control', action='store_true',
                        help='closed-loop: drive the vehicle from /cmd_vel velocity setpoints '
                             '(external controller closes the loop via /deadreckon/odom)')
    parser.add_argument('--replay', type=str, default=None,
                        help='npz command profile from a real bag (extract_cmd_profile.py); '
                             'replays real thruster commands instead of the scripted path')
    parser.add_argument('--cruise', type=float, default=0.50, help='scripted cruise command')
    parser.add_argument('--turn', type=float, default=0.44, help='scripted asymmetry command')
    parser.add_argument('--dynamics', choices=['standard', 'builtin'], default='standard',
                        help="'standard' = exact 6-thruster BlueROV2 Fossen model (default); "
                             "'builtin' = HoloOcean's 8-thruster Heavy dynamics")
    args = parser.parse_args()

    global NOISE_ENABLED, SCRIPT_CRUISE, SCRIPT_TURN
    if args.no_noise:
        NOISE_ENABLED = False
    SCRIPT_CRUISE, SCRIPT_TURN = args.cruise, args.turn

    rclpy.init()
    node = rclpy.create_node('holoocean_mavros_bridge')
    bridge = MavrosBridge(node)

    import json
    with open(SCENARIO_JSON) as f:
        scenario = json.load(f)
    ticks_per_sec = float(scenario['ticks_per_sec'])

    if not NOISE_ENABLED:
        # zero every sigma so HoloOcean's in-engine noise is off too
        for agent in scenario.get('agents', []):
            for sensor in agent.get('sensors', []):
                cfg = sensor.get('configuration', {})
                for key in list(cfg):
                    if key.endswith('Sigma'):
                        cfg[key] = 0.0
        print('[bridge] noise DISABLED (bridge-side + scenario sigmas zeroed)')

    model = None
    if args.dynamics == 'standard':
        # exact standard-BlueROV2 dynamics: drive the agent with accelerations
        # (control scheme 1) computed by our Fossen model; DynamicsSensor must
        # capture every tick to feed the model
        model = BlueROV2StandardModel()
        for agent in scenario['agents']:
            if agent['agent_name'] == AGENT_NAME:
                # engine scheme order (BlueROV2Controller.h): 0=thrusters, 1=PD,
                # 2=raw dynamics/accelerations (which also disables engine damping)
                agent['control_scheme'] = 2
                for sensor in agent['sensors']:
                    if sensor['sensor_type'] == 'DynamicsSensor':
                        sensor['Hz'] = scenario['ticks_per_sec']
        print('[bridge] dynamics: standard 6-thruster BlueROV2 (Fossen model)')

    node.get_logger().info(f'Starting HoloOcean ({SCENARIO_JSON}), headless={args.headless}')
    import time as _time
    replay = None
    if args.replay:
        rp = np.load(args.replay)
        replay = {'t': rp['t'], 'cmd': rp['cmd'], 'z0': float(rp['z0']), 'smooth': np.zeros(6),
                  'gz': (rp['t_gz'], rp['gz']) if 't_gz' in rp else None,
                  'gxy': (rp['t_gz'], rp['gx'], rp['gy']) if 'gx' in rp else None}
        for agent in scenario['agents']:
            if agent['agent_name'] == AGENT_NAME:
                # HYBRID: clamp spawn depth so the vehicle stays submerged (surface
                # bobbing wrecked the depth spectrum when profiles spawned at z0~-0.13)
                agent['location'][2] = min(replay['z0'], -0.5)
        print(f"[bridge] replaying real command profile: {args.replay} "
              f"({len(replay['t'])} cmds, {replay['t'][-1]:.0f}s, spawn z {replay['z0']:.2f})")

    if model is not None:
        # DynamicsSensor captures every tick in standard mode; keep GT topic at 50 Hz
        bridge._gt_decim = max(1, int(ticks_per_sec / 50))

    with holoocean.make(scenario_cfg=scenario, show_viewport=not args.headless) as env:
        t = 0.0
        wall_start = _time.time()
        last_dyn = None
        try:
            while rclpy.ok():
                if model is not None:
                    # exact standard-BlueROV2 dynamics: accelerations from our
                    # Fossen model, computed on the measured state, sent in world
                    # frame (same pattern as HoloOcean's fossen_interface)
                    mixer = 'script'
                    tau_ext = None
                    yaw_ref = None
                    att_ref = None
                    if args.control:
                        cmd6 = np.zeros(6)   # thruster path unused in control mode
                        if last_dyn is not None:
                            q = last_dyn[DYN_QUAT]
                            Rc = quat_to_rot_matrix_xyzw(q)
                            nu_meas = np.concatenate([Rc.T @ last_dyn[3:6], Rc.T @ last_dyn[12:15]])
                            tau_ext = bridge.autopilot_tau(nu_meas)
                    elif replay is not None:
                        idx = int(np.searchsorted(replay['t'], t, side='right')) - 1
                        raw = replay['cmd'][max(idx, 0)]
                        # HYBRID: ~1 s low-pass on the ZOH commands removes the 4 Hz
                        # step artifact that made the DVL/thrust spectrum too steep
                        alpha = 1.0 / (1.0 + 1.0 * ticks_per_sec)
                        replay['smooth'] += alpha * (raw - replay['smooth'])
                        cmd6 = replay['smooth']
                        mixer = 'ardusub'
                        # v11: low-pass the rate refs (~1 s) — they are raw 10 Hz gyro
                        # measurements, and servoing to their noise injects it as real
                        # torque (chatter inflated accel_x 2x via gravity/lever leak).
                        # Servo supplies MOTION; injected noise supplies noise.
                        alpha_r = 1.0 / (1.0 + 0.3 * ticks_per_sec)  # v12: 0.3 s — keep the 0.5-2 Hz rocking band (ACF carrier), still kill >3 Hz chatter
                        if replay['gz'] is not None and YAWREF_GAIN > 0:
                            tgz, gz = replay['gz']
                            raw_z = YAWREF_SIGN * float(np.interp(t, tgz, gz))
                            replay['lp_z'] = replay.get('lp_z', raw_z) + alpha_r * (raw_z - replay.get('lp_z', raw_z))
                            yaw_ref = replay['lp_z']
                        if replay['gxy'] is not None and ATTREF_GAIN > 0:
                            tg, gx, gy = replay['gxy']
                            raw_x = ATTREF_SIGN_X * float(np.interp(t, tg, gx))
                            raw_y = ATTREF_SIGN_Y * float(np.interp(t, tg, gy))
                            replay['lp_x'] = replay.get('lp_x', raw_x) + alpha_r * (raw_x - replay.get('lp_x', raw_x))
                            replay['lp_y'] = replay.get('lp_y', raw_y) + alpha_r * (raw_y - replay.get('lp_y', raw_y))
                            att_ref = (replay['lp_x'], replay['lp_y'])
                        if t > replay['t'][-1]:
                            break
                    elif args.move and last_dyn is not None:
                        cmd6 = scripted_command6(t, z=float(last_dyn[8]),
                                                 w_vert=float(last_dyn[5]))
                    elif args.move:
                        cmd6 = scripted_command6(t)
                    else:
                        cmd6 = np.zeros(6)
                    bridge.effort = float(np.mean(np.abs(cmd6)))
                    if last_dyn is not None:
                        quat = last_dyn[DYN_QUAT]
                        R = quat_to_rot_matrix_xyzw(quat)
                        nu = np.concatenate([R.T @ last_dyn[3:6], R.T @ last_dyn[12:15]])
                        tau_add = None
                        if yaw_ref is not None or att_ref is not None:
                            tau_add = np.zeros(6)
                        if yaw_ref is not None:
                            tau_add[5] = YAWREF_GAIN * (yaw_ref - nu[5])
                        if att_ref is not None:
                            tau_add[3] = ATTREF_GAIN * (att_ref[0] - nu[3])
                            tau_add[4] = ATTREF_GAIN * (att_ref[1] - nu[4])
                        nu_dot = model.step(cmd6, quat, nu,
                                            z_world=float(last_dyn[8]),
                                            surface_z=WATER_SURFACE_Z, mixer=mixer,
                                            tau_ext=tau_ext, tau_add=tau_add)
                        acc_world = np.concatenate([R @ nu_dot[:3], R @ nu_dot[3:]])
                        env.act(AGENT_NAME, acc_world)
                elif args.move:
                    env.act(AGENT_NAME, scripted_command(t))
                rclpy.spin_once(node, timeout_sec=0.0)   # process incoming /cmd_vel
                state = env.tick()
                t = float(state.get('t', t + 1.0 / ticks_per_sec))
                bridge.sim_time = t
                if 'DynamicsSensor' in state:
                    last_dyn = np.asarray(state['DynamicsSensor'], dtype=float)

                if REAL_TIME_PACE:
                    lag = t - (_time.time() - wall_start)
                    if lag > 0:
                        _time.sleep(min(lag, 0.05))

                if 'DynamicsSensor' in state:
                    bridge.on_dynamics(last_dyn)
                if 'IMUSensor' in state:
                    bridge.on_imu(np.asarray(state['IMUSensor'], dtype=float))
                if 'DVLSensor' in state:
                    bridge.on_dvl(np.asarray(state['DVLSensor'], dtype=float))
                if 'DepthSensor' in state:
                    bridge.on_depth(np.asarray(state['DepthSensor'], dtype=float))

                if args.duration and t >= args.duration:
                    break
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()
    print(f'[bridge] stopped after {t:.1f} sim-seconds')


if __name__ == '__main__':
    main()
