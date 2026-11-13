"""6-DOF Fossen dynamics model of the STANDARD (6-thruster) BlueROV2 — the lab's
actual vehicle configuration, which HoloOcean does not ship (its BlueROV2 agent is
the 8-thruster Heavy). The bridge integrates this model each tick and drives the
HoloOcean agent with the resulting accelerations, so the vehicle MOVES as a
standard BlueROV2 while the Heavy asset is only the visual shell.

Sources (see work.md):
- Thruster geometry: Blue Robotics standard config — 4 vectored @ ±45° at
  (±0.14, ±0.092, 0) m, 2 vertical at (0, ±0.109, 0.077) m
  (clydemcqueen/bluerov2_gz model.sdf, the community-standard Gazebo model).
- Rigid body / added mass / damping: Wu 2018 (Flinders thesis, Tables 5.1–5.3),
  Fossen form. Published sets vary between papers — treat as the base, and tune
  THRUST_SCALE / drag multipliers against the lab's own bag (teleop→DVL response).
- T200 thrust curve: von Benzon et al. 2022 (JMSE 10:1898, Eq. 18), normalized
  command in [-1, 1] → Newtons.

Frames: body FLU (x fwd, y left, z up) to match HoloOcean; Fossen coefficients are
axis-symmetric here so the SNAME(NED)→FLU sign differences cancel for the diagonal
terms used.
"""
import numpy as np

# ============================== PARAMETERS ==========================================
# Rigid body (Wu 2018 Table 5.1): the bare vehicle
G = 9.81
MASS_BASE = 11.5               # kg
WEIGHT_BASE = 112.8            # N
INERTIA_BASE = np.diag([0.16, 0.16, 0.16])     # kg m^2
BG = 0.02                      # m, CG below CB (Wu); sets the pitch/roll stiffness

# v19 (2026-11-13): the lab's payload and trim, from measurements.
# Water Linked DVL A50 (datasheet: 170 g in air, 105 g in water, 66 x 25 mm) at the
# lab's measured mount (BlueROV-Tools config/sensor_transforms.yaml, y-forward frame
# x=-88.9 y=-171.45 z=-114.3 mm -> FLU): 17 cm aft, 9 cm left, 11 cm below the centre.
DVL_POS = np.array([-0.171, 0.089, -0.114])
DVL_MASS_AIR = 0.170
DVL_MASS_WATER = 0.105         # net in-water weight
# Net buoyancy is NOT measured: Blue Robotics' setup guidance is "slightly positive"
# after re-ballasting for payloads; 0.2 kg (Wu's value) is kept as the assumption.
# Replace with (weight in air, waterline) when the lab measures them.
NET_BUOYANCY_KG = 0.2
# Static attitude at float, MEASURED from the tub float recordings
# (rostest_floating_armed/unarmed, 1800 samples each, sd 0.02 deg): pitch +12.72 deg
# (REP-103 FLU: positive = nose DOWN), roll +0.49 deg. Independently confirmed by the
# resting accelerometer: accel_x = -2.01 m/s^2 = -g sin(11.8 deg). Driving, the
# vehicle sits ~10 deg nose-down, so a fifth of forward thrust points at the floor
# (the "driving forward pulls it down" seen on 2026-11-12). The bare Wu body floats
# level; this trim is the single largest static difference from the lab's vehicle.
TRIM_PITCH_DEG = 12.72
TRIM_ROLL_DEG = 0.49

MASS = MASS_BASE + DVL_MASS_AIR                                   # 11.67 kg
WEIGHT = WEIGHT_BASE + DVL_MASS_AIR * G
BUOYANCY = WEIGHT + NET_BUOYANCY_KG * G
# Wu's inertia about the body origin + the DVL as a point mass (parallel axis):
# roll +2.2 %, pitch +4.5 %, yaw +3.9 %
INERTIA = INERTIA_BASE + DVL_MASS_AIR * (np.dot(DVL_POS, DVL_POS) * np.eye(3)
                                         - np.outer(DVL_POS, DVL_POS))
# CG: at rest gravity passes through CG and CB, so the measured float attitude fixes
# the DIRECTION of CG from CB (world "down" expressed in the body frame at that
# attitude); Wu's BG fixes its distance. The DVL's own CG shift (2.5 mm aft, 1.3 mm
# left, 1.7 mm down at 170 g) is part of what the lab's ballast placement produced;
# the measured attitude already contains it.
_th, _ph = np.radians(TRIM_PITCH_DEG), np.radians(TRIM_ROLL_DEG)
R_G = BG * np.array([np.sin(_th), -np.cos(_th) * np.sin(_ph), -np.cos(_th) * np.cos(_ph)])
R_B = np.array([0.0, 0.0, 0.0])                # body origin at CB

# Added mass (Wu 2018 Table 5.2), positive-definite diagonal
ADDED_MASS = np.array([5.5, 12.7, 14.57, 0.12, 0.12, 0.12])

# Damping (Wu 2018 Table 5.3), positive coefficients
D_LIN = np.array([4.03, 6.22, 5.18, 0.07, 0.07, 0.07])
D_QUAD = np.array([18.18, 21.66, 36.99, 1.55, 1.55, 1.55])
# v17: per-axis drag corrections from the 2026-09-17 single-axis steady-state
# runs (tub_sysid.py) — the first data exciting sway/heave/yaw. Scale =
# (v_model/v_measured)^2 at the tested level: surge x1.34 (0.389 vs 0.336 m/s),
# sway x3.55 (0.326 vs 0.173 — Wu coefficients + tether badly under-damp
# lateral), heave x1.54 (0.226 vs 0.182), yaw x0.38 (0.78 vs 1.27 rad/s — the
# real vehicle spins far faster than modeled). Roll/pitch unmeasured -> 1.
AXIS_DRAG_CORR = np.array([1.34, 3.55, 1.54, 1.0, 1.0, 0.38])
D_LIN = D_LIN * AXIS_DRAG_CORR
D_QUAD = D_QUAD * AXIS_DRAG_CORR

# Tuning knobs — fit these against the real vehicle's response.
# THRUST_SCALE = 0.55 accounts for installed-thrust losses (thruster-hull
# interaction + advance-speed deration vs bollard pull); calibrated so full
# thrust gives the BlueROV2's documented ~1.5 m/s top speed with Wu's drag.
THRUST_SCALE = 0.55
DRAG_SCALE = 1.0               # scales all damping
# v18 (2026-10-27): SURFACE heave damping. Wu's heave damping (8 N*s/m linear
# after the tub correction) is a deep-water value; against the surface
# buoyancy spring BUOYANCY/SURFACE_TAPER = 459 N/m and the 26.07 kg wet heave
# mass it gives zeta = 0.036, so the floating vehicle rang at 0.67 Hz for
# ~15 s after every surface arrival (Phase A measurement: ring-down from the
# spawn rise lasted to sim t = 19.6 s; the real vehicle at the surface is
# heavily damped by wave radiation + the frame's flat-plate drag). This term
# is active only within SURFACE_DAMP_FADE below full submersion and fades to
# zero by then, so the mid-column heave drag calibrated by tub_sysid (v17) is
# untouched. Value chosen for zeta ~ 0.9 of the surface mode; it is a design
# target, not a scorecard fit.
SURFACE_TAPER = 0.25           # m, hull height over which buoyancy tapers
SURFACE_HEAVE_DAMP = 190.0     # N*s/m, extra linear heave damping at the surface
SURFACE_DAMP_FADE = 0.30       # m below full submersion where it reaches zero


def static_float_z(surface_z=0.0):
    """World z at which the taper balances weight (the static float point)."""
    return surface_z + SURFACE_TAPER / 2.0 - SURFACE_TAPER * WEIGHT / BUOYANCY

# Thruster geometry, STANDARD config (6 × T200). Positions (m) and unit
# directions in body FLU. T1-T4 vectored ±45° in the horizontal plane,
# T5-T6 vertical. Signs follow ArduSub motor directions for the standard frame.
THRUSTER_POS = np.array([
    [0.14, -0.092, 0.0],       # T1 front-starboard, pushes fwd-port
    [0.14, 0.092, 0.0],        # T2 front-port, pushes fwd-starboard
    [-0.14, -0.092, 0.0],      # T3 rear-starboard
    [-0.14, 0.092, 0.0],       # T4 rear-port
    [0.0, -0.109, 0.077],      # T5 starboard vertical
    [0.0, 0.109, 0.077],       # T6 port vertical
])
_s = np.sqrt(0.5)
THRUSTER_DIR = np.array([
    [_s, _s, 0.0],
    [_s, -_s, 0.0],
    [_s, -_s, 0.0],
    [_s, _s, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 0.0, 1.0],
])
# ====================================================================================


# T200 deadband: ±30 PWM around neutral ≈ 0.075 normalized (Blue Robotics spec;
# validated against the lab bag's sustained-cruise segment — see work.md)
DEADZONE = 0.075


def t200_force(cmd):
    """Normalized command in [-1, 1] -> thrust in N (von Benzon 2022 Eq. 18,
    with the T200 deadband applied)."""
    v = np.clip(np.asarray(cmd, dtype=float), -1.0, 1.0)
    v = np.sign(v) * np.maximum(0.0, np.abs(v) - DEADZONE) / (1.0 - DEADZONE)
    return (-140.3 * v**9 + 389.9 * v**7 - 404.1 * v**5 + 176.0 * v**3 + 8.9 * v)


# ArduSub-convention thruster directions (FRD factors from AP_Motors6DOF.cpp,
# converted to FLU: lateral flips sign; verticals push down for positive output).
# Used when replaying real per-thruster PWM through the model.
ARDUSUB_DIR = np.array([
    [-0.7071, -0.7071, 0.0],
    [-0.7071, 0.7071, 0.0],
    [0.7071, -0.7071, 0.0],
    [0.7071, 0.7071, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 0.0, -1.0],
])


# Effective per-axis sign of the REAL vehicle's command->motion mapping, recovered
# from data (infer_signs_teleop.py on the March obstacle/river bags, which have rich
# sway+yaw): surge +, sway +, YAW INVERTED (teleop yaw cmd vs gyro_z: -0.73, all bags,
# ~5% sign agreement). The yaw inversion is applied in the BRIDGE (YAW_CMD_SIGN in the
# control path, YAWREF_SIGN in the replay yaw-rate servo), not here: the former
# EFFECTIVE_SIGN array in this module was never referenced and was removed (v18).


class BlueROV2StandardModel:
    """Fossen dynamics: tau = thrusters + restoring; M*nu_dot = tau - C(nu)nu - D(nu)nu.

    Stateless per tick — nu (body velocities [u v w p q r]) comes from HoloOcean's
    DynamicsSensor each step (same self-correcting pattern as HoloOcean's own
    fossen_interface), and the returned body accelerations are sent back via the
    acceleration control scheme.
    """

    def __init__(self):
        # Full mass matrix: rigid body (with CG offset) + added mass
        m, I = MASS, INERTIA
        S_rg = _skew(R_G)
        self.M = np.zeros((6, 6))
        self.M[:3, :3] = m * np.eye(3)
        self.M[:3, 3:] = -m * S_rg
        self.M[3:, :3] = m * S_rg
        self.M[3:, 3:] = I
        self.M += np.diag(ADDED_MASS)
        self.M_inv = np.linalg.inv(self.M)

        # Thruster mixing matrices: tau = T @ forces (6x6)
        self.T = np.zeros((6, 6))
        self.T_ardusub = np.zeros((6, 6))
        for i in range(6):
            self.T[:3, i] = THRUSTER_DIR[i]
            self.T[3:, i] = np.cross(THRUSTER_POS[i], THRUSTER_DIR[i])
            self.T_ardusub[:3, i] = ARDUSUB_DIR[i]
            self.T_ardusub[3:, i] = np.cross(THRUSTER_POS[i], ARDUSUB_DIR[i])

    def step(self, thruster_cmd, quat_xyzw, nu_body, z_world=-10.0, surface_z=0.0,
             mixer='script', tau_ext=None, tau_add=None):
        """Compute body accelerations for the current tick.

        Args:
            thruster_cmd: 6 normalized commands in [-1, 1] (T1..T6)
            quat_xyzw: current body->world attitude
            nu_body: measured body velocities [u, v, w, p, q, r]
            z_world: current world z of the vehicle (for surface handling)
            surface_z: world z of the water surface

        Returns:
            nu_dot: 6-vector of body accelerations [lin(3), ang(3)]
        """
        nu = np.asarray(nu_body, dtype=float)
        if tau_ext is not None:
            # closed-loop control: body-force setpoint from the velocity autopilot,
            # bypassing the thruster polynomial (the autopilot produces whatever
            # force it needs, within the saturation the caller applied)
            tau = np.asarray(tau_ext, dtype=float).copy()
        else:
            forces = THRUST_SCALE * t200_force(thruster_cmd)
            tau = (self.T_ardusub if mixer == 'ardusub' else self.T) @ forces
            # NOTE: raw-PWM replay can't reproduce yaw (geometric mixer is degenerate
            # for yaw vs ArduSub's real mixing; verify_yaw_fix.py -> ~0 corr). The
            # yaw-sign finding is applied in the CONTROL path instead (bridge autopilot).
        if tau_add is not None:
            # additive body wrench on top of the thruster/setpoint forces — the
            # bridge's replay servos (yaw-rate, roll/pitch-rate, heave-rate and the
            # opt-in absolute-depth servo) and the opt-in tether pull. Surge/sway
            # stay thruster-driven.
            tau += np.asarray(tau_add, dtype=float)

        # Restoring forces: gravity & buoyancy rotated into body frame (FLU, z up).
        # Buoyancy tapers to zero as the hull breaches the surface (vehicle
        # height ~0.25 m), so the model floats AT the surface like the real one.
        submerged = np.clip((surface_z - z_world + SURFACE_TAPER / 2.0) / SURFACE_TAPER, 0.0, 1.0)
        R = _quat_to_rot(quat_xyzw)            # body -> world
        g_world = np.array([0.0, 0.0, -WEIGHT])
        b_world = np.array([0.0, 0.0, BUOYANCY * submerged])
        f_rest = R.T @ (g_world + b_world)
        m_rest = np.cross(R_G, R.T @ g_world) + np.cross(R_B, R.T @ b_world)
        tau += np.concatenate([f_rest, m_rest])

        # Coriolis (rigid body + added mass, diagonal approximation) and damping
        tau -= _coriolis(nu) @ nu
        damping = DRAG_SCALE * (D_LIN + D_QUAD * np.abs(nu))
        # v18: surface heave damping (see SURFACE_HEAVE_DAMP); fades out with
        # depth below full submersion so deep-water heave drag is unchanged
        below_full = (surface_z - z_world) - SURFACE_TAPER / 2.0
        surf = np.clip(1.0 - below_full / SURFACE_DAMP_FADE, 0.0, 1.0)
        damping[2] += SURFACE_HEAVE_DAMP * surf
        tau -= damping * nu

        return self.M_inv @ tau


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _coriolis(nu):
    m = MASS
    u, v, w, p, q, r = nu
    Xu, Yv, Zw, Kp, Mq, Nr = ADDED_MASS
    C = np.zeros((6, 6))
    # rigid body part (origin at CG approximation)
    C[:3, 3:] = -m * _skew([u, v, w])
    C[3:, :3] = -m * _skew([u, v, w])
    C[3:, 3:] = -_skew(INERTIA @ [p, q, r])
    # added mass part
    C[:3, 3:] += _skew([Xu * u, Yv * v, Zw * w])
    C[3:, :3] += _skew([Xu * u, Yv * v, Zw * w])
    C[3:, 3:] += _skew([Kp * p, Mq * q, Nr * r])
    return C
