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
# Rigid body (Wu 2018 Table 5.1)
MASS = 11.5                    # kg
WEIGHT = 112.8                 # N
BUOYANCY = 114.8               # N  (net ~0.2 kg positive, like the real vehicle)
INERTIA = np.diag([0.16, 0.16, 0.16])          # kg m^2
R_G = np.array([0.0, 0.0, -0.02])              # CG 2 cm BELOW CB (FLU: down = -z)
R_B = np.array([0.0, 0.0, 0.0])                # body origin at CB

# Added mass (Wu 2018 Table 5.2), positive-definite diagonal
ADDED_MASS = np.array([5.5, 12.7, 14.57, 0.12, 0.12, 0.12])

# Damping (Wu 2018 Table 5.3), positive coefficients
D_LIN = np.array([4.03, 6.22, 5.18, 0.07, 0.07, 0.07])
D_QUAD = np.array([18.18, 21.66, 36.99, 1.55, 1.55, 1.55])

# Tuning knobs — fit these against the real vehicle's response.
# THRUST_SCALE = 0.55 accounts for installed-thrust losses (thruster-hull
# interaction + advance-speed deration vs bollard pull); calibrated so full
# thrust gives the BlueROV2's documented ~1.5 m/s top speed with Wu's drag.
THRUST_SCALE = 0.55
DRAG_SCALE = 1.0               # scales all damping

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
             mixer='script'):
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
        forces = THRUST_SCALE * t200_force(thruster_cmd)
        tau = (self.T_ardusub if mixer == 'ardusub' else self.T) @ forces

        # Restoring forces: gravity & buoyancy rotated into body frame (FLU, z up).
        # Buoyancy tapers to zero as the hull breaches the surface (vehicle
        # height ~0.25 m), so the model floats AT the surface like the real one.
        submerged = np.clip((surface_z - z_world + 0.125) / 0.25, 0.0, 1.0)
        R = _quat_to_rot(quat_xyzw)            # body -> world
        g_world = np.array([0.0, 0.0, -WEIGHT])
        b_world = np.array([0.0, 0.0, BUOYANCY * submerged])
        f_rest = R.T @ (g_world + b_world)
        m_rest = np.cross(R_G, R.T @ g_world) + np.cross(R_B, R.T @ b_world)
        tau += np.concatenate([f_rest, m_rest])

        # Coriolis (rigid body + added mass, diagonal approximation) and damping
        tau -= _coriolis(nu) @ nu
        damping = DRAG_SCALE * (D_LIN + D_QUAD * np.abs(nu))
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
