"""The lab's indoor Intex pool (model 26770) as a reusable environment module.

Exact geometry from the owner's manual + published capacity: inner 400 x 200 cm,
frame height 122 cm, 8418 L design capacity -> water depth 1.05 m (surface z=0).
Used by indoor_pool_capture.py (footage) and mavros_bridge --pool (closed-loop
benchmark runs inside the actual test environment, so DVL bottom lock and wall
proximity match the real sessions).
"""

POOL_LEN = 4.0
POOL_WID = 2.0
FLOOR_Z = -1.05
WALL_TOP = 0.17
WALL_T = 0.15
X_OFF = 35.0

# in-pool spawn: course corner, heading +x. The benchmark course (relative to
# spawn) is the 2.0 x 0.6 m rectangle — the largest course that stays clear of
# the walls including corner overshoot (verified: min clearance 0.30 m).
SPAWN = [X_OFF + 1.0, -0.3, -0.5]


def spawn_pool(env):
    cx = X_OFF + POOL_LEN / 2
    env.spawn_prop("box", location=[cx, 0, FLOOR_Z - 0.2],
                   scale=[POOL_LEN, POOL_WID, 0.4], material="white")
    for y in (-POOL_WID / 2 - WALL_T / 2, POOL_WID / 2 + WALL_T / 2):
        env.spawn_prop("box", location=[cx, y, (WALL_TOP + FLOOR_Z - 0.3) / 2],
                       scale=[POOL_LEN + 2 * WALL_T, WALL_T, WALL_TOP - FLOOR_Z + 0.3],
                       material="white")
    for x in (X_OFF - WALL_T / 2, X_OFF + POOL_LEN + WALL_T / 2):
        env.spawn_prop("box", location=[x, 0, (WALL_TOP + FLOOR_Z - 0.3) / 2],
                       scale=[WALL_T, POOL_WID, WALL_TOP - FLOOR_Z + 0.3],
                       material="white")
