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


# A pipe for camera-driven tests (mavros_bridge --pipe). Dark, 2.4 m long, 0.1 m across,
# resting on the floor at 12 degrees to the pool's long axis. It starts just ahead and to
# the left of the spawn point, like the real "pipeline at an angle in the middle of the
# pool". Built from short axis-aligned segments: spawn_prop stood a single rotated box on
# its end (observed 2026-10-06), so no rotation is used.
PIPE_LEN = 2.4
PIPE_YAW_DEG = 12.0
PIPE_START = [X_OFF + 1.3, -0.15]      # spawn is at X_OFF + 1.0, y = -0.3
PIPE_SEGMENTS = 24


def pipe_points():
    """Centre-line end points of the pipe in world x, y (for scoring and plots)."""
    import math
    c, s = math.cos(math.radians(PIPE_YAW_DEG)), math.sin(math.radians(PIPE_YAW_DEG))
    return (PIPE_START[0], PIPE_START[1]), (PIPE_START[0] + PIPE_LEN * c, PIPE_START[1] + PIPE_LEN * s)


def spawn_pipe(env):
    (x0, y0), (x1, y1) = pipe_points()
    n = PIPE_SEGMENTS
    for i in range(n):
        f = (i + 0.5) / n
        env.spawn_prop("box", location=[x0 + f * (x1 - x0), y0 + f * (y1 - y0), FLOOR_Z + 0.05],
                       scale=[(x1 - x0) / n + 0.02, 0.1, 0.1], material="black")
