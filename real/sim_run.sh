#!/bin/bash
# One closed-loop run in the Intex pool twin with a course_runner law.
# Usage: sim_run.sh <law> <course rect|line> <mode direct|sticks> <tag> [out_dir]
# direct: runner -> /cmd_vel -> sim.   sticks: runner -> /cmd_vel_ctrl -> cmdvel_to_manual --loopback -> /cmd_vel -> sim
# INIT=1: the real-vehicle start-up in the twin. Spawns floating (BRIDGE_ARGS gets
#   --spawn-surface), resets the estimator at the float point like run_one.sh does, then
#   runs course_runner --init (sink + turn to the saved heading, sensor-verified, floor
#   guard from /dvl/altitude) with a heading reference of 0 (the spawn heading).
#   Without INIT the pre-Amendment-6 path is unchanged: spawn at depth, course at once.
LAW=$1; COURSE=$2; MODE=$3; TAG=$4; OUT=${5:-$HOME/data/pilot_sim}
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(dirname "$HERE")
PY=${PY:-$HOME/holoocean-venv/bin/python}
BLUEROV_DR=${BLUEROV_DR:-$HOME/lab/BlueROV-Tools/bluerov_dr}
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-73}
export PYTHONPATH="$BLUEROV_DR:$PYTHONPATH"
if ps -eo comm,args | awk '$1 ~ /^python/ && /mavros_bridge/' | grep -q .; then echo "a bridge is already running"; exit 1; fi
D=$OUT/$TAG; mkdir -p $D
cd $REPO/sim
[ "${INIT:-0}" = "1" ] && BRIDGE_ARGS="--spawn-surface $BRIDGE_ARGS"
timeout 260 $PY mavros_bridge.py --headless --control --pool intex --duration $([ "${INIT:-0}" = "1" ] && echo 170 || echo 110) $BRIDGE_ARGS > $D/bridge.log 2>&1 & BR=$!
for i in $(seq 1 60); do ros2 topic list 2>/dev/null | grep -q /mavros/imu/data && break; sleep 2; done
$PY $BLUEROV_DR/bluerov_dr/dead_reckon.py --ros-args -p estimator_mode:=legacy_integrator > $D/dr.log 2>&1 & DR=$!
sleep 4
$PY $REPO/tools/sim_traj_recorder.py $D > $D/rec.log 2>&1 & RC=$!
ST=""
if [ "$MODE" = "sticks" ]; then
  FENCE="-0.4,2.4,-0.4,1.0"; [ "$COURSE" = "line" ] && FENCE="-0.4,2.9,-0.4,0.8"
  $PY $HERE/cmdvel_to_manual.py --loopback --in-topic /cmd_vel_ctrl --depth-mode passthrough --fence="$FENCE" $STICK_ARGS > $D/sticks.log 2>&1 & ST=$!
  TOPIC=/cmd_vel_ctrl
else TOPIC=/cmd_vel; fi
sleep 1
if [ "${INIT:-0}" = "1" ]; then
  sleep 6                                     # let the float settle before the estimator is zeroed there
  ros2 topic pub /deadreckon/reset std_msgs/msg/Empty "{}" -1 > /dev/null 2>&1
  printf '{"heading": {"yaw_rad": 0.0, "yaw_deg": 0.0, "note": "sim spawn heading"}}' > $D/heading_ref.json
  # HEADING=placed|saved as on the vehicle; identical here (spawn heading = saved heading = 0)
  RUNNER_ARGS="--init --heading ${HEADING:-saved} --heading-ref $D/heading_ref.json $RUNNER_ARGS"
fi
$PY $HERE/course_runner.py --law $LAW --course $COURSE --name $TAG --depth-target -0.5 --cmd-topic $TOPIC --log $D/runner.csv $RUNNER_ARGS > $D/runner.log 2>&1
sleep 2
kill -INT $RC 2>/dev/null; sleep 2; kill $DR $ST 2>/dev/null; kill -INT $BR 2>/dev/null; sleep 3; kill $BR 2>/dev/null
wait 2>/dev/null
echo "[sim_run] $TAG: $(grep -hoE '(course complete|time limit).*' $D/runner.log | tail -1)"
