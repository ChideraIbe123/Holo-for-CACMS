#!/bin/bash
# One recorded controller run on the real vehicle.
#   ./run_one.sh <law> <course> <rep>         e.g.  ./run_one.sh los line 1
# law: p | pursuit | los | ilos | fuzzy_lab      course: rect | line
# Resets the estimator, starts a bag, runs the controller until it finishes, stops the bag.
# cmdvel_to_manual.py must already be running in another terminal, vehicle armed.
# Optional environment: DEPTH (default -0.3), LINE_LENGTH (default 2.4), OUT (default ./runs)
set -e
LAW=${1:?law}; COURSE=${2:?course}; REP=${3:?rep}
HERE=$(cd "$(dirname "$0")" && pwd)
TAG=${LAW}_${COURSE}_r${REP}
OUT=${OUT:-$HERE/runs}/$TAG
if [ -e "$OUT" ]; then echo "$OUT already exists: pick another rep number"; exit 1; fi
mkdir -p "$OUT"
TOPICS="/deadreckon/odom /cmd_vel /auto/cmd /mavros/rc/out /mavros/imu/data /mavros/state
/mavros/battery /mavros/global_position/rel_alt /dvl/twist /dvl/altitude /dvl/fom /dvl/velocity_valid
/camera/image_raw /camera/camera_info /echo/image_fan /tf_static"
ros2 topic pub /deadreckon/reset std_msgs/msg/Empty "{}" -1 > /dev/null
sleep 1
ros2 bag record -o "$OUT/bag" $TOPICS > "$OUT/bag.log" 2>&1 &
BAG=$!
sleep 3
echo "=== $TAG: controller starting ==="
python3 "$HERE/course_runner.py" --law "$LAW" --course "$COURSE" --name "$TAG" \
    --depth-target "${DEPTH:--0.3}" --line-length "${LINE_LENGTH:-2.4}" --log "$OUT/runner.csv" || true
sleep 1
kill -INT $BAG 2>/dev/null; wait $BAG 2>/dev/null || true
echo "=== $TAG: done. $(tail -1 "$OUT/runner.csv" | cut -d, -f1) s logged -> $OUT ==="
