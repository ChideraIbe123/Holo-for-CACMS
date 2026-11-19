#!/bin/bash
# One recorded controller run on the real vehicle.
#   ./run_one.sh <law> <course> <rep>         e.g.  ./run_one.sh los line 1
# law: p | smc | pursuit | los | ilos | alos | fuzzy_lab      course: rect | line
# Resets the estimator, starts a bag, runs the controller until it finishes, stops the bag.
# cmdvel_to_manual.py must already be running in another terminal, vehicle armed.
# Optional environment: DEPTH (default -0.5), LINE_LENGTH (default 2.4), OUT (default ./runs),
# HEADING (default placed; saved = the 2026-11-12 behaviour)
# DEPTH is metres below the FLOATING position: the estimator zeroes depth at the reset
# sent below, so the vehicle must be floating freely when this script starts.
# Start-up (--init, 2026-11-12): before the course the runner sinks to DEPTH and holds the
# course heading; the course starts only when the depth sensor (within 5 cm) and the
# heading (within 6 deg) both hold for 1.0 s.
# HEADING=placed (default since 2026-11-19): the course heading is the heading the vehicle
# was placed at, so aim it along the pool by hand; no turn. HEADING=saved: turn on the
# spot to the heading saved with `python3 save_heading.py` (the 2026-11-13 line session
# ran this way; the compass heading for the same placement wandered 60 deg between runs).
# If it cannot settle in 45 s nothing runs and the folder is renamed *_ABORTED_<time>,
# so the same command can simply be run again.
set -e
LAW=${1:?law}; COURSE=${2:?course}; REP=${3:?rep}
HERE=$(cd "$(dirname "$0")" && pwd)
TAG=${LAW}_${COURSE}_r${REP}
OUT=${OUT:-$HERE/runs}/$TAG
if [ -e "$OUT" ]; then echo "$OUT already exists: pick another rep number"; exit 1; fi
mkdir -p "$OUT"
TOPICS="/deadreckon/odom /cmd_vel /auto/cmd /mavros/rc/out /mavros/imu/data /mavros/state
/mavros/battery /mavros/global_position/rel_alt /dvl/twist /dvl/altitude /dvl/fom /dvl/velocity_valid
/camera/image_raw /camera/camera_info /echo/image_fan /tf_static /course/origin /auto/status"
ros2 topic pub /deadreckon/reset std_msgs/msg/Empty "{}" -1 > /dev/null
sleep 1
ros2 bag record -o "$OUT/bag" $TOPICS > "$OUT/bag.log" 2>&1 &
BAG=$!
sleep 3
DEPTH=${DEPTH:--0.5}
HEADING=${HEADING:-placed}
echo "=== $TAG: start-up: sinking to $DEPTH m (less if the floor is closer than 0.30 m below that), heading: $HEADING ==="
RC=0
python3 "$HERE/course_runner.py" --law "$LAW" --course "$COURSE" --name "$TAG" \
    --depth-target "$DEPTH" --init --heading "$HEADING" --line-length "${LINE_LENGTH:-2.4}" --log "$OUT/runner.csv" || RC=$?
sleep 1
# SIGINT is ignored by background jobs from non-interactive shells (session
# 2026-10-08 finding: orphaned recorders wrote ~700 GB); SIGTERM works.
kill -TERM $BAG 2>/dev/null; wait $BAG 2>/dev/null || true
if [ "$RC" -eq 3 ] || [ "$RC" -eq 2 ]; then
    mv "$OUT" "${OUT}_ABORTED_$(date +%H%M%S)"
    echo "=== $TAG: start-up did not settle, course NOT run. Folder renamed ${OUT}_ABORTED_*; run the same command again ==="
    exit 1
fi
echo "=== $TAG: done. $(tail -1 "$OUT/runner.csv" | cut -d, -f1) s logged -> $OUT ==="
