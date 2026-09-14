#!/bin/bash
# End-to-end closed-loop test: sim(control mode) -> sensors -> lab dead_reckon ->
# /deadreckon/odom -> waypoint controller -> /cmd_vel -> sim. Records both the
# dead-reckoning estimate and sim ground truth.
OUT=$HOME/data/closedloop
mkdir -p $OUT
cd ~/projects/holoocean_bridge
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/projects/bluerov-tools/bluerov_dr:$PYTHONPATH"
PY=~/projects/holoocean-env/bin/python

echo "[cl] starting sim in closed-loop control mode..."
$PY mavros_bridge.py --headless --control --duration 120 > $OUT/bridge.log 2>&1 &
BRIDGE=$!
# wait for topics
for i in $(seq 1 150); do
  ros2 topic list 2>/dev/null | grep -q '/mavros/imu/data' && break; sleep 2
  kill -0 $BRIDGE 2>/dev/null || { echo "[cl] bridge died"; tail -20 $OUT/bridge.log; exit 1; }
done

echo "[cl] starting lab dead_reckon (legacy integrator)..."
python3 ~/projects/bluerov-tools/bluerov_dr/bluerov_dr/dead_reckon.py \
  --ros-args -p estimator_mode:=legacy_integrator > $OUT/dr.log 2>&1 &
DR=$!
sleep 4
echo "[cl] starting waypoint controller..."
$PY waypoint_controller.py 0.3 > $OUT/ctrl.log 2>&1 &
CTRL=$!
echo "[cl] recording sim trajectory..."
$PY sim_traj_recorder.py $OUT > $OUT/rec.log 2>&1 &
REC=$!

wait $BRIDGE
kill -INT $REC 2>/dev/null; sleep 3
kill $DR $CTRL 2>/dev/null

echo "=== controller published? ==="; grep -c "" $OUT/ctrl.log 2>/dev/null
$PY - <<PYEOF
import numpy as np, glob, os
d = os.path.join("$OUT","sim_gt.csv")
if os.path.exists(d):
    a = np.genfromtxt(d, delimiter=",", names=True)
    x,y = a["x"]-a["x"][0], a["y"]-a["y"][0]
    print(f"[cl] ground-truth path: x {x.min():.1f}..{x.max():.1f}, y {y.min():.1f}..{y.max():.1f}, {len(x)} samples")
    laps = (x.max()>2.3 and y.max()>2.3)
    print("[cl] SQUARE COURSE FOLLOWED" if laps else "[cl] course incomplete")
PYEOF
echo "[cl] COMPLETE -> $OUT"
