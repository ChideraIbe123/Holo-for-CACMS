#!/bin/bash
# Closed-loop test of the lab's dead_reckon node in its DEFAULT st_car_ekf mode
# (real bluerov_dr package, no stubs) against the live HoloOcean bridge.
cd ~/projects/holoocean_bridge
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/projects/bluerov-tools/bluerov_dr:$PYTHONPATH"

echo "[ekf] starting HoloOcean bridge (headless, scripted motion)..."
~/projects/holoocean-env/bin/python mavros_bridge.py --headless --move > bridge_ekf.log 2>&1 &
BRIDGE=$!

echo "[ekf] waiting for /mavros/imu/data..."
found=""
for i in $(seq 1 150); do
  if ! kill -0 $BRIDGE 2>/dev/null; then echo "[ekf] BRIDGE DIED"; tail -20 bridge_ekf.log; exit 1; fi
  if ros2 topic list 2>/dev/null | grep -q '^/mavros/imu/data$'; then found=1; break; fi
  sleep 2
done
[ -z "$found" ] && { echo "[ekf] TOPIC NEVER APPEARED"; kill $BRIDGE; exit 1; }

echo "[ekf] starting dead_reckon in st_car_ekf mode (default)..."
python3 ~/projects/bluerov-tools/bluerov_dr/bluerov_dr/dead_reckon.py > dr_ekf.log 2>&1 &
DR=$!
sleep 8

if ! kill -0 $DR 2>/dev/null; then
  echo "[ekf] DEAD_RECKON DIED — log:"; tail -30 dr_ekf.log; kill $BRIDGE; exit 1
fi

echo "=== EKF profile/situation topics ==="
timeout 8 ros2 topic echo /deadreckon/st_car_ekf/profile --once
timeout 8 ros2 topic echo /deadreckon/st_car_ekf/situation --once
echo "=== odom (A) with covariance ==="
timeout 10 ros2 topic echo /deadreckon/odom --once | grep -A4 "position:\|orientation:" | head -10
timeout 10 ros2 topic echo /deadreckon/odom --once | grep -A7 "covariance" | head -8
echo "=== ground truth (A) ==="
timeout 10 ros2 topic echo /holoocean/ground_truth --once | grep -A4 "position:" | head -5
sleep 20
echo "=== odom (B, +20s) ==="
timeout 10 ros2 topic echo /deadreckon/odom --once | grep -A4 "position:" | head -5
echo "=== ground truth (B) ==="
timeout 10 ros2 topic echo /holoocean/ground_truth --once | grep -A4 "position:" | head -5
echo "=== odom publish rate ==="
timeout 10 ros2 topic hz /deadreckon/odom 2>&1 | tail -2

kill $DR $BRIDGE 2>/dev/null
sleep 2; kill -9 $BRIDGE 2>/dev/null
echo "=== dead_reckon log ==="
grep -v "^$" dr_ekf.log | head -8
echo "[ekf] COMPLETE"
