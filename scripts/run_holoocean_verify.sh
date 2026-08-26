#!/bin/bash
# End-to-end verification: live HoloOcean -> mavros_bridge -> topic checks ->
# lab dead-reckon node -> drift vs sim ground truth.
cd ~/projects/holoocean_bridge
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/tmp/imu_verify/stubs:$PYTHONPATH"

echo "[verify] starting HoloOcean bridge (headless, scripted motion)..."
~/projects/holoocean-env/bin/python mavros_bridge.py --headless --move > bridge.log 2>&1 &
BRIDGE=$!

echo "[verify] waiting for /mavros/imu/data (up to 5 min for first Unreal boot)..."
found=""
for i in $(seq 1 150); do
  if ! kill -0 $BRIDGE 2>/dev/null; then
    echo "[verify] BRIDGE DIED — log tail:"; tail -30 bridge.log; exit 1
  fi
  if ros2 topic list 2>/dev/null | grep -q '^/mavros/imu/data$'; then found=1; break; fi
  sleep 2
done
[ -z "$found" ] && { echo "[verify] TOPIC NEVER APPEARED"; tail -30 bridge.log; kill $BRIDGE; exit 1; }

echo "=== IMU message ==="
timeout 15 ros2 topic echo /mavros/imu/data --once
echo "=== DVL message ==="
timeout 15 ros2 topic echo /dvl/twist --once
echo "=== rel_alt message ==="
timeout 15 ros2 topic echo /mavros/global_position/rel_alt --once
echo "=== ground truth ==="
timeout 15 ros2 topic echo /holoocean/ground_truth --once | head -15
echo "=== rates ==="
timeout 12 ros2 topic hz /mavros/imu/data 2>&1 | tail -2
timeout 12 ros2 topic hz /dvl/twist 2>&1 | tail -2

echo "[verify] starting lab dead-reckon node (legacy mode)..."
python3 ~/tmp/imu_verify/dvl_dead_reckon.py --ros-args -p estimator_mode:=legacy_integrator > dr.log 2>&1 &
DR=$!
sleep 5
echo "=== dead-reckon odom (A) + ground truth (A) ==="
timeout 10 ros2 topic echo /deadreckon/odom --once | grep -A4 "position:" | head -5
timeout 10 ros2 topic echo /holoocean/ground_truth --once | grep -A4 "position:" | head -5
sleep 20
echo "=== dead-reckon odom (B, +20s) + ground truth (B) ==="
timeout 10 ros2 topic echo /deadreckon/odom --once | grep -A4 "position:" | head -5
timeout 10 ros2 topic echo /holoocean/ground_truth --once | grep -A4 "position:" | head -5

kill $DR $BRIDGE 2>/dev/null
sleep 2; kill -9 $BRIDGE 2>/dev/null
echo "=== bridge log tail ==="; tail -10 bridge.log
echo "[verify] COMPLETE"
