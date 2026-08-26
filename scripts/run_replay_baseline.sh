#!/bin/bash
# Real-data baseline: replay the real BlueROV2 bag through the lab's dead-reckon
# pipeline; record our /deadreckon/odom next to the DVL vendor's onboard DR pose.
BAG=~/data/rosbag_20260504_154118/rosbag_20260504_154118_0.mcap
OUT=~/data/baseline_out
cd ~/projects/holoocean_bridge
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/projects/bluerov-tools/bluerov_dr:$PYTHONPATH"
PY=~/projects/holoocean-env/bin/python
mkdir -p $OUT

echo "[baseline] starting dead_reckon (st_car_ekf, default params)..."
python3 ~/projects/bluerov-tools/bluerov_dr/bluerov_dr/dead_reckon.py > $OUT/dr.log 2>&1 &
DR=$!
pkill -f trajectory_recorder.py 2>/dev/null
echo "[baseline] starting trajectory recorder..."
$PY -u trajectory_recorder.py $OUT > $OUT/recorder.log 2>&1 &
REC=$!
sleep 5

echo "[baseline] replaying bag (real-time)..."
$PY bag_replayer.py $BAG 1.0
sleep 2

kill -INT $REC 2>/dev/null   # rclpy handles SIGINT -> spin returns -> CSV save + analysis
sleep 4
wait $REC 2>/dev/null
kill $DR 2>/dev/null

echo "=== recorder output ==="
cat $OUT/recorder.log
echo "=== dead_reckon log (first lines) ==="
grep -v "^$" $OUT/dr.log | head -4
echo "[baseline] COMPLETE — CSVs in $OUT"
