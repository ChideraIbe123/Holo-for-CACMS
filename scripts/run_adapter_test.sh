#!/bin/bash
# E2E test of lab_controller_adapter.py (PassthroughP law through the plugin
# path), mirroring run_benchmark.sh run_one. Pass = tracks the square with RMSE
# in the stand-in band.
OUT=$HOME/data/adapter_test
mkdir -p $OUT
cd ~/projects/holoocean_bridge
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/projects/bluerov-tools/bluerov_dr:$PYTHONPATH"
PY=$HOME/projects/holoocean-env/bin/python
tag=passthrough_s1
$PY mavros_bridge.py --headless --control --duration 90 > $OUT/${tag}_bridge.log 2>&1 &
BR=$!
for i in $(seq 1 90); do ros2 topic list 2>/dev/null | grep -q /mavros/imu/data && break; sleep 2; done
python3 ~/projects/bluerov-tools/bluerov_dr/bluerov_dr/dead_reckon.py \
  --ros-args -p estimator_mode:=legacy_integrator > $OUT/${tag}_dr.log 2>&1 &
DR=$!
sleep 4
$PY lab_controller_adapter.py --name passthrough --law self:PassthroughP > $OUT/${tag}_ctrl.log 2>&1 &
CT=$!
$PY sim_traj_recorder.py $OUT/$tag > $OUT/${tag}_rec.log 2>&1 &
RC=$!
wait $BR
kill -INT $RC 2>/dev/null; sleep 3; kill $DR $CT 2>/dev/null
$PY score_ranking.py $OUT
echo ADAPTER_E2E_DONE
