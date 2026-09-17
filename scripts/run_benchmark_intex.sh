#!/bin/bash
# Controller-ranking benchmark IN THE INTEX POOL TWIN (the real test environment):
# its ground-truth trajectory. Repeat each with 2 seeds (stand-in for run-to-run
# variability). score_ranking.py then ranks them by cross-track error.
OUT=$HOME/data/benchmark_intex
mkdir -p $OUT
cd ~/projects/holoocean_bridge
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/projects/bluerov-tools/bluerov_dr:$PYTHONPATH"
PY=~/projects/holoocean-env/bin/python
DUR=90

# name : law : kp_yaw : kd_yaw : cruise
# All controllers use yaw-sign -1: the real vehicle's yaw command/response is
# inverted (teleop-data finding, corr -0.73), and a direct closed-loop test
# confirmed the sim velocity-setpoint path needs the same flip to track (with
# +1 the vehicle diverged to -7 m; with -1 it tracks the square). YAW_SIGN below.
YAW_SIGN=-1
CONTROLLERS=(
  "smc_vonbenzon:smc:0:0:0.30"
  "tight_pd:pd:1.6:0.8:0.30"
  "pursuit:pursuit:1.4:0.0:0.32"
  "p_baseline:p:1.4:0.0:0.30"
  "aggressive:p:3.0:0.0:0.50"
  "sluggish:p:0.7:0.0:0.25"
)

run_one () {
  local name=$1 law=$2 kp=$3 kd=$4 cruise=$5 seed=$6
  local tag=${name}_s${seed}
  $PY mavros_bridge.py --headless --control --pool intex --duration $DUR > $OUT/${tag}_bridge.log 2>&1 &
  local BR=$!
  for i in $(seq 1 90); do ros2 topic list 2>/dev/null | grep -q /mavros/imu/data && break; sleep 2; done
  python3 ~/projects/bluerov-tools/bluerov_dr/bluerov_dr/dead_reckon.py \
    --ros-args -p estimator_mode:=legacy_integrator > $OUT/${tag}_dr.log 2>&1 &
  local DR=$!
  sleep 4
  $PY waypoint_controller.py --name $name --law $law --kp-yaw $kp --kd-yaw $kd --cruise $cruise \
    --yaw-sign $YAW_SIGN > $OUT/${tag}_ctrl.log 2>&1 &
  local CT=$!
  $PY sim_traj_recorder.py $OUT/$tag > $OUT/${tag}_rec.log 2>&1 &
  local RC=$!
  wait $BR
  kill -INT $RC 2>/dev/null; sleep 3; kill $DR $CT 2>/dev/null
  echo "[bench] done $tag"
}

for spec in "${CONTROLLERS[@]}"; do
  IFS=":" read -r name law kp kd cruise <<< "$spec"
  for seed in 1 2; do
    run_one "$name" "$law" "$kp" "$kd" "$cruise" "$seed"
  done
done
echo "[bench] ALL DONE -> $OUT"
