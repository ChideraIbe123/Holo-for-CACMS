#!/bin/bash
# Monte Carlo campaign: N sim runs with varied trajectories, each recorded to npz.
# Usage: bash run_mc_campaign.sh <N> <out_dir>
N=${1:-24}
OUT=${2:-$HOME/data/mc_runs}
mkdir -p $OUT
cd ~/projects/holoocean_bridge
source /opt/ros/humble/setup.bash
PY=~/projects/holoocean-env/bin/python

for i in $(seq 1 $N); do
  # vary trajectory: cruise 0.30-0.60, turn asymmetry 82-96% of cruise, 35-55 s
  CRUISE=$(python3 -c "import random; random.seed($i); print(round(random.uniform(0.30,0.60),3))")
  TURN=$(python3 -c "import random; random.seed($i+100); print(round($CRUISE*random.uniform(0.82,0.96),3))")
  DUR=$(python3 -c "import random; random.seed($i+200); print(random.randint(35,55))")
  echo "[mc] run $i/$N cruise=$CRUISE turn=$TURN dur=${DUR}s"
  ($PY mavros_bridge.py --headless --move --cruise $CRUISE --turn $TURN --duration $DUR > /dev/null 2>&1) &
  BRIDGE=$!
  sleep 12
  $PY record_run.py $OUT/sim_run_$(printf %02d $i).npz $((DUR - 8)) > /dev/null 2>&1
  wait $BRIDGE 2>/dev/null
done
ls $OUT | wc -l
echo "[mc] campaign complete -> $OUT"
