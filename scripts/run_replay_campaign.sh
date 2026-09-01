#!/bin/bash
# Campaign v4: replay real command profiles with excitation-dependent noise
OUT=$HOME/data/mc_runs4
mkdir -p $OUT
cd ~/projects/holoocean_bridge
source /opt/ros/humble/setup.bash
PY=~/projects/holoocean-env/bin/python
for prof in ~/data/profiles/*.npz; do
  base=$(basename $prof .npz)
  DUR=$($PY -c "import numpy as np; print(int(np.load('$prof')['t'][-1]))")
  for seed in 1 2 3 4 5; do
    echo "[v4] $base seed $seed (${DUR}s)"
    ($PY mavros_bridge.py --headless --replay $prof --duration $((DUR + 5)) > /dev/null 2>&1) &
    B=$!
    sleep 12
    $PY record_run.py $OUT/${base}_s${seed}.npz $((DUR - 6)) > /dev/null 2>&1
    wait $B 2>/dev/null
  done
done
ls $OUT | wc -l
