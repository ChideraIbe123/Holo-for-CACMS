#!/bin/bash
# Pool-focused fidelity round: replay the 2026-09-17 tub runs INSIDE the Intex
# twin, score against the 6 tub bags themselves (matched venue, matched runs).
set -e
cd ~/projects/holoocean_bridge
PY=$HOME/projects/holoocean-env/bin/python
mkdir -p ~/data/profiles_tub
for b in ~/data/tubtest/*/; do
  n=$(basename "$b")
  $PY extract_cmd_profile.py "$b" ~/data/profiles_tub/$n.npz
done
ls ~/data/profiles_tub
source /opt/ros/humble/setup.bash
OUT=$HOME/data/mc_runs_tub
rm -rf $OUT; mkdir -p $OUT
for prof in ~/data/profiles_tub/*.npz; do
  base=$(basename $prof .npz)
  DUR=$($PY -c "import numpy as np; print(min(int(np.load('$prof')['t'][-1]), 120))")
  for seed in 1 2; do
    echo "[tub] $base seed $seed (${DUR}s)"
    ($PY mavros_bridge.py --headless --replay $prof --pool intex --duration $((DUR + 5)) > /dev/null 2>&1) &
    B=$!; sleep 12
    $PY record_run.py $OUT/${base}_s${seed}.npz $((DUR - 6)) > /dev/null 2>&1
    wait $B 2>/dev/null || true
  done
done
echo "[tub] runs: $(ls $OUT | wc -l)"
$PY fidelity_scorecard.py --real ~/data/tubtest_npz/*.npz --sim $OUT/*.npz --out ~/data/scorecard_tub
echo TUB_CAMPAIGN_DONE
