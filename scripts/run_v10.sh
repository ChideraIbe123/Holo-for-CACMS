#!/bin/bash
# v10: full body-rate trajectory matching (roll/pitch/yaw servos) + campaign.
# Prereq: profiles regenerated with gx/gy/gz (extract_cmd_profile.py v10),
# new bridge deployed. Sign check gates all 3 axes before spending the campaign.
set -e
cd ~/projects/holoocean_bridge
source /opt/ros/humble/setup.bash
PY=$HOME/projects/holoocean-env/bin/python

PROF=~/data/profiles/rosbag_20260310_151023.npz
DUR=$($PY -c "import numpy as np; print(int(np.load('$PROF')['t'][-1]))")
($PY mavros_bridge.py --headless --replay $PROF --duration $((DUR + 5)) > /dev/null 2>&1) &
B=$!; sleep 12
$PY record_run.py ~/data/signcheck10.npz $((DUR - 6)) > /dev/null 2>&1
wait $B 2>/dev/null || true
$PY - << "PYEOF"
import numpy as np
import os
H = os.path.expanduser("~")
sim = np.load(H + "/data/signcheck10.npz")
prof = np.load(H + "/data/profiles/rosbag_20260310_151023.npz")
t = sim["imu"][:, 0]
ok = True
for name, col, key in (("gyro_x", 1, "gx"), ("gyro_y", 2, "gy"), ("gyro_z", 3, "gz")):
    ref = np.interp(t, prof["t_gz"], prof[key])
    c = float(np.corrcoef(sim["imu"][:, col], ref)[0, 1])
    print("[v10] sign check %s: corr = %+.3f" % (name, c))
    ok = ok and c > 0.4
assert ok, "A RATE-SERVO AXIS SIGN/GAIN IS WRONG - aborting before campaign"
PYEOF

OUT=$HOME/data/mc_runs11
rm -rf $OUT; mkdir -p $OUT
for prof in ~/data/profiles/*.npz; do
  base=$(basename $prof .npz)
  DUR=$($PY -c "import numpy as np; print(int(np.load('$prof')['t'][-1]))")
  for seed in 1 2; do
    echo "[v10] $base seed $seed (${DUR}s)"
    ($PY mavros_bridge.py --headless --replay $prof --duration $((DUR + 5)) > /dev/null 2>&1) &
    B=$!; sleep 12
    $PY record_run.py $OUT/${base}_s${seed}.npz $((DUR - 6)) > /dev/null 2>&1
    wait $B 2>/dev/null || true
  done
done
echo "[v10] runs done: $(ls $OUT | wc -l)"
$PY fidelity_scorecard.py --real ~/data/real_npz/*.npz --sim $OUT/*.npz --out ~/data/scorecard11
echo "[v10] wide-floor variant:"
$PY fidelity_scorecard.py --real ~/data/real_npz/*.npz ~/data/real_npz_wide/*.npz \
  --sim $OUT/*.npz --out ~/data/scorecard11_wide || true
echo V10_CAMPAIGN_DONE
