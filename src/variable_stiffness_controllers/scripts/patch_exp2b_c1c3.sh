#!/bin/bash
# Patch: fill in missing C1 and C3 for Experiment 2B
# Run from your terminal:  ./src/variable_stiffness_controllers/scripts/patch_exp2b_c1c3.sh

set -e
cd ~/LiuCodeSun
source devel/setup.bash

DIR=/tmp/adls_oneday/exp2b
mkdir -p $DIR

run() {
    local CID=$1 EMA=$2 REF=$3
    local CSV=$DIR/exp2b_${CID}.csv
    [[ -f $CSV ]] && { echo "[skip] $CID"; return; }

    echo "[run] $CID ema=$EMA ref=$REF"
    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=0.05 force_magnitude:=30 force_direction:=2 \
        force_start_time:=3.0 force_end_time:=5.0 method:=M4 \
        ema_mode:=$EMA ref_modulation:=$REF headless:=true &
    PID=$!

    echo "  waiting for rosmaster..."
    sleep 3
    echo "  recording 9s (waiting for controller)..."
    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:=$CSV _duration:=9.0 __name:=diag_patch_${CID}

    echo "  shutting down..."
    kill $PID 2>/dev/null
    sleep 2
    while pgrep -x gzserver >/dev/null 2>&1; do sleep 1; done
    sleep 3
    echo "  [done] $CID"
}

run C1 none      false
run C3 asymmetric false

echo ""
echo "=== Re-generating Exp 2B figures ==="
python3 src/variable_stiffness_controllers/scripts/analyze_exp2b.py \
    --c1 $DIR/exp2b_C1.csv --c2 $DIR/exp2b_C2.csv \
    --c3 $DIR/exp2b_C3.csv --c4 $DIR/exp2b_C4.csv \
    --output $DIR/figs/ --collision-start 3.0 --collision-end 5.0

echo "Done. Figures in $DIR/figs/"
ls $DIR/figs/
