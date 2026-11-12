#!/bin/bash
# Patch: re-run all M4 data for Exp2A (needed after dual-deadzone fix).
# Run from ~/LiuCodeSun:  ./src/variable_stiffness_controllers/scripts/patch_exp2a_m4.sh

set -e
cd ~/LiuCodeSun
source devel/setup.bash

DIR=/tmp/adls_oneday/exp2a
mkdir -p $DIR

run_m4() {
    local F=$1
    local CSV=$DIR/m4_F${F}_d2.csv
    [[ -f $CSV ]] && { echo "[skip] $CSV"; return; }

    echo "[run] M4 F=${F}N"
    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=0.0 \
        force_magnitude:=${F} \
        force_direction:=2 \
        force_start_time:=3.0 \
        force_end_time:=5.0 \
        method:=M4 \
        headless:=true \
        > /tmp/adls_exp2a_m4_F${F}.log 2>&1 &
    LAUNCH_PID=$!

    sleep 3   # let rosmaster start; recorder waits for topic naturally
    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:=$CSV _duration:=6.0 __name:=adls_rec_m4_F${F}

    echo "  shutting down F=${F}N..."
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 2
    while pgrep -x gzserver >/dev/null 2>&1; do sleep 1; done
    sleep 2
    echo "  [done] $CSV ($(wc -l < $CSV) rows)"
}

# 5 force levels matching run_one_day.sh
for F in 5 15 25 35 50; do
    run_m4 $F
done

echo ""
echo "=== Re-generating Exp2A figures ==="
python3 src/variable_stiffness_controllers/scripts/analyze_exp2a.py \
    --csv-dir $DIR/ \
    --output $DIR/figs/ \
    --collision-start 3.0 --collision-end 5.0 \
    --target-F 25.0

echo ""
echo "Done. Figures in $DIR/figs/"
ls $DIR/figs/ 2>/dev/null
