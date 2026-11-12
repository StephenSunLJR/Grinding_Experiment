#!/bin/bash
# Run a single Exp1 method with reliable timing.
# Usage:
#   ./run_exp1_single.sh M3
#   ./run_exp1_single.sh M4
#
# Key fix: start recorder with sleep 3 (rosmaster is up) so it naturally
# waits for /adls/diagnostics and captures from controller t≈0.

set -e
METHOD=${1:?Usage: $0 M3|M4}
DIR=/tmp/adls_oneday/exp1
mkdir -p $DIR

CSV=$DIR/exp1_${METHOD}.csv
if [[ -f "$CSV" ]]; then
    echo "[skip] $CSV already exists"
    exit 0
fi

cd ~/LiuCodeSun
source devel/setup.bash

echo "=== Exp1 $METHOD: launching Gazebo + controller ==="
roslaunch variable_stiffness_controllers adls_experiment.launch \
    circle_radius:=0.05 force_magnitude:=25 force_direction:=2 \
    method:=$METHOD headless:=true \
    > /tmp/adls_launch_${METHOD}.log 2>&1 &
LAUNCH_PID=$!

# Sleep just long enough for rosmaster to be ready.
# The recorder itself waits for /adls/diagnostics to appear — no need for
# a long sleep here.
sleep 3

echo "=== Exp1 $METHOD: recording (waiting for controller to publish) ==="
rosrun variable_stiffness_controllers record_diagnostics.py \
    _output:=$CSV _duration:=9.0 __name:=adls_recorder_${METHOD}

echo "=== Exp1 $METHOD: shutting down ==="
kill $LAUNCH_PID 2>/dev/null || true
sleep 2
while pgrep -x gzserver >/dev/null 2>&1; do sleep 1; done
sleep 2

echo "=== Done: $CSV ==="
wc -l $CSV
