#!/bin/bash
# 实验 3B v2: 奇异位形测试 (修复版)
# ==================================
# 使用 traj_mode:=extend 让机械臂真正进入奇异附近
# M1 vs M4, 10s 仿真

set -e

WORKSPACE=${WORKSPACE:-/home/sun/LiuCodeSun}
OUTPUT_DIR=${OUTPUT_DIR:-/tmp/adls_oneday/exp3b}
DURATION=${DURATION:-10.0}

mkdir -p "${OUTPUT_DIR}"
echo "==== Exp 3B v2: Singularity Boundary (with extend trajectory) ===="

source ${WORKSPACE}/devel/setup.bash

run_one() {
    local METHOD=$1
    local CSV="${OUTPUT_DIR}/exp3b_${METHOD}_extend.csv"
    [[ -f "$CSV" ]] && { echo "  [skip] $CSV exists"; return; }

    echo "[run] ${METHOD} with traj_mode=extend"

    roslaunch variable_stiffness_controllers adls_experiment.launch \
        traj_mode:=extend \
        extend_distance:=0.30 \
        circle_radius:=0.0 \
        force_magnitude:=0.0 \
        method:=${METHOD} \
        headless:=true \
        > /dev/null 2>&1 &
    LAUNCH_PID=$!

    sleep 6
    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:="${CSV}" _duration:=${DURATION} \
        __name:=adls_diag_recorder > /dev/null 2>&1

    kill $LAUNCH_PID 2>/dev/null || true
    pkill -f "adls_experiment.launch" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    sleep 2
}

run_one M1
run_one M4

echo ""
echo "==== Generating Exp 3B figures ===="
python3 ${WORKSPACE}/src/variable_stiffness_controllers/scripts/analyze_exp3b.py \
    --m1 "${OUTPUT_DIR}/exp3b_M1_extend.csv" \
    --m4 "${OUTPUT_DIR}/exp3b_M4_extend.csv" \
    --output "${OUTPUT_DIR}/figs_v2/"

echo "==== Exp 3B v2 complete ===="
ls "${OUTPUT_DIR}/figs_v2/"
