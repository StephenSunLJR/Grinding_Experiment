#!/bin/bash
# 实验 2B: 弹弓效应 + 非对称 EMA + 参考轨迹调制
# ================================================
# 4 配置 × ~10s 仿真 ≈ 4 分钟

set -e

WORKSPACE=${WORKSPACE:-/home/sun/LiuCodeSun}
OUTPUT_DIR=${OUTPUT_DIR:-/tmp/adls_oneday/exp2b}
DURATION=${DURATION:-9.0}

mkdir -p "${OUTPUT_DIR}"
echo "==== Exp 2B: Slingshot Mitigation Ablation ===="
echo "Output: ${OUTPUT_DIR}"

source ${WORKSPACE}/devel/setup.bash

# Force window: t∈[3,5]s, F=30N in -Z direction (per paper §2B.2)
COMMON_ARGS="circle_radius:=0.05 force_magnitude:=30 force_direction:=2 \
             force_start_time:=3.0 force_end_time:=5.0 method:=M4"

run_config() {
    local CID=$1 EMA=$2 REF=$3
    local CSV="${OUTPUT_DIR}/exp2b_${CID}.csv"
    [[ -f "$CSV" ]] && { echo "  [skip] $CSV exists"; return; }

    echo "[run] ${CID}: ema=${EMA}, ref_modulation=${REF}"

    roslaunch variable_stiffness_controllers adls_experiment.launch \
        ${COMMON_ARGS} \
        ema_mode:=${EMA} \
        ref_modulation:=${REF} \
        headless:=true \
        > /tmp/adls_launch_${CID}.log 2>&1 &
    LAUNCH_PID=$!

    # Wait for controller to be ready (check for diag topic)
    for i in $(seq 1 20); do
        rostopic list 2>/dev/null | grep -q "/adls/diagnostics" && break
        sleep 1
    done
    sleep 2

    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:="${CSV}" _duration:=${DURATION} \
        __name:=adls_diag_recorder_${CID} > /dev/null 2>&1

    # Graceful shutdown: roslaunch first, then gazebo
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 2
    # Kill any leftover processes from THIS launch only
    pkill -f "record_diagnostics.py.*${CID}" 2>/dev/null || true
    # Wait for gazebo to fully exit before next run
    while pgrep -f "gzserver" > /dev/null 2>&1; do sleep 1; done
    sleep 3
}

# 4 configurations (paper §2B.3)
run_config C1 none       false   # baseline: no EMA, no ref-mod
run_config C2 symmetric  false   # v5.9: symmetric EMA only
run_config C3 asymmetric false   # v5.10 step 1: asymmetric EMA only
run_config C4 asymmetric true    # v5.10 full: both protections

echo ""
echo "==== Generating Exp 2B figures ===="
python3 ${WORKSPACE}/src/variable_stiffness_controllers/scripts/analyze_exp2b.py \
    --c1 "${OUTPUT_DIR}/exp2b_C1.csv" \
    --c2 "${OUTPUT_DIR}/exp2b_C2.csv" \
    --c3 "${OUTPUT_DIR}/exp2b_C3.csv" \
    --c4 "${OUTPUT_DIR}/exp2b_C4.csv" \
    --output "${OUTPUT_DIR}/figs/" \
    --collision-start 3.0 --collision-end 5.0

echo ""
echo "==== Exp 2B complete ===="
ls "${OUTPUT_DIR}/figs/"
