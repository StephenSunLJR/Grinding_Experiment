#!/bin/bash
# 实验 3B: 奇异位形测试 — 一键运行 + 可视化
# ==========================================
# M1 vs M4 在向完全伸展位形过渡时的对比

set -e

WORKSPACE=${WORKSPACE:-/home/sun/LiuCodeSun}
OUTPUT_DIR=${OUTPUT_DIR:-/tmp/adls_exp3b}
DURATION=${DURATION:-10.0}

mkdir -p "${OUTPUT_DIR}"
echo "==== Exp 3B: Singularity Boundary ===="

source ${WORKSPACE}/devel/setup.bash

for METHOD in M1 M4; do
    CSV="${OUTPUT_DIR}/exp3b_${METHOD}.csv"
    echo "[run] Method = ${METHOD} → ${CSV}"

    # 无外力 + 静态保持（避免触发 A-DLS damping）
    # 通过设置极小 radius 让控制器进入静态模式
    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=0.0 \
        force_magnitude:=0.0 \
        method:=${METHOD} \
        headless:=true &
    LAUNCH_PID=$!

    sleep 5

    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:="${CSV}" _duration:=${DURATION} \
        __name:=adls_diag_recorder_${METHOD}

    kill $LAUNCH_PID 2>/dev/null || true
    pkill -f "adls_experiment.launch" 2>/dev/null || true
    sleep 2
    echo "[done] ${METHOD}"
done

echo "==== Generating figures ===="
python3 ${WORKSPACE}/src/variable_stiffness_controllers/scripts/analyze_exp3b.py \
    --m1 "${OUTPUT_DIR}/exp3b_M1.csv" \
    --m4 "${OUTPUT_DIR}/exp3b_M4.csv" \
    --output "${OUTPUT_DIR}/figs/"

echo "==== Exp 3B complete ===="
ls "${OUTPUT_DIR}/figs/"
