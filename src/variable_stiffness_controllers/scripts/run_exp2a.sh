#!/bin/bash
# 实验 2A: Pareto 前沿 — 扫参运行 + 可视化
# ============================================
# 扫描: 4 方法 × 力幅值 × (M4 额外扫 λ²_max)
# 每次试验 6s, 总共约 120 次试验 (~12 分钟)

set -e

WORKSPACE=${WORKSPACE:-/home/sun/LiuCodeSun}
OUTPUT_DIR=${OUTPUT_DIR:-/tmp/adls_exp2a}
DURATION=${DURATION:-6.0}

# 力幅值扫描
FORCE_LEVELS=(5 10 15 20 25 30 35 40 45 50)
DIRECTIONS=(0 1 2)        # 0=X, 1=Y, 2=Z
METHODS=(M1 M2 M3 M4)
# M4 额外扫描 λ²_max
LAMBDA_LEVELS=(0.01 0.05 0.1 0.2 0.5 1.0)

mkdir -p "${OUTPUT_DIR}"
echo "==== Exp 2A: Pareto Frontier ===="
echo "Output dir: ${OUTPUT_DIR}"
echo "Force levels: ${FORCE_LEVELS[@]}"
echo "Directions: ${DIRECTIONS[@]}"
echo ""

source ${WORKSPACE}/devel/setup.bash

run_one() {
    local CSV=$1 METHOD=$2 F=$3 DIR=$4 LAM=$5
    echo "  → ${CSV}"

    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=0.0 \
        force_magnitude:=${F} \
        force_direction:=${DIR} \
        force_start_time:=2.0 \
        force_end_time:=4.0 \
        method:=${METHOD} \
        lambda2_max:=${LAM:-0.5} \
        headless:=true &
    LAUNCH_PID=$!
    sleep 4
    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:="${CSV}" _duration:=${DURATION} \
        __name:=adls_diag_recorder
    kill $LAUNCH_PID 2>/dev/null || true
    pkill -f "adls_experiment.launch" 2>/dev/null || true
    sleep 2
}

# Phase 1: M1/M2/M3 + M4 default lambda
for METHOD in "${METHODS[@]}"; do
    for DIR in "${DIRECTIONS[@]}"; do
        for F in "${FORCE_LEVELS[@]}"; do
            CSV="${OUTPUT_DIR}/m${METHOD#M}_F${F}_d${DIR}.csv"
            [[ -f $CSV ]] && { echo "  [skip] ${CSV}"; continue; }
            run_one "${CSV}" ${METHOD} ${F} ${DIR}
        done
    done
done

# Phase 2: M4 lambda sweep (only Z direction, fewer force levels)
for LAM in "${LAMBDA_LEVELS[@]}"; do
    for F in 10 25 40; do
        CSV="${OUTPUT_DIR}/m4_F${F}_lambda${LAM}_d2.csv"
        [[ -f $CSV ]] && { echo "  [skip] ${CSV}"; continue; }
        run_one "${CSV}" M4 ${F} 2 ${LAM}
    done
done

# 分析
echo ""
echo "==== Generating Pareto figures ===="
python3 ${WORKSPACE}/src/variable_stiffness_controllers/scripts/analyze_exp2a.py \
    --csv-dir "${OUTPUT_DIR}/" \
    --output "${OUTPUT_DIR}/figs/" \
    --collision-start 2.0 \
    --collision-end 4.0 \
    --target-F 25.0

echo "==== Exp 2A complete ===="
ls "${OUTPUT_DIR}/figs/"
