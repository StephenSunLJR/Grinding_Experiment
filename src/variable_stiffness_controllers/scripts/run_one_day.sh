#!/bin/bash
# 一天完成论文实验的主控脚本
# ===========================
# 顺序执行: 实验一 → 实验 3B → 实验 2A（精简版）→ 生成全部图表
# 总运行时间约 50-60 分钟，纯 Gazebo 仿真
#
# 用法:
#   ./run_one_day.sh             # 完整执行
#   ./run_one_day.sh --quick     # 5 分钟冒烟测试 (只做 M4 单次)
#   SKIP_EXP2A=1 ./run_one_day.sh  # 跳过 2A (省 25 分钟)

set -e

WORKSPACE=${WORKSPACE:-/home/sun/LiuCodeSun}
BASE_DIR=${BASE_DIR:-/tmp/adls_oneday}
SKIP_EXP2A=${SKIP_EXP2A:-0}
QUICK=${QUICK:-0}

[[ "$1" == "--quick" ]] && QUICK=1

source ${WORKSPACE}/devel/setup.bash

# ──────────────────────────────────────────────────────
# 工具函数: 跑单次实验 (启动+记录+清理)
# ──────────────────────────────────────────────────────
run_one() {
    local CSV=$1 METHOD=$2 RADIUS=$3 F=$4 DIR=$5 DUR=$6 LAM=${7:-0.5} TRAJ=${8:-circle} EXTD=${9:-0.5}

    [[ -f "$CSV" ]] && [[ $QUICK == 0 ]] && { echo "  [skip] $CSV exists"; return; }

    echo "  → $(basename $CSV)  M=$METHOD r=$RADIUS F=${F}N dir=$DIR λ²=$LAM traj=$TRAJ"

    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=${RADIUS} \
        force_magnitude:=${F} \
        force_direction:=${DIR} \
        force_start_time:=8.0 \
        force_end_time:=10.0 \
        traj_mode:=${TRAJ} \
        extend_distance:=${EXTD} \
        method:=${METHOD} \
        lambda2_max:=${LAM} \
        headless:=true \
        > /dev/null 2>&1 &
    local LAUNCH_PID=$!

    sleep 6   # Gazebo 启动等待

    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:="${CSV}" _duration:=${DUR} \
        __name:=adls_diag_recorder > /dev/null 2>&1

    kill $LAUNCH_PID 2>/dev/null || true
    pkill -f "adls_experiment.launch" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
    sleep 2
}

mkdir -p "${BASE_DIR}"
START_TIME=$(date +%s)

# ──────────────────────────────────────────────────────
# Phase 0: 冒烟测试 (5 min)
# ──────────────────────────────────────────────────────
if [[ $QUICK == 1 ]]; then
    echo ""
    echo "════════════════════════════════════════════════════"
    echo " QUICK MODE: smoke test only (M4, circle + force)"
    echo "════════════════════════════════════════════════════"
    mkdir -p "${BASE_DIR}/smoke"
    run_one "${BASE_DIR}/smoke/m4.csv" M4 0.05 25 2 16
    python3 ${WORKSPACE}/src/variable_stiffness_controllers/scripts/analyze_exp1.py \
        --m4 "${BASE_DIR}/smoke/m4.csv" \
        --output "${BASE_DIR}/smoke/figs/" \
        --collision-start 8.0 --collision-end 10.0
    echo ""
    echo "✓ Smoke test complete. Check ${BASE_DIR}/smoke/figs/"
    exit 0
fi

# ──────────────────────────────────────────────────────
# Phase 1: 实验一 — 4 方法方向选择性 (12 min)
# ──────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════"
echo " Phase 1: Experiment 1 — Directional Selectivity"
echo "════════════════════════════════════════════════════"
EXP1_DIR="${BASE_DIR}/exp1"
mkdir -p "${EXP1_DIR}"
for METHOD in M1 M2 M3 M4; do
    run_one "${EXP1_DIR}/exp1_${METHOD}.csv" ${METHOD} 0.05 25 2 16 0.1
done
echo ""
echo "[Phase 1 analysis]"
python3 ${WORKSPACE}/src/variable_stiffness_controllers/scripts/analyze_exp1.py \
    --m1 "${EXP1_DIR}/exp1_M1.csv" \
    --m2 "${EXP1_DIR}/exp1_M2.csv" \
    --m3 "${EXP1_DIR}/exp1_M3.csv" \
    --m4 "${EXP1_DIR}/exp1_M4.csv" \
    --output "${EXP1_DIR}/figs/" \
    --collision-start 8.0 --collision-end 10.0
echo "✓ Phase 1 figures: ${EXP1_DIR}/figs/"

# ──────────────────────────────────────────────────────
# Phase 2: 实验 3B — 奇异位形 (6 min)
# ──────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════"
echo " Phase 2: Experiment 3B — Singularity Boundary"
echo "════════════════════════════════════════════════════"
EXP3B_DIR="${BASE_DIR}/exp3b"
mkdir -p "${EXP3B_DIR}"
for METHOD in M1 M4; do
    # 无外力, extend 模式伸展至奇异位形 (M1 1/σ 发散 vs M4 DLS 有界)
    run_one "${EXP3B_DIR}/exp3b_${METHOD}.csv" ${METHOD} 0.0 0 2 12 0.1 extend 0.5
done
echo ""
echo "[Phase 2 analysis]"
python3 ${WORKSPACE}/src/variable_stiffness_controllers/scripts/analyze_exp3b.py \
    --m1 "${EXP3B_DIR}/exp3b_M1.csv" \
    --m4 "${EXP3B_DIR}/exp3b_M4.csv" \
    --output "${EXP3B_DIR}/figs/"
echo "✓ Phase 2 figures: ${EXP3B_DIR}/figs/"

# ──────────────────────────────────────────────────────
# Phase 3: 实验 2A — Pareto (精简版, 25 min)
# ──────────────────────────────────────────────────────
if [[ $SKIP_EXP2A == 0 ]]; then
    echo ""
    echo "════════════════════════════════════════════════════"
    echo " Phase 3: Experiment 2A — Pareto (reduced)"
    echo "════════════════════════════════════════════════════"
    EXP2A_DIR="${BASE_DIR}/exp2a"
    mkdir -p "${EXP2A_DIR}"

    # 精简: 仅 Z 方向, 5 个力幅值, 4 方法 = 20 次
    FORCE_LEVELS=(5 15 25 35 50)
    for METHOD in M1 M2 M3 M4; do
        for F in "${FORCE_LEVELS[@]}"; do
            CSV="${EXP2A_DIR}/m${METHOD#M}_F${F}_d2.csv"
            run_one "${CSV}" ${METHOD} 0.0 ${F} 2 16
        done
    done

    # M4 λ²_max 扫描: 4 个值 × 3 个力 = 12 次
    LAMBDA_LEVELS=(0.05 0.1 0.5 1.0)
    for LAM in "${LAMBDA_LEVELS[@]}"; do
        for F in 10 25 40; do
            CSV="${EXP2A_DIR}/m4_F${F}_lambda${LAM}_d2.csv"
            run_one "${CSV}" M4 0.0 ${F} 2 16 ${LAM}
        done
    done

    echo ""
    echo "[Phase 3 analysis]"
    python3 ${WORKSPACE}/src/variable_stiffness_controllers/scripts/analyze_exp2a.py \
        --csv-dir "${EXP2A_DIR}/" \
        --output "${EXP2A_DIR}/figs/" \
        --collision-start 8.0 --collision-end 10.0 \
        --target-F 25.0
    echo "✓ Phase 3 figures: ${EXP2A_DIR}/figs/"
fi

# ──────────────────────────────────────────────────────
# 完成总结
# ──────────────────────────────────────────────────────
END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))
echo ""
echo "════════════════════════════════════════════════════"
echo " ALL EXPERIMENTS COMPLETE"
echo "════════════════════════════════════════════════════"
echo "Total runtime: $((ELAPSED / 60)) min $((ELAPSED % 60)) sec"
echo ""
echo "Output directories:"
echo "  Exp 1   : ${BASE_DIR}/exp1/figs/"
echo "  Exp 3B  : ${BASE_DIR}/exp3b/figs/"
[[ $SKIP_EXP2A == 0 ]] && echo "  Exp 2A  : ${BASE_DIR}/exp2a/figs/"
echo ""
echo "Recommended next steps:"
echo "  1. Open each figs/ directory and check fig1*.pdf, fig2*.pdf, etc."
echo "  2. Inspect tab1_summary.md for quantitative metrics"
echo "  3. Re-run any experiment that looks abnormal:"
echo "       ./run_one_day.sh                  # full re-run"
echo "       BASE_DIR=/tmp/adls_oneday ./run_one_day.sh  # incremental"
