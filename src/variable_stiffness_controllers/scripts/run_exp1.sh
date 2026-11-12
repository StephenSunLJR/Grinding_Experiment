#!/bin/bash
# 实验一: 方向选择性验证 — 一键运行 + 可视化
# ================================================
# 依次运行 M1/M2/M3/M4 各一次（共 4 次 8s 仿真），记录诊断数据，
# 然后调用 analyze_exp1.py 生成图表。

set -e

# 配置
WORKSPACE=${WORKSPACE:-/home/sun/LiuCodeSun}
OUTPUT_DIR=${OUTPUT_DIR:-/tmp/adls_exp1}
DURATION=${DURATION:-9.0}             # 仿真时长
FORCE=${FORCE:-25.0}                  # 碰撞力 [N]
DIRECTION=${DIRECTION:-2}             # 0=X, 1=Y, 2=Z
RADIUS=${RADIUS:-0.05}                # 圆轨迹半径

mkdir -p "${OUTPUT_DIR}"
echo "==== Exp 1: Directional Selectivity ===="
echo "Output dir: ${OUTPUT_DIR}"
echo ""

source ${WORKSPACE}/devel/setup.bash

# 依次跑四个方法
for METHOD in M1 M2 M3 M4; do
    CSV="${OUTPUT_DIR}/exp1_${METHOD}.csv"
    echo "[run] Method = ${METHOD} → ${CSV}"

    # 启动 launch (后台)
    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=${RADIUS} \
        force_magnitude:=${FORCE} \
        force_direction:=${DIRECTION} \
        method:=${METHOD} \
        headless:=true &
    LAUNCH_PID=$!

    # 等待 controller 起来
    sleep 5

    # 启动 recorder (前台, 自动结束)
    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:="${CSV}" _duration:=${DURATION} \
        __name:=adls_diag_recorder_${METHOD}

    # 清理 launch
    kill $LAUNCH_PID 2>/dev/null || true
    pkill -f "adls_experiment.launch" 2>/dev/null || true
    sleep 2

    echo "[done] ${METHOD}"
    echo ""
done

# 调用 Python 分析脚本
echo "==== Generating figures ===="
python3 ${WORKSPACE}/src/variable_stiffness_controllers/scripts/analyze_exp1.py \
    --m1 "${OUTPUT_DIR}/exp1_M1.csv" \
    --m2 "${OUTPUT_DIR}/exp1_M2.csv" \
    --m3 "${OUTPUT_DIR}/exp1_M3.csv" \
    --m4 "${OUTPUT_DIR}/exp1_M4.csv" \
    --output "${OUTPUT_DIR}/figs/" \
    --collision-start 3.0 \
    --collision-end 5.0

echo ""
echo "==== Exp 1 complete ===="
echo "Figures: ${OUTPUT_DIR}/figs/"
ls "${OUTPUT_DIR}/figs/"
