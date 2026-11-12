#!/bin/bash
# 任务分级 + 零空间各向异性柔顺 — M3 vs M4, 开次级任务(secondary_task), 末端 Z 向 25N
# 数据与图落持久目录 (1.3 节融合控制律佐证)
WORKSPACE=/home/sun/LiuCodeSun
OUT="/home/sun/毕设/中期/实验结果/exp_hier"
SCR="${WORKSPACE}/src/variable_stiffness_controllers/scripts"
mkdir -p "$OUT/figs"

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | grep -v conda | paste -sd:)
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
source ${WORKSPACE}/devel/setup.bash
echo "[env] roslaunch=$(which roslaunch)  python3=$(which python3)"

run_one() {
    local CSV=$1 METHOD=$2
    [[ -f "$CSV" ]] && { echo "  [skip] $CSV"; return; }
    echo "  → ${METHOD} (secondary_task=true, 静止保持) Z向25N  $(date +%T)"
    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=0.0 force_magnitude:=25 force_direction:=2 \
        force_start_time:=8.0 force_end_time:=10.0 \
        traj_mode:=circle method:=${METHOD} lambda2_max:=0.1 \
        secondary_task:=true \
        headless:=true > /tmp/roslaunch_hier_${METHOD}.log 2>&1 &
    local PID=$!
    sleep 6
    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:="${CSV}" _duration:=16 __name:=adls_diag_recorder
    kill $PID 2>/dev/null
    pkill -f "adls_experiment.launch" 2>/dev/null
    pkill -f "gzserver" 2>/dev/null; pkill -f "gzclient" 2>/dev/null
    pkill -f "rosmaster" 2>/dev/null
    sleep 4
    echo "    done ${METHOD}: $([[ -f $CSV ]] && wc -l < $CSV || echo MISSING) lines"
}

# 配置已从圆轨迹改为静止保持, 删除旧 CSV 以强制重跑
rm -f "${OUT}/hier_M3.csv" "${OUT}/hier_M4.csv"

run_one "${OUT}/hier_M3.csv" M3
run_one "${OUT}/hier_M4.csv" M4

echo ""; echo "[出图] fig_hierarchy"
python3 ${SCR}/plot_hierarchy.py \
    --m3 "${OUT}/hier_M3.csv" --m4 "${OUT}/hier_M4.csv" \
    --output "${OUT}/figs/" --cs 8.0 --ce 10.0

echo ""; echo "DONE_HIER → ${OUT}/figs/"
