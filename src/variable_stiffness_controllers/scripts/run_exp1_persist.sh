#!/bin/bash
# 实验一: 方向选择性 — M1/M2/M3/M4, 末端 Z 向 25N 碰撞 (确定性配置: 力 8-10s, 录 16s)
# 数据与图直接落到持久文件夹 (不再用 /tmp, 防跨天清空)
WORKSPACE=/home/sun/LiuCodeSun
OUT="/home/sun/毕设/中期/实验结果/exp1"
SCR="${WORKSPACE}/src/variable_stiffness_controllers/scripts"
mkdir -p "$OUT/figs"

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | grep -v conda | paste -sd:)
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
source ${WORKSPACE}/devel/setup.bash
echo "[env] roslaunch=$(which roslaunch)  python3=$(which python3)"

run_one() {
    local CSV=$1 METHOD=$2
    [[ -f "$CSV" ]] && { echo "  [skip] $CSV"; return; }
    echo "  → ${METHOD}  Z向25N  $(date +%T)"
    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=0.05 force_magnitude:=25 force_direction:=2 \
        force_start_time:=8.0 force_end_time:=10.0 \
        traj_mode:=circle method:=${METHOD} lambda2_max:=0.1 \
        headless:=true > /tmp/roslaunch_${METHOD}.log 2>&1 &
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

for M in M1 M2 M3 M4; do
    run_one "${OUT}/exp1_${M}.csv" ${M}
done

echo ""; echo "[出图] 四方法时域 + λ²演化 + η + tab1"
python3 ${SCR}/analyze_exp1.py \
    --m1 "${OUT}/exp1_M1.csv" --m2 "${OUT}/exp1_M2.csv" \
    --m3 "${OUT}/exp1_M3.csv" --m4 "${OUT}/exp1_M4.csv" \
    --output "${OUT}/figs/" --collision-start 8.0 --collision-end 10.0

echo "[出图] 核心论点图 fig_core_M2vsM4"
python3 ${SCR}/plot_core.py \
    --m2 "${OUT}/exp1_M2.csv" --m4 "${OUT}/exp1_M4.csv" \
    --output "${OUT}/figs/"

echo ""; echo "DONE_EXP1 → ${OUT}/figs/"
