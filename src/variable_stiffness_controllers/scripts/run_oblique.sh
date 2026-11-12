#!/bin/bash
# 实验 1-3 延伸: 45° 斜向碰撞 — M4 在 XY(45°) 与 XZ(45°) 两个斜方向各跑一次 (25N 末端力)
WORKSPACE=/home/sun/LiuCodeSun
OUT=/tmp/adls_oneday/exp_multidir       # 与单轴实验同目录, 便于合并热图
mkdir -p "$OUT"

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | grep -v conda | paste -sd:)
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
source ${WORKSPACE}/devel/setup.bash
echo "[env] roslaunch=$(which roslaunch)  python3=$(which python3)"

run_one() {
    local CSV=$1 DIR=$2 DIR2=$3 NAME=$4
    [[ -f "$CSV" ]] && { echo "  [skip] $CSV"; return; }
    echo "  → M4 force@${NAME} (dir=${DIR},dir2=${DIR2}) 25N@45°  $(date +%T)"
    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=0.05 force_magnitude:=25 \
        force_direction:=${DIR} force_direction2:=${DIR2} \
        force_start_time:=8.0 force_end_time:=10.0 \
        traj_mode:=circle method:=M4 lambda2_max:=0.1 \
        headless:=true > /tmp/roslaunch_${NAME}.log 2>&1 &
    local PID=$!
    sleep 6
    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:="${CSV}" _duration:=16 __name:=adls_diag_recorder
    kill $PID 2>/dev/null
    pkill -f "adls_experiment.launch" 2>/dev/null
    pkill -f "gzserver" 2>/dev/null
    pkill -f "gzclient" 2>/dev/null
    pkill -f "rosmaster" 2>/dev/null
    sleep 4
    echo "    done@${NAME}: $([[ -f $CSV ]] && wc -l < $CSV || echo MISSING) lines  $(date +%T)"
}

run_one "${OUT}/m4_XY.csv" 0 1 XY   # 45° in XY plane
run_one "${OUT}/m4_XZ.csv" 0 2 XZ   # 45° in XZ plane
echo "DONE_OBLIQUE"
