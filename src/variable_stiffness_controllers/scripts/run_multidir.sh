#!/bin/bash
# 实验 1-3: 多方向碰撞独立性 — M4 在 X/Y/Z 三个方向各跑一次 (25N 末端力)
WORKSPACE=/home/sun/LiuCodeSun
OUT=/tmp/adls_oneday/exp_multidir
mkdir -p "$OUT"

# 剥离 conda (否则 ROS spawn 失败)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | grep -v conda | paste -sd:)
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
source ${WORKSPACE}/devel/setup.bash
echo "[env] roslaunch=$(which roslaunch)  python3=$(which python3)"

run_one() {
    local CSV=$1 DIR=$2 NAME=$3
    [[ -f "$CSV" ]] && { echo "  [skip] $CSV"; return; }
    echo "  → M4 force@${NAME} (dir=${DIR}) 25N  $(date +%T)"
    roslaunch variable_stiffness_controllers adls_experiment.launch \
        circle_radius:=0.05 force_magnitude:=25 force_direction:=${DIR} \
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

run_one "${OUT}/m4_X.csv" 0 X
run_one "${OUT}/m4_Y.csv" 1 Y
run_one "${OUT}/m4_Z.csv" 2 Z
echo "DONE_MULTIDIR"
