# 名义模型离线扫描（2026-11-11）

输入为 `bidirectional_20261111_163217_049766_run2.csv` 的第一条实际关节位置和速度。使用项目中 `panda.urdf.xacro` 的 Gazebo 识别惯量，生成 `hand:=false gazebo:=true parent:=` 的 URDF，以 `panda_link8` 作为末端、不附加负载。该惯量文件明确不是 Franka 官方惯量；这次 CSV 未记录实际的 `F_T_EE`、质量、质心和惯量，也没有记录 Franka 内置模型的质量矩阵或雅可比。故 `approx_mass.csv` 和 `approx_jacobian.csv` 是近似模型计算结果，不是设备实测值。

扫描在启动关节姿态附近，使用当前配置的任务刚度 `[100,100,100,40,40,40]` 和阻尼 `[10,10,10,4,4,4]`，各自整体乘 `[0.25,0.5,1,2,4]`，步长为 `[0.00025,0.0005,0.001,0.002,0.004]` 秒，共 125 组。假定零外力、alpha=0、目标保持启动位姿；每组计算冻结启动模型的离散线性谱，并独立运行最多 0.2 秒非线性近似名义动力学。速度绝对值超过 50 rad/s 时停止仿真，仅作为数值发散指标。

零空间方向线性化特征值为 1，谱指标排除距离 1 小于 1e-6 的中性方向，其余方向最大模超过 1+1e-6 判为线性不稳定。0.2 秒不发散或线性稳定均不等同于机器人闭环安全。

| 其他条件 | 调整量 | 离散谱（非中性） | 非线性仿真 |
|---|---|---:|---|
| K=1、D=1 | dt=1 ms | 0.999536 | 0.2 秒速度未超 50 rad/s |
| K=1、dt=1 ms | D=4 倍 | 2.377382 | 14 ms 后速度超 50 rad/s |
| K=1、D=1 | dt=4 ms | 2.4674 | 52 ms 后速度超 50 rad/s |
| D=1、dt=1 ms | K=0.25–4 倍 | 约 0.999536 | 0.2 秒均未超 50 rad/s |

`scan.csv` 给出 125 组完整结果；`baseline_trace.csv` 给出基准仿真逐周期名义速度。基准仿真没有复现实际日志中的第七关节名义速度交替放大：实际前 6 个有效样本为 -0.00594、0.02239、-0.13421、0.73224、-4.05964、22.48800 rad/s；近似仿真对应值约 -0.00136、-0.00145、-0.00147、-0.00148、-0.00149、-0.00150 rad/s。真实采样、负载、初始状态、内部动力学模型与该近似都可能不同，因此不能从这些对比推断实机某个参数的安全范围。

复现：

```bash
source /opt/ros/noetic/setup.bash
source /home/franka/stephensun/devel_diagnostics/setup.bash
cd /home/franka/stephensun
/usr/bin/python3 analysis/nominal_dynamics/scan_nominal.py --csv /home/franka/.ros/bidirectional_logs/bidirectional_20261111_163217_049766_run2.csv
```
