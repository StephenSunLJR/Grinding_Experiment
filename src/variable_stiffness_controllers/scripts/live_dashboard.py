#!/usr/bin/env python3
"""
A-DLS 实时可视化仪表盘
=======================

订阅 /adls/diagnostics 话题，实时显示 6 个面板：
  ┌────────────────────────────┬────────────────────────────┐
  │  (0,0) XY 平面轨迹          │  (0,1) 笛卡尔误差 (3 轴)    │
  │   末端实际 vs 目标           │   X/Y/Z 随时间             │
  ├────────────────────────────┼────────────────────────────┤
  │  (1,0) A-DLS λ²_k (6 方向) │  (1,1) GMO 残差 r_corr (7 关节)│
  │   验证方向选择性             │   碰撞检测信号              │
  ├────────────────────────────┼────────────────────────────┤
  │  (2,0) 外力 ‖F_ext‖_∞       │  (2,1) 关节速度 ‖dq‖, ‖dq_cmd‖│
  │   + ‖r_corr‖_∞              │   速度跟踪健康度            │
  └────────────────────────────┴────────────────────────────┘

用法 (在 Gazebo 运行时另开一个终端):
    rosrun variable_stiffness_controllers live_dashboard.py
"""

import os
import sys
import collections
import threading
import numpy as np
import rospy
import matplotlib
matplotlib.use('TkAgg')  # 交互后端
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from std_msgs.msg import Float64MultiArray

# 字段布局（与 A_DLS.cpp 一致）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_utils import DIAG_FIELDS, AXIS_LABELS, METHOD_NAMES, METHOD_COLORS


# 配置
HISTORY_SEC = 10.0           # 滚动窗口长度 [s]
UPDATE_HZ   = 15             # 仪表盘刷新频率
INCOMING_HZ = 100            # /adls/diagnostics 发布频率（C++ 端）
BUFFER_LEN  = int(HISTORY_SEC * INCOMING_HZ)


class LiveDashboard:
    def __init__(self):
        rospy.init_node('adls_live_dashboard', anonymous=True)

        # 环形缓冲区（线程安全 deque）
        self.lock = threading.Lock()
        self.buf_t = collections.deque(maxlen=BUFFER_LEN)
        self.buf_error = collections.deque(maxlen=BUFFER_LEN)
        self.buf_lambda2 = collections.deque(maxlen=BUFFER_LEN)
        self.buf_r_corr = collections.deque(maxlen=BUFFER_LEN)
        self.buf_pos = collections.deque(maxlen=BUFFER_LEN)
        self.buf_pos_d = collections.deque(maxlen=BUFFER_LEN)
        self.buf_F_ext = collections.deque(maxlen=BUFFER_LEN)
        self.buf_dq = collections.deque(maxlen=BUFFER_LEN)
        self.buf_dq_cmd = collections.deque(maxlen=BUFFER_LEN)
        self.method_id = 4

        # 全程轨迹（不滚动，用于 XY 平面图）
        self.full_pos_x = collections.deque(maxlen=10000)
        self.full_pos_y = collections.deque(maxlen=10000)
        self.full_pos_d_x = collections.deque(maxlen=10000)
        self.full_pos_d_y = collections.deque(maxlen=10000)

        # ROS 订阅
        self.sub = rospy.Subscriber('/adls/diagnostics', Float64MultiArray,
                                    self.callback, queue_size=200)
        rospy.loginfo('[dashboard] Subscribed to /adls/diagnostics')

        # 创建画布
        self._setup_figure()

    def callback(self, msg: Float64MultiArray):
        d = np.asarray(msg.data)
        if d.size < 93:
            return

        with self.lock:
            self.buf_t.append(d[DIAG_FIELDS['t']].item())
            self.buf_error.append(d[DIAG_FIELDS['error']].copy())
            self.buf_lambda2.append(d[DIAG_FIELDS['lambda2']].copy())
            self.buf_r_corr.append(d[DIAG_FIELDS['r_corr']].copy())
            self.buf_pos.append(d[DIAG_FIELDS['position']].copy())
            self.buf_pos_d.append(d[DIAG_FIELDS['position_d']].copy())
            self.buf_F_ext.append(d[DIAG_FIELDS['F_ext_applied']].copy())
            self.buf_dq.append(d[DIAG_FIELDS['dq']].copy())
            self.buf_dq_cmd.append(d[DIAG_FIELDS['dq_cmd']].copy())
            self.method_id = int(d[DIAG_FIELDS['method']].item())

            self.full_pos_x.append(d[DIAG_FIELDS['position']][0])
            self.full_pos_y.append(d[DIAG_FIELDS['position']][1])
            self.full_pos_d_x.append(d[DIAG_FIELDS['position_d']][0])
            self.full_pos_d_y.append(d[DIAG_FIELDS['position_d']][1])

    def _setup_figure(self):
        plt.style.use('default')
        self.fig, self.axes = plt.subplots(3, 2, figsize=(13, 9))
        self.fig.suptitle('A-DLS Live Dashboard', fontsize=13, fontweight='bold')
        self.fig.canvas.manager.set_window_title('A-DLS Live')

        # (0,0) XY 平面轨迹
        ax = self.axes[0, 0]
        ax.set_title('XY Plane Trajectory (top view)')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        self.line_target_xy, = ax.plot([], [], '--', color='gray',
                                       alpha=0.6, label='Target', linewidth=1.0)
        self.line_actual_xy, = ax.plot([], [], '-', color='C0',
                                       label='Actual', linewidth=1.5)
        self.dot_current, = ax.plot([], [], 'o', color='red',
                                    markersize=7, label='EE now')
        self.dot_target, = ax.plot([], [], 's', color='green',
                                   markersize=7, label='Target now')
        ax.legend(loc='upper right', fontsize=8)

        # (0,1) Cartesian error
        ax = self.axes[0, 1]
        ax.set_title('Cartesian Position Error [mm]')
        ax.set_xlabel('Time [s]')
        ax.grid(True, alpha=0.3)
        self.line_err = []
        labels = ['X', 'Y', 'Z']
        colors = ['#1f77b4', '#2ca02c', '#d62728']
        for k in range(3):
            ln, = ax.plot([], [], label=labels[k], color=colors[k], linewidth=1.3)
            self.line_err.append(ln)
        ax.legend(loc='upper right', fontsize=9)

        # (1,0) λ²_k (6 directions)
        ax = self.axes[1, 0]
        ax.set_title('A-DLS Damping $\\lambda^2_k(t)$ (per direction)')
        ax.set_xlabel('Time [s]')
        ax.set_yscale('log')
        ax.grid(True, alpha=0.3)
        self.line_lam = []
        colors_l = plt.cm.tab10(np.arange(6))
        for k in range(6):
            ln, = ax.plot([], [], label=AXIS_LABELS[k],
                          color=colors_l[k], linewidth=1.2)
            self.line_lam.append(ln)
        ax.legend(loc='upper right', fontsize=8, ncol=2)

        # (1,1) GMO 残差 (7 关节)
        ax = self.axes[1, 1]
        ax.set_title('GMO Residual $r_{corr}$ (per joint) [N·m]')
        ax.set_xlabel('Time [s]')
        ax.grid(True, alpha=0.3)
        self.line_r = []
        colors_j = plt.cm.tab10(np.arange(7))
        for j in range(7):
            ln, = ax.plot([], [], label=f'J{j+1}',
                          color=colors_j[j], linewidth=1.0)
            self.line_r.append(ln)
        ax.legend(loc='upper right', fontsize=8, ncol=2)

        # (2,0) 外力 + r 范数
        ax = self.axes[2, 0]
        ax.set_title('External Force vs GMO Residual Norm')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('[N] / [N·m]')
        ax.grid(True, alpha=0.3)
        self.line_F_ext, = ax.plot([], [], label='$\\|F_{ext}\\|$ [N]',
                                    color='#ff7f0e', linewidth=1.5)
        self.line_r_norm, = ax.plot([], [], label='$\\|r_{corr}\\|_\\infty$ [N·m]',
                                     color='#9467bd', linewidth=1.5)
        ax.legend(loc='upper right', fontsize=9)

        # (2,1) 关节速度
        ax = self.axes[2, 1]
        ax.set_title('Joint Velocity Tracking Health')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('[rad/s]')
        ax.grid(True, alpha=0.3)
        self.line_dq, = ax.plot([], [], label='$\\|\\dot{q}\\|$',
                                color='C0', linewidth=1.5)
        self.line_dq_cmd, = ax.plot([], [], label='$\\|\\dot{q}_{cmd}\\|$',
                                    color='C3', linewidth=1.5, linestyle='--')
        ax.legend(loc='upper right', fontsize=9)

        # 状态文本（左上）
        self.text_status = self.fig.text(0.01, 0.98,
                                          'Waiting for data...',
                                          fontsize=10, verticalalignment='top',
                                          family='monospace',
                                          bbox=dict(facecolor='lightyellow',
                                                    edgecolor='gray', alpha=0.85))

        plt.tight_layout(rect=[0, 0, 1, 0.96])

    def update_plot(self, frame):
        with self.lock:
            if len(self.buf_t) < 2:
                return
            t = np.array(self.buf_t)
            error = np.array(self.buf_error)
            lambda2 = np.array(self.buf_lambda2)
            r_corr = np.array(self.buf_r_corr)
            pos = np.array(self.buf_pos)
            pos_d = np.array(self.buf_pos_d)
            F_ext = np.array(self.buf_F_ext)
            dq = np.array(self.buf_dq)
            dq_cmd = np.array(self.buf_dq_cmd)
            full_x = list(self.full_pos_x)
            full_y = list(self.full_pos_y)
            full_dx = list(self.full_pos_d_x)
            full_dy = list(self.full_pos_d_y)
            method_id = self.method_id

        # 滚动窗口
        t_now = t[-1]
        t_min = max(0.0, t_now - HISTORY_SEC)
        mask = t >= t_min
        t_win = t[mask]

        # (0,0) XY 轨迹（全程显示）
        self.line_target_xy.set_data(full_dx, full_dy)
        self.line_actual_xy.set_data(full_x, full_y)
        self.dot_current.set_data([pos[-1, 0]], [pos[-1, 1]])
        self.dot_target.set_data([pos_d[-1, 0]], [pos_d[-1, 1]])
        if full_x:
            xs = full_x + full_dx
            ys = full_y + full_dy
            x_pad = (max(xs) - min(xs)) * 0.15 + 0.005
            y_pad = (max(ys) - min(ys)) * 0.15 + 0.005
            self.axes[0, 0].set_xlim(min(xs) - x_pad, max(xs) + x_pad)
            self.axes[0, 0].set_ylim(min(ys) - y_pad, max(ys) + y_pad)

        # (0,1) Cartesian error
        for k in range(3):
            self.line_err[k].set_data(t_win, 1000.0 * error[mask, k])
        self._autoscale(self.axes[0, 1], t_win, 1000.0 * error[mask, :3], pad=0.15)

        # (1,0) λ²_k
        for k in range(6):
            self.line_lam[k].set_data(t_win, lambda2[mask, k])
        self.axes[1, 0].set_xlim(t_min, t_now)
        lam_min = max(1e-5, lambda2[mask].min())
        lam_max = max(1e-3, lambda2[mask].max())
        self.axes[1, 0].set_ylim(lam_min * 0.5, lam_max * 2.0)

        # (1,1) r_corr per joint
        for j in range(7):
            self.line_r[j].set_data(t_win, r_corr[mask, j])
        self._autoscale(self.axes[1, 1], t_win, r_corr[mask], pad=0.15)

        # (2,0) F_ext + ‖r‖
        F_norm = np.linalg.norm(F_ext[mask, :3], axis=1)
        r_norm = np.linalg.norm(r_corr[mask], ord=np.inf, axis=1)
        self.line_F_ext.set_data(t_win, F_norm)
        self.line_r_norm.set_data(t_win, r_norm)
        self._autoscale(self.axes[2, 0], t_win,
                        np.column_stack([F_norm, r_norm]), pad=0.15)

        # (2,1) ‖dq‖ vs ‖dq_cmd‖
        dq_norm = np.linalg.norm(dq[mask], axis=1)
        dq_cmd_norm = np.linalg.norm(dq_cmd[mask], axis=1)
        self.line_dq.set_data(t_win, dq_norm)
        self.line_dq_cmd.set_data(t_win, dq_cmd_norm)
        self._autoscale(self.axes[2, 1], t_win,
                        np.column_stack([dq_norm, dq_cmd_norm]), pad=0.15)

        # 状态文本
        pos_err_mm = 1000 * np.linalg.norm(error[-1, :3])
        method_name = METHOD_NAMES.get(method_id, f'M{method_id}')
        F_now = np.linalg.norm(F_ext[-1, :3])
        r_now = np.linalg.norm(r_corr[-1], ord=np.inf)
        lam_max_now = lambda2[-1].max()
        self.text_status.set_text(
            f't = {t_now:6.2f} s   |   {method_name}\n'
            f'pos_err  = {pos_err_mm:7.2f} mm\n'
            f'‖F_ext‖  = {F_now:6.2f} N\n'
            f'‖r_corr‖ = {r_now:6.2f} N·m\n'
            f'λ²_max   = {lam_max_now:.4f}\n'
            f'samples  = {len(t)}'
        )

    @staticmethod
    def _autoscale(ax, x, y_data, pad=0.1):
        if len(x) < 2:
            return
        ax.set_xlim(x[0], x[-1])
        y_min = y_data.min()
        y_max = y_data.max()
        if y_max - y_min < 1e-9:
            y_min -= 0.5
            y_max += 0.5
        span = y_max - y_min
        ax.set_ylim(y_min - pad * span, y_max + pad * span)

    def run(self):
        interval_ms = int(1000.0 / UPDATE_HZ)
        # save_count just controls the animation framework's internal cache;
        # cache_frame_data=False avoids growing memory for live plots.
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot,
            interval=interval_ms,
            cache_frame_data=False,
        )
        plt.show()


def main():
    try:
        d = LiveDashboard()
        d.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
