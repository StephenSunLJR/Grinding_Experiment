#!/usr/bin/env python3
"""
实验一: 方向选择性验证
======================

验证 A-DLS 是否真正实现了"仅在与碰撞力重合的任务方向上松弛控制，
在正交方向上维持全精度"。

输入: 4 个方法 (M1/M2/M3/M4) 在相同碰撞场景下的诊断 CSV
输出:
    Fig 1: 4 方法时域对比 (4 行 × 3 列: tilde_x_X, tilde_x_Z, ‖r‖)
    Fig 2: M4 的 6 条 λ²_k(t) 曲线叠加
    Fig 3: 方向选择性指数 η 柱状图
    Tab 1: 6 指标 × 4 方法汇总表

用法:
    python3 analyze_exp1.py \
        --m1 /tmp/exp1_M1.csv --m2 /tmp/exp1_M2.csv \
        --m3 /tmp/exp1_M3.csv --m4 /tmp/exp1_M4.csv \
        --output /tmp/exp1_figs/
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')   # must be set BEFORE importing pyplot or plot_utils
import matplotlib.pyplot as plt

# Allow direct execution
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_utils import (
    load_csv, extract, setup_plot_style, shade_collision_region,
    save_figure, compute_rmse, compute_eta, compute_force_metrics,
    METHOD_SHORTS, METHOD_COLORS
)


def figure_1_time_domain(datas: dict, t_coll: tuple, output_dir: str):
    """Fig 1: 4 方法时域对比 (4 行 × 3 列)。
       行 = {M1, M2, M3, M4}
       列 = {tilde_x_X, tilde_x_Z, ‖r‖_inf}
    """
    fig, axes = plt.subplots(4, 3, figsize=(13, 11), sharex=True)
    fig.suptitle('Fig 1 — Four-Method Time-Domain Comparison (Directional Selectivity)',
                 fontsize=13, fontweight='bold', y=1.0)

    col_titles = ['$\\tilde{x}_X$ (X-direction error, healthy)',
                  '$\\tilde{x}_Z$ (Z-direction error, collision)',
                  '$\\|r(t)\\|_\\infty$ (external torque, JTS)']

    for row, method in enumerate(['M1', 'M2', 'M3', 'M4']):
        data = datas.get(method)
        if data is None:
            for col in range(3):
                axes[row, col].text(0.5, 0.5, f'{method}: no data',
                                    ha='center', va='center',
                                    transform=axes[row, col].transAxes)
            continue

        t = extract(data, 't')
        error = extract(data, 'error')      # (N, 6)
        r = extract(data, 'r_corr')         # (N, 7)

        color = METHOD_COLORS[int(METHOD_SHORTS_INV[method])]

        # Column 0: tilde_x_X (健康方向)
        ax = axes[row, 0]
        ax.plot(t, 1000.0 * error[:, 0], color=color, linewidth=1.2)
        ax.set_ylabel(f'{method}\n[mm]')
        shade_collision_region(ax, *t_coll)
        if row == 0:
            ax.set_title(col_titles[0])

        # Column 1: tilde_x_Z (碰撞方向)
        ax = axes[row, 1]
        ax.plot(t, 1000.0 * error[:, 2], color=color, linewidth=1.2)
        shade_collision_region(ax, *t_coll)
        if row == 0:
            ax.set_title(col_titles[1])

        # Column 2: ‖r‖_inf
        ax = axes[row, 2]
        r_norm = np.linalg.norm(r, ord=np.inf, axis=1)
        ax.plot(t, r_norm, color=color, linewidth=1.2)
        ax.set_ylabel('[N·m]')
        shade_collision_region(ax, *t_coll)
        if row == 0:
            ax.set_title(col_titles[2])

    for col in range(3):
        axes[-1, col].set_xlabel('Time [s]')

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    save_figure(fig, output_dir, 'fig1_time_domain')
    plt.close(fig)


def figure_2_lambda_evolution(data_m4: np.ndarray, t_coll: tuple,
                              output_dir: str):
    """Fig 2: A-DLS 的 6 条 λ²_i(t) 曲线叠加 (v6.0 笛卡尔基阻尼)。
       关键验证: Z 向碰撞下应只有 λ²_Z 上升, 健康轴 λ²_X 维持 base。
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    t = extract(data_m4, 't')
    lambda2 = extract(data_m4, 'lambda2')  # (N, 6) — 笛卡尔各轴 λ²

    # v6.0: 阻尼定义在笛卡尔坐标, 每条曲线对应一个物理任务轴
    cart = ['X', 'Y', 'Z', 'R_X', 'R_Y', 'R_Z']
    colors = plt.cm.tab10(np.arange(6))
    for k in range(6):
        ax.plot(t, lambda2[:, k], color=colors[k],
                label=f'$\\lambda^2_{{{cart[k]}}}$ ({cart[k]} axis)',
                linewidth=1.5)

    shade_collision_region(ax, *t_coll)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('$\\lambda^2_i(t)$')
    ax.set_title('Fig 2 — A-DLS Per-Cartesian-Axis Damping $\\lambda^2_i(t)$ Evolution\n'
                 '(M4, collision applied at TCP in world Z. Each curve = one Cartesian task axis; '
                 'only $\\lambda^2_Z$ should rise.)',
                 fontsize=11)
    ax.legend(loc='upper right', ncol=2)
    ax.set_yscale('log')

    plt.tight_layout()
    save_figure(fig, output_dir, 'fig2_lambda_evolution')
    plt.close(fig)


def figure_3_eta_bar(datas: dict, t_coll: tuple, output_dir: str):
    """Fig 3: 方向选择性指数 η 柱状图。
       η << 1 = A-DLS 各向异性优势
       η ≈ 1 = 各向同性退化
    """
    fig, ax = plt.subplots(figsize=(8, 5.5))
    methods = ['M1', 'M2', 'M3', 'M4']
    etas = []
    for m in methods:
        data = datas.get(m)
        if data is None:
            etas.append(np.nan)
            continue
        t = extract(data, 't')
        error = extract(data, 'error')
        rmse = compute_rmse(error[:, :3], t, *t_coll)
        eta = compute_eta(rmse, collision_axis=2)
        etas.append(eta)

    colors = [METHOD_COLORS[int(METHOD_SHORTS_INV[m])] for m in methods]
    bars = ax.bar(methods, etas, color=colors, edgecolor='black', linewidth=1.0)

    for bar, eta in zip(bars, etas):
        if not np.isnan(eta):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height(),
                    f'{eta:.3f}',
                    ha='center', va='bottom', fontsize=11)

    ax.axhline(y=1.0, color='red', linestyle='--', linewidth=1,
               label='$\\eta = 1$ (isotropic baseline)')
    ax.axhline(y=0.3, color='green', linestyle=':', linewidth=1,
               label='$\\eta < 0.3$ (A-DLS target)')

    ax.set_xlabel('Control Method')
    ax.set_ylabel('Directional Selectivity Index $\\eta$')
    ax.set_title('Fig 3 — Directional Selectivity (lower = better)\n'
                 r'$\eta = \frac{1}{2}(\mathrm{RMSE}_X + \mathrm{RMSE}_Y) / \mathrm{RMSE}_Z$',
                 fontsize=12)
    ax.legend(loc='upper right')

    plt.tight_layout()
    save_figure(fig, output_dir, 'fig3_eta_bar')
    plt.close(fig)


def table_1_summary(datas: dict, t_pre: tuple, t_coll: tuple, output_dir: str):
    """Tab 1: 6 指标 × 4 方法汇总表。"""
    methods = ['M1', 'M2', 'M3', 'M4']
    rows = []

    for m in methods:
        data = datas.get(m)
        if data is None:
            rows.append([m] + ['N/A'] * 6)
            continue

        t = extract(data, 't')
        error = extract(data, 'error')
        r = extract(data, 'r_corr')
        lambda2 = extract(data, 'lambda2')

        # 碰撞区间 RMSE per axis
        rmse = compute_rmse(error[:, :3], t, *t_coll)
        rmse_x_mm = 1000 * rmse[0]
        rmse_y_mm = 1000 * rmse[1]
        rmse_z_mm = 1000 * rmse[2]

        # 方向选择性
        eta = compute_eta(rmse, collision_axis=2)

        # 力矩指标
        metrics = compute_force_metrics(r, np.zeros((len(t), 6)), t, *t_coll)
        r_peak = metrics['r_peak']

        # 平均 λ²_Z (碰撞方向)
        mask = (t >= t_coll[0]) & (t <= t_coll[1])
        lambda2_z_avg = lambda2[mask, 2].mean() if mask.sum() else 0.0

        rows.append([
            m,
            f'{rmse_x_mm:.2f}',
            f'{rmse_y_mm:.2f}',
            f'{rmse_z_mm:.2f}',
            f'{eta:.3f}',
            f'{r_peak:.2f}',
            f'{lambda2_z_avg:.4f}',
        ])

    # 输出 markdown 表
    header = ['Method', 'RMSE_X [mm]', 'RMSE_Y [mm]', 'RMSE_Z [mm]',
              'η', 'r_peak [N·m]', 'λ²_Z avg']
    out_path = os.path.join(output_dir, 'tab1_summary.md')
    os.makedirs(output_dir, exist_ok=True)
    with open(out_path, 'w') as f:
        f.write('# Table 1 — Experiment 1 Summary\n\n')
        f.write('| ' + ' | '.join(header) + ' |\n')
        f.write('|' + '|'.join(['---'] * len(header)) + '|\n')
        for r in rows:
            f.write('| ' + ' | '.join(r) + ' |\n')

    # 同时输出 CSV
    csv_path = os.path.join(output_dir, 'tab1_summary.csv')
    with open(csv_path, 'w') as f:
        f.write(','.join(header) + '\n')
        for r in rows:
            f.write(','.join(r) + '\n')

    print(f'  → saved {out_path}')
    print(f'  → saved {csv_path}')


# Inverse method name → id
METHOD_SHORTS_INV = {'M1': 1, 'M2': 2, 'M3': 3, 'M4': 4}


def main():
    p = argparse.ArgumentParser(description='实验一: 方向选择性验证可视化')
    p.add_argument('--m1', type=str, default=None, help='M1 CSV path')
    p.add_argument('--m2', type=str, default=None, help='M2 CSV path')
    p.add_argument('--m3', type=str, default=None, help='M3 CSV path')
    p.add_argument('--m4', type=str, default=None, help='M4 CSV path (most important)')
    p.add_argument('--output', type=str, default='/tmp/exp1_figs/',
                   help='输出目录')
    p.add_argument('--collision-start', type=float, default=3.0,
                   help='碰撞开始时间 [s]')
    p.add_argument('--collision-end', type=float, default=5.0,
                   help='碰撞结束时间 [s]')
    args = p.parse_args()

    setup_plot_style()

    # 加载数据
    paths = {'M1': args.m1, 'M2': args.m2, 'M3': args.m3, 'M4': args.m4}
    datas = {}
    for m, path in paths.items():
        if path is None:
            print(f'[skip] {m}: no path given')
            continue
        if not os.path.exists(path):
            print(f'[skip] {m}: file not found: {path}')
            continue
        print(f'[load] {m}: {path}')
        datas[m] = load_csv(path)

    if not datas:
        print('No data loaded. Specify at least one --m1/--m2/--m3/--m4')
        sys.exit(1)

    t_coll = (args.collision_start, args.collision_end)
    t_pre = (1.0, 3.0)

    print('\n[Fig 1] 4-method time-domain comparison')
    figure_1_time_domain(datas, t_coll, args.output)

    if 'M4' in datas:
        print('[Fig 2] A-DLS λ² evolution')
        figure_2_lambda_evolution(datas['M4'], t_coll, args.output)

    print('[Fig 3] Directional selectivity index η')
    figure_3_eta_bar(datas, t_coll, args.output)

    print('[Tab 1] Summary table')
    table_1_summary(datas, t_pre, t_coll, args.output)

    print(f'\nAll outputs saved to: {args.output}')


if __name__ == '__main__':
    main()
