#!/usr/bin/env python3
"""
实验 3B: 奇异位形边界测试
==========================

输入: M1 和 M4 在向完全伸展位形过渡时的 CSV
输出:
    Fig 12: M1 vs M4 的 ‖q̇(t)‖ 对比 (M1 发散, M4 有界)
    Fig 13: M4 的 σ_min(t), V(t), c_eff(t) 三轴时域图 (验证定理 3)
    Fig 14: M4 的 6 个 μ_k(t) 曲线 (验证 μ_k ∈ (0,1))
    Tab 5: σ_min = 0.01/0.001/0.0001 时关键指标对比

用法:
    python3 analyze_exp3b.py --m1 /tmp/exp3b_M1.csv --m4 /tmp/exp3b_M4.csv \
                             --output /tmp/exp3b_figs/
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_utils import (
    load_csv, extract, setup_plot_style, save_figure,
    METHOD_COLORS, AXIS_LABELS
)


def figure_12_dq_comparison(data_m1: np.ndarray, data_m4: np.ndarray,
                            output_dir: str):
    """Fig 12: ‖q̇‖ 对比 (M1 发散，M4 有界)."""
    fig, ax = plt.subplots(figsize=(10, 6))

    if data_m1 is not None:
        t1 = extract(data_m1, 't')
        dq1 = extract(data_m1, 'dq')
        ax.plot(t1, np.linalg.norm(dq1, axis=1),
                color=METHOD_COLORS[1], label='M1 (Moore-Penrose)',
                linewidth=1.5)

    if data_m4 is not None:
        t4 = extract(data_m4, 't')
        dq4 = extract(data_m4, 'dq')
        ax.plot(t4, np.linalg.norm(dq4, axis=1),
                color=METHOD_COLORS[4], label='M4 (A-DLS)',
                linewidth=1.5)

    # Franka 速度规格上限
    ax.axhline(y=2.175, color='red', linestyle='--', linewidth=1,
               label='Franka spec limit (2.175 rad/s)')

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('$\\|\\dot{q}(t)\\|_2$ [rad/s]')
    ax.set_title('Fig 12 — Joint velocity norm at singular configuration\n'
                 'M1 should diverge as $\\sigma_{\\min} \\to 0$; '
                 'M4 should stay bounded',
                 fontsize=12)
    ax.legend(loc='upper left', fontsize=10)

    plt.tight_layout()
    save_figure(fig, output_dir, 'fig12_dq_singular')
    plt.close(fig)


def figure_13_theorem3_validation(data_m4: np.ndarray, output_dir: str):
    """Fig 13: M4 的 σ_min(t), V(t), c_eff(t) 验证定理 3."""
    if data_m4 is None:
        print('  [skip] no M4 data')
        return

    t = extract(data_m4, 't')
    sigma = extract(data_m4, 'sigma')   # (N, 6)
    error = extract(data_m4, 'error')   # (N, 6)

    sigma_min = sigma[:, -1]  # σ_6
    V = 0.5 * np.sum(error ** 2, axis=1)  # Lyapunov function

    # c_eff = -V̇/V via numerical differentiation
    dV = np.gradient(V, t)
    c_eff = -dV / (V + 1e-12)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle('Fig 13 — Theorem 3 validation (A-DLS, singular trajectory)',
                 fontsize=13, fontweight='bold')

    axes[0].plot(t, sigma_min, color='#1f77b4', linewidth=1.5)
    axes[0].set_ylabel('$\\sigma_{\\min}(t)$ [—]')
    axes[0].set_yscale('log')
    axes[0].axhline(y=1e-3, color='red', linestyle=':', alpha=0.5,
                    label='$\\sigma=10^{-3}$')
    axes[0].legend()

    axes[1].plot(t, V, color='#2ca02c', linewidth=1.5)
    axes[1].set_ylabel('Lyapunov $V(t) = \\frac{1}{2}\\|\\mathbf{e}\\|^2$')
    axes[1].set_yscale('log')

    axes[2].plot(t, c_eff, color='#d62728', linewidth=1.5)
    axes[2].set_ylabel('$c_{eff}(t) = -\\dot{V}/V$')
    axes[2].set_xlabel('Time [s]')
    axes[2].axhline(y=0, color='black', linestyle='--', alpha=0.3)
    axes[2].set_title('Effective convergence rate (should fade to 0 as $\\sigma_{\\min} \\to 0$, '
                      'but never become negative)',
                      fontsize=10)

    plt.tight_layout()
    save_figure(fig, output_dir, 'fig13_theorem3_validation')
    plt.close(fig)


def figure_14_mu_evolution(data_m4: np.ndarray, output_dir: str):
    """Fig 14: M4 的 6 个 μ_k(t)（μ_k = σ_k²/(σ_k²+λ_k²)）."""
    if data_m4 is None:
        print('  [skip] no M4 data')
        return

    t = extract(data_m4, 't')
    sigma = extract(data_m4, 'sigma')      # (N, 6)
    lambda2 = extract(data_m4, 'lambda2')  # (N, 6)

    mu = (sigma ** 2) / (sigma ** 2 + lambda2)

    fig, ax = plt.subplots(figsize=(10, 6))
    colors = plt.cm.tab10(np.arange(6))
    for k in range(6):
        ax.plot(t, mu[:, k], color=colors[k],
                label=f'$\\mu_{k+1}$ ({AXIS_LABELS[k]})',
                linewidth=1.3)

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('$\\mu_k(t) = \\sigma_k^2 / (\\sigma_k^2 + \\lambda_k^2)$')
    ax.set_title('Fig 14 — Closed-loop direction-wise eigenvalues $\\mu_k(t)$\n'
                 'All values must stay in (0, 1) — defines exponential rate per direction',
                 fontsize=12)
    ax.set_ylim(-0.05, 1.1)
    ax.axhline(y=0, color='black', linestyle=':', alpha=0.3)
    ax.axhline(y=1, color='black', linestyle=':', alpha=0.3)
    ax.legend(loc='center right', ncol=2)

    plt.tight_layout()
    save_figure(fig, output_dir, 'fig14_mu_evolution')
    plt.close(fig)


def table_5_singularity_comparison(data_m1: np.ndarray, data_m4: np.ndarray,
                                    output_dir: str):
    """Tab 5: σ_min = 0.01/0.001/0.0001 时的指标对比."""
    thresholds = [0.01, 0.001, 0.0001]

    out_path = os.path.join(output_dir, 'tab5_singularity.md')
    os.makedirs(output_dir, exist_ok=True)
    with open(out_path, 'w') as f:
        f.write('# Table 5 — Behavior at singular thresholds\n\n')
        f.write('| Method | $\\sigma_{\\min}$ | $\\|\\dot{q}\\|_2$ [rad/s] | '
                '$V(t)$ | Status |\n')
        f.write('|---|---|---|---|---|\n')

        for data, method in [(data_m1, 'M1'), (data_m4, 'M4')]:
            if data is None:
                continue
            t = extract(data, 't')
            sigma = extract(data, 'sigma')
            dq = extract(data, 'dq')
            error = extract(data, 'error')
            sigma_min = sigma[:, -1]
            V = 0.5 * np.sum(error ** 2, axis=1)
            dq_norm = np.linalg.norm(dq, axis=1)

            for thr in thresholds:
                idx = np.where(sigma_min <= thr)[0]
                if len(idx) == 0:
                    f.write(f'| {method} | {thr} | — | — | not reached |\n')
                    continue
                i = idx[0]
                status = 'diverges' if dq_norm[i] > 2.0 else 'bounded'
                f.write(f'| {method} | {thr} | {dq_norm[i]:.3f} | '
                        f'{V[i]:.4f} | {status} |\n')

    print(f'  → saved {out_path}')


def main():
    p = argparse.ArgumentParser(description='实验 3B: 奇异位形测试可视化')
    p.add_argument('--m1', type=str, default=None, help='M1 CSV')
    p.add_argument('--m4', type=str, default=None, help='M4 CSV')
    p.add_argument('--output', type=str, default='/tmp/exp3b_figs/')
    args = p.parse_args()

    setup_plot_style()

    data_m1 = load_csv(args.m1) if args.m1 and os.path.exists(args.m1) else None
    data_m4 = load_csv(args.m4) if args.m4 and os.path.exists(args.m4) else None

    if data_m1 is None and data_m4 is None:
        print('No data loaded.')
        sys.exit(1)

    print('[Fig 12] dq comparison')
    figure_12_dq_comparison(data_m1, data_m4, args.output)
    print('[Fig 13] Theorem 3 validation')
    figure_13_theorem3_validation(data_m4, args.output)
    print('[Fig 14] mu_k evolution')
    figure_14_mu_evolution(data_m4, args.output)
    print('[Tab 5] Singularity threshold comparison')
    table_5_singularity_comparison(data_m1, data_m4, args.output)

    print(f'\nAll outputs saved to: {args.output}')


if __name__ == '__main__':
    main()
