#!/usr/bin/env python3
"""
实验 2B: 弹弓效应 + 非对称 EMA + 参考轨迹调制
==============================================

输入: 4 个配置 (C1/C2/C3/C4) 的诊断 CSV
输出:
    Fig 6: 四配置 ‖q̇_cmd(t)‖ 在 [4.5, 6.5] 叠加 (C4 应无尖峰)
    Fig 7: 四配置 λ²_Z(t) 在 [2.5, 6.5] 叠加 (展示 attack/decay 速度)
    Fig 8: v_peak / Δx_over / t_settle 三指标 bar chart
    Tab 3: 四配置 5 指标汇总

四配置 (论文 §2B.3):
    C1: ema=none,       ref=off  ← v5.9 之前的裸 A-DLS, 最危险
    C2: ema=symmetric,  ref=off  ← v5.9 基线: 平滑但未解决弹弓
    C3: ema=asymmetric, ref=off  ← v5.10 EMA: 第一道防线
    C4: ema=asymmetric, ref=on   ← v5.10 完整: 两道防线

用法:
    python3 analyze_exp2b.py \
        --c1 /tmp/c1.csv --c2 /tmp/c2.csv \
        --c3 /tmp/c3.csv --c4 /tmp/c4.csv \
        --output /tmp/exp2b_figs/
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
    load_csv, extract, setup_plot_style, shade_collision_region,
    save_figure
)

CONFIG_NAMES = {
    'C1': 'C1: no EMA, no ref-mod (v5.9-, dangerous)',
    'C2': 'C2: symmetric EMA, no ref-mod (v5.9)',
    'C3': 'C3: asymmetric EMA, no ref-mod (v5.10 step 1)',
    'C4': 'C4: asymmetric EMA + ref-mod (v5.10 full)',
}
CONFIG_COLORS = {'C1': '#d62728', 'C2': '#ff7f0e',
                 'C3': '#1f77b4', 'C4': '#2ca02c'}


def compute_metrics(data, t_force_off=5.0,
                    t_force_on=3.0) -> dict:
    """Compute v_peak, Δx_over, t_settle, τ_rise, τ_fall (paper §2B.4).

    Note on lambda2 indexing: λ²_3 is the 3rd SVD principal direction,
    which empirically is dominant for TCP-Z collisions in Franka's
    elbow-up configuration. We treat it as the proxy for Z compliance.
    """
    t = extract(data, 't')
    dq_cmd = extract(data, 'dq_cmd')
    error = extract(data, 'error')
    lambda2 = extract(data, 'lambda2')

    # Slice masks for relevant intervals
    pre_collision = (t < t_force_on)
    post_off = (t >= t_force_off) & (t <= t_force_off + 0.5)
    rebound_window = (t >= t_force_off) & (t <= t_force_off + 1.0)
    settle_window = t >= t_force_off

    res = {}

    # v_peak: peak joint velocity command magnitude after force off
    if post_off.sum() > 0:
        dq_norms = np.linalg.norm(dq_cmd[post_off], ord=np.inf, axis=1)
        res['v_peak'] = float(dq_norms.max())
    else:
        res['v_peak'] = 0.0

    # Δx_over: position overshoot after force off, relative to pre-collision
    pre_norm = np.linalg.norm(error[pre_collision][-100:].mean(axis=0)[:3]) \
        if pre_collision.sum() > 100 else 0.0
    if rebound_window.sum() > 0:
        rebound_norms = np.linalg.norm(error[rebound_window, :3], axis=1)
        res['delta_x_over_mm'] = float((rebound_norms.max() - pre_norm) * 1000)
    else:
        res['delta_x_over_mm'] = 0.0

    # t_settle: time to return to within 10% of pre-collision error norm
    target_norm = max(pre_norm * 1.1, 0.002)  # min 2mm floor
    if settle_window.sum() > 0:
        err_norms_after = np.linalg.norm(error[settle_window, :3], axis=1)
        t_after = t[settle_window]
        # find first time where error returns below target
        settled_idx = np.where(err_norms_after <= target_norm)[0]
        if len(settled_idx) > 0:
            # require persistence (next 50ms also below)
            for idx in settled_idx:
                end_idx = min(idx + 50, len(err_norms_after))
                if np.all(err_norms_after[idx:end_idx] <= target_norm * 1.5):
                    res['t_settle'] = float(t_after[idx] - t_force_off)
                    break
            else:
                res['t_settle'] = float('inf')
        else:
            res['t_settle'] = float('inf')
    else:
        res['t_settle'] = float('inf')

    # τ_rise: time for λ²_3 to rise from baseline to 50% λ_max during collision
    coll = (t >= t_force_on) & (t < t_force_on + 0.5)
    lam_z = lambda2[:, 2]
    if coll.sum() > 0:
        coll_lam = lam_z[coll]
        coll_t = t[coll]
        target = 0.50 * 0.5  # 50% of lambda_max=0.5 (lower bar for realistic detection)
        rise_idx = np.where(coll_lam >= target)[0]
        if len(rise_idx) > 0:
            res['tau_rise_ms'] = float((coll_t[rise_idx[0]] - t_force_on) * 1000)
        else:
            res['tau_rise_ms'] = float('nan')
    else:
        res['tau_rise_ms'] = float('nan')

    # τ_fall: time for λ²_3 to fall from peak back to ~1.1 × λ_base
    fall_window = (t >= t_force_off) & (t <= t_force_off + 1.0)
    if fall_window.sum() > 0:
        fall_lam = lam_z[fall_window]
        fall_t = t[fall_window]
        target = 1.1 * 1e-4
        fall_idx = np.where(fall_lam <= target)[0]
        if len(fall_idx) > 0:
            res['tau_fall_ms'] = float((fall_t[fall_idx[0]] - t_force_off) * 1000)
        else:
            res['tau_fall_ms'] = float('nan')
    else:
        res['tau_fall_ms'] = float('nan')

    return res


def figure_6_dq_overlay(datas, output_dir, t_window=(4.5, 6.5)):
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for cid, data in datas.items():
        if data is None:
            continue
        t = extract(data, 't')
        dq_cmd = extract(data, 'dq_cmd')
        m = (t >= t_window[0]) & (t <= t_window[1])
        dq_norm = np.linalg.norm(dq_cmd[m], ord=np.inf, axis=1)
        ax.plot(t[m], dq_norm, color=CONFIG_COLORS[cid],
                label=CONFIG_NAMES[cid], linewidth=1.5)
    ax.axvline(x=5.0, color='black', linestyle='--', alpha=0.5,
               label='Force OFF (t=5s)')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel(r'$\|\dot{q}_{cmd}\|_\infty$ [rad/s]')
    ax.set_title('Fig 6 — Slingshot Velocity Spike Comparison\n'
                 '(C4 with v5.10 full mitigation should be lowest)',
                 fontsize=12)
    ax.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    save_figure(fig, output_dir, 'fig6_dq_slingshot')
    plt.close(fig)


def figure_7_lambda_overlay(datas, output_dir, t_window=(2.5, 6.5)):
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for cid, data in datas.items():
        if data is None:
            continue
        t = extract(data, 't')
        lam = extract(data, 'lambda2')[:, 2]  # 3rd SVD direction (Z-dominant)
        m = (t >= t_window[0]) & (t <= t_window[1])
        ax.plot(t[m], lam[m], color=CONFIG_COLORS[cid],
                label=CONFIG_NAMES[cid], linewidth=1.5)
    shade_collision_region(ax, 3.0, 5.0)
    ax.axvline(x=5.0, color='black', linestyle='--', alpha=0.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel(r'$\lambda^2_3(t)$ (Z-dominant SVD direction)')
    ax.set_yscale('log')
    ax.set_title('Fig 7 — A-DLS Damping Attack/Decay under Different EMA Strategies',
                 fontsize=12)
    ax.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    save_figure(fig, output_dir, 'fig7_lambda_ema')
    plt.close(fig)


def figure_8_metrics_bar(metrics_dict, output_dir):
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5))
    cids = list(metrics_dict.keys())
    keys_titles = [
        ('v_peak', r'$v_{peak}$ [rad/s]', 'Joint velocity spike'),
        ('delta_x_over_mm', r'$\Delta x_{over}$ [mm]', 'Position overshoot'),
        ('t_settle', r'$t_{settle}$ [s]', 'Recovery time'),
    ]
    for ax, (key, ylabel, title) in zip(axes, keys_titles):
        vals = [metrics_dict[c].get(key, float('nan')) for c in cids]
        colors = [CONFIG_COLORS[c] for c in cids]
        # Cap inf for plotting
        plot_vals = [v if np.isfinite(v) else 99 for v in vals]
        bars = ax.bar(cids, plot_vals, color=colors,
                      edgecolor='black', linewidth=0.8)
        for bar, v in zip(bars, vals):
            label = f'{v:.2f}' if np.isfinite(v) else 'no-settle'
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height(), label,
                    ha='center', va='bottom', fontsize=10)
        ax.set_title(title, fontsize=11)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
    fig.suptitle('Fig 8 — Experiment 2B Quantitative Summary',
                 fontsize=12, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    save_figure(fig, output_dir, 'fig8_metrics_bar')
    plt.close(fig)


def table_3_summary(metrics_dict, output_dir):
    out_path = os.path.join(output_dir, 'tab3_summary.md')
    os.makedirs(output_dir, exist_ok=True)
    with open(out_path, 'w') as f:
        f.write('# Table 3 — Experiment 2B: Slingshot Mitigation Comparison\n\n')
        f.write('| Config | τ_rise [ms] | τ_fall [ms] | v_peak [rad/s] | '
                'Δx_over [mm] | t_settle [s] |\n')
        f.write('|---|---|---|---|---|---|\n')
        for cid, m in metrics_dict.items():
            rise = f"{m['tau_rise_ms']:.1f}" if np.isfinite(m['tau_rise_ms']) else '—'
            fall = f"{m['tau_fall_ms']:.1f}" if np.isfinite(m['tau_fall_ms']) else '—'
            settle = f"{m['t_settle']:.2f}" if np.isfinite(m['t_settle']) else 'no-settle'
            f.write(f"| {cid} | {rise} | {fall} | "
                    f"{m['v_peak']:.2f} | {m['delta_x_over_mm']:.2f} | "
                    f"{settle} |\n")
    print(f'  → saved {out_path}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--c1', type=str, default=None)
    p.add_argument('--c2', type=str, default=None)
    p.add_argument('--c3', type=str, default=None)
    p.add_argument('--c4', type=str, default=None)
    p.add_argument('--output', type=str, default='/tmp/exp2b_figs/')
    p.add_argument('--collision-start', type=float, default=3.0)
    p.add_argument('--collision-end', type=float, default=5.0)
    args = p.parse_args()

    setup_plot_style()

    datas = {}
    for cid, path in [('C1', args.c1), ('C2', args.c2),
                       ('C3', args.c3), ('C4', args.c4)]:
        if path is None or not os.path.exists(path):
            print(f'[skip] {cid}: {path}')
            continue
        datas[cid] = load_csv(path)
        print(f'[load] {cid}: {path}')

    if not datas:
        print('No data loaded.')
        sys.exit(1)

    metrics = {}
    for cid, data in datas.items():
        metrics[cid] = compute_metrics(data, args.collision_end,
                                       args.collision_start)
        print(f'[metrics] {cid}: {metrics[cid]}')

    print('\n[Fig 6] dq slingshot overlay')
    figure_6_dq_overlay(datas, args.output)
    print('[Fig 7] λ²_Z EMA dynamics')
    figure_7_lambda_overlay(datas, args.output)
    print('[Fig 8] Three-metric bars')
    figure_8_metrics_bar(metrics, args.output)
    print('[Tab 3] Summary')
    table_3_summary(metrics, args.output)

    print(f'\nAll outputs in: {args.output}')


if __name__ == '__main__':
    main()
