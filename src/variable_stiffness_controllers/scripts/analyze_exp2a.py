#!/usr/bin/env python3
"""
实验 2A: 精度-安全 Pareto 前沿
===============================

输入: 多个 CSV (每个是一次试验, 不同的方法/力幅值/阻尼参数)
输出:
    Fig 4: Pareto 散点图: 横轴 ρ_F (安全)，纵轴 ρ_P (精度)，4 方法不同标记
    Fig 5: M4 在不同 λ²_max 下的参数扫描曲线
    Tab 2: 代表条件下 (F=25N, Z 方向) 的数值对比

用法:
    python3 analyze_exp2a.py --csv-dir /tmp/exp2a/ --output /tmp/exp2a_figs/
    # CSV 命名约定: m{1,2,3,4}_F{force}_lambda{value}.csv
    # 例如: m4_F25_lambda0.5.csv
"""

import argparse
import os
import re
import sys
import glob
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_utils import (
    load_csv, extract, setup_plot_style, save_figure,
    compute_rmse, METHOD_COLORS, METHOD_SHORTS
)


# Filename pattern: m{N}_F{value}_lambda{value}.csv
FN_PATTERN = re.compile(
    r'm(\d+)_F(\d+(?:\.\d+)?)(?:_lambda(\d+(?:\.\d+)?))?(?:_d(\d+))?\.csv'
)


def parse_filename(path: str) -> dict:
    """从文件名解析实验参数。"""
    name = os.path.basename(path)
    m = FN_PATTERN.match(name)
    if not m:
        return None
    return {
        'method_id': int(m.group(1)),
        'F_target':  float(m.group(2)),
        'lambda_max': float(m.group(3)) if m.group(3) else None,
        'direction': int(m.group(4)) if m.group(4) else 2,  # default Z
    }


def compute_pareto_metrics(data: np.ndarray, t_coll: tuple,
                           t_pre: tuple) -> dict:
    """计算单次试验的 (ρ_P, ρ_F) 点。
    ρ_P = RMSE_health^coll / RMSE_health^pre  (精度退化比)
    ρ_F = ‖r‖_steady / ‖F‖_applied             (碰撞力衰减比, 越小越安全)
    """
    t = extract(data, 't')
    error = extract(data, 'error')
    r_corr = extract(data, 'r_corr')
    F_applied = extract(data, 'F_ext_applied')

    rmse_pre = compute_rmse(error[:, :3], t, *t_pre)
    rmse_coll = compute_rmse(error[:, :3], t, *t_coll)

    healthy = [0, 1]  # XY 健康，Z 碰撞
    rmse_health_pre = rmse_pre[healthy].mean()
    rmse_health_coll = rmse_coll[healthy].mean()

    # ρ_P = relative degradation of healthy directions, with a floor on
    # rmse_pre to prevent division by near-zero (which produces 1e7+ values
    # when static-hold pre-collision tracking is essentially perfect).
    # Floor at 0.5 mm = 0.0005 m → ratio is bounded for tracking-good baselines.
    rho_P = rmse_health_coll / max(rmse_health_pre, 0.0005)

    mask_coll = (t >= t_coll[0]) & (t <= t_coll[1])
    if mask_coll.sum() == 0:
        rho_F = 1.0
    else:
        r_steady = np.linalg.norm(r_corr[mask_coll], axis=1, ord=np.inf).mean()
        F_norm = np.linalg.norm(F_applied[mask_coll], axis=1).max() + 1e-9
        rho_F = r_steady / F_norm

    return {
        'rho_P': rho_P,
        'rho_F': rho_F,
        'rmse_health': rmse_health_coll * 1000,  # mm
    }


def figure_4_pareto(records: list, output_dir: str):
    """Fig 4: Pareto 散点图."""
    fig, ax = plt.subplots(figsize=(8.5, 6.5))

    method_data = {1: [], 2: [], 3: [], 4: []}
    for rec in records:
        method_data.setdefault(rec['method_id'], []).append(rec)

    markers = {1: 's', 2: 'D', 3: 'o', 4: '*'}
    sizes = {1: 70, 2: 70, 3: 70, 4: 200}

    for mid in [1, 2, 3, 4]:
        recs = method_data.get(mid, [])
        if not recs:
            continue
        rho_F = [r['metrics']['rho_F'] for r in recs]
        rho_P = [r['metrics']['rho_P'] for r in recs]
        ax.scatter(rho_F, rho_P,
                   c=METHOD_COLORS[mid],
                   marker=markers[mid],
                   s=sizes[mid],
                   edgecolors='black',
                   linewidths=0.7,
                   label=f'M{mid}',
                   alpha=0.85)

    ax.set_xlabel(r'$\rho_F = \|r\|_{ss} / \|\tau_{ext}\|$  (safer $\leftarrow$)',
                  fontsize=11)
    ax.set_ylabel(r'$\rho_P = \mathrm{RMSE}_{health}^{coll} / '
                  r'\mathrm{RMSE}_{health}^{pre}$  ($\uparrow$ accuracy worse)',
                  fontsize=11)
    ax.set_title('Fig 4 — Pareto Frontier: Accuracy vs Safety\n'
                 'M4 should occupy the bottom-left (best of both)', fontsize=12)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_figure(fig, output_dir, 'fig4_pareto')
    plt.close(fig)


def figure_5_lambda_sweep(records: list, output_dir: str):
    """Fig 5: M4 在 λ²_max 扫描下的参数曲线."""
    m4_recs = [r for r in records if r['method_id'] == 4 and
               r['params']['lambda_max'] is not None]
    if not m4_recs:
        print('  [skip] no M4 lambda sweep data')
        return

    # Group by lambda_max
    by_lambda = {}
    for r in m4_recs:
        lam = r['params']['lambda_max']
        by_lambda.setdefault(lam, []).append(r)

    fig, ax = plt.subplots(figsize=(9, 6))
    cmap = plt.cm.viridis
    sorted_lambdas = sorted(by_lambda.keys())
    norm = matplotlib.colors.Normalize(vmin=min(sorted_lambdas),
                                       vmax=max(sorted_lambdas))

    for lam in sorted_lambdas:
        recs = by_lambda[lam]
        recs.sort(key=lambda r: r['params']['F_target'])
        rho_F = [r['metrics']['rho_F'] for r in recs]
        rho_P = [r['metrics']['rho_P'] for r in recs]
        ax.plot(rho_F, rho_P,
                color=cmap(norm(lam)),
                marker='o', markersize=8,
                label=f'$\\lambda^2_{{max}}={lam}$',
                linewidth=1.2)

    ax.set_xlabel(r'$\rho_F$ (safer $\leftarrow$)', fontsize=11)
    ax.set_ylabel(r'$\rho_P$ (accuracy worse $\uparrow$)', fontsize=11)
    ax.set_title('Fig 5 — M4 (A-DLS) Parameter Sweep on $\\lambda^2_{max}$\n'
                 'Each curve is one $\\lambda^2_{max}$ swept over force levels',
                 fontsize=12)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_figure(fig, output_dir, 'fig5_lambda_sweep')
    plt.close(fig)


def table_2_representative(records: list, target_F: float, output_dir: str):
    """Tab 2: 代表条件 (F=target_F) 下的数值对比. 优先选择无 λ 扫描的基础试验."""
    sel = [r for r in records
           if abs(r['params']['F_target'] - target_F) < 1.0
           and r['params'].get('direction', 2) == 2
           and r['params'].get('lambda_max') is None]   # 排除 λ 扫描的重复
    # 同一方法只保留一行
    seen = set()
    sel_unique = []
    for r in sorted(sel, key=lambda x: x['method_id']):
        if r['method_id'] not in seen:
            sel_unique.append(r)
            seen.add(r['method_id'])
    sel = sel_unique

    out_path = os.path.join(output_dir, 'tab2_representative.md')
    os.makedirs(output_dir, exist_ok=True)
    with open(out_path, 'w') as f:
        f.write(f'# Table 2 — Representative Pareto Comparison (F = {target_F} N, Z direction)\n\n')
        f.write('| Method | ρ_F (safer ↓) | ρ_P (accuracy ↑) | '
                'RMSE_health [mm] |\n')
        f.write('|---|---|---|---|\n')
        for r in sel:
            mid = r['method_id']
            f.write(f"| M{mid} | "
                    f"{r['metrics']['rho_F']:.3f} | "
                    f"{r['metrics']['rho_P']:.2f} | "
                    f"{r['metrics']['rmse_health']:.2f} |\n")
    print(f'  → saved {out_path}')


def main():
    p = argparse.ArgumentParser(description='实验 2A: Pareto 前沿可视化')
    p.add_argument('--csv-dir', type=str, required=True,
                   help='含 m{N}_F{V}_lambda{V}.csv 的目录')
    p.add_argument('--output', type=str, default='/tmp/exp2a_figs/',
                   help='输出目录')
    p.add_argument('--collision-start', type=float, default=2.0)
    p.add_argument('--collision-end', type=float, default=4.0)
    p.add_argument('--target-F', type=float, default=25.0,
                   help='Tab 2 代表力幅值')
    args = p.parse_args()

    setup_plot_style()

    # 扫描目录
    csv_files = sorted(glob.glob(os.path.join(args.csv_dir, '*.csv')))
    print(f'[scan] found {len(csv_files)} CSV files in {args.csv_dir}')

    records = []
    for path in csv_files:
        params = parse_filename(path)
        if params is None:
            print(f'  [skip] cannot parse: {os.path.basename(path)}')
            continue
        try:
            data = load_csv(path)
        except Exception as e:
            print(f'  [skip] load failed: {os.path.basename(path)} ({e})')
            continue
        metrics = compute_pareto_metrics(
            data, (args.collision_start, args.collision_end),
            (max(0.5, args.collision_start - 1.5), args.collision_start - 0.5)
        )
        records.append({
            'params': params,
            'method_id': params['method_id'],
            'metrics': metrics,
        })
        print(f'  [load] M{params["method_id"]} F={params["F_target"]} '
              f'λ={params["lambda_max"]} → ρ_F={metrics["rho_F"]:.3f}, '
              f'ρ_P={metrics["rho_P"]:.2f}')

    if not records:
        print('No usable records.')
        sys.exit(1)

    print(f'\n[Fig 4] Pareto scatter')
    figure_4_pareto(records, args.output)
    print('[Fig 5] M4 λ²_max sweep')
    figure_5_lambda_sweep(records, args.output)
    print('[Tab 2] Representative table')
    table_2_representative(records, args.target_F, args.output)

    print(f'\nAll outputs saved to: {args.output}')


if __name__ == '__main__':
    main()
