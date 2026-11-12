#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实验 1-3: 多方向碰撞独立性 — 验证 Λ_cart 对角结构、六个笛卡尔方向互不耦合。

对 M4 分别在世界系 X / Y / Z 三个方向施加同样大小的末端碰撞力, 取碰撞期(8-10s)
每个方向激活出的阻尼对角 Λ_ii。若 Λ_cart 真为对角解耦:
  力在 X → 只 Λ_XX 升;  力在 Y → 只 Λ_YY 升;  力在 Z → 只 Λ_ZZ 升。
画成 3×6 热图: 强对角主导 = 解耦证据(无跨方向泄漏)。

CSV 列: 0=t, 20-25=err(6), 73-78=Fx(6), 87-92=Λ对角(6)
"""
import csv, argparse, os
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

CART = ['X','Y','Z','RX','RY','RZ']
LB, LM = 1e-4, 0.1

def load(fn, lo=8.0, hi=10.0):
    with open(fn) as f:
        r = csv.reader(f); next(r)
        rows = [x for x in r if len(x) > 92 and lo < float(x[0]) <= hi]
    Lc  = np.array([[float(x[87+i]) for i in range(6)] for x in rows])
    err = np.array([[float(x[20+i]) for i in range(6)] for x in rows])*1e3
    F   = np.array([[float(x[73+i]) for i in range(6)] for x in rows])
    return Lc.mean(0), np.sqrt((err**2).mean(0)), np.abs(F).mean(0)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', required=True); ap.add_argument('--y', required=True)
    ap.add_argument('--z', required=True); ap.add_argument('--output', required=True)
    ap.add_argument('--collision-start', type=float, default=8.0)
    ap.add_argument('--collision-end',   type=float, default=10.0)
    a = ap.parse_args()
    os.makedirs(a.output, exist_ok=True)

    dirs = ['X','Y','Z']; files = [a.x, a.y, a.z]
    LcM = np.zeros((3,6)); rmsM = np.zeros((3,6)); Fapp = np.zeros(3)
    for i,fn in enumerate(files):
        lc, rms, F = load(fn, a.collision_start, a.collision_end)
        LcM[i], rmsM[i] = lc, rms
        Fapp[i] = F[i]   # 该方向施加的力
        print(f'力@{dirs[i]}: Λ_diag={np.round(lc,5)}  |F|={F[i]:.1f}N')

    # 选择性: 对角阻尼 / 同行最大非对角阻尼
    sel = [LcM[i,i] / max(np.delete(LcM[i],i).max(), LB) for i in range(3)]
    print('对角/非对角阻尼比 (越大越解耦):', np.round(sel,1))

    fig = plt.figure(figsize=(13,5))
    gs = fig.add_gridspec(1,2, width_ratios=[1.25,1], wspace=0.3)
    fig.suptitle('Multi-direction collision independence — diagonal structure of $\\Lambda_{cart}$',
                 fontweight='bold', fontsize=13)

    # (a) 热图: 行=施力方向, 列=被激活的阻尼轴
    ax = fig.add_subplot(gs[0,0])
    im = ax.imshow(LcM, norm=LogNorm(vmin=LB*0.5, vmax=LM*1.2),
                   cmap='inferno', aspect='auto')
    ax.set_xticks(range(6)); ax.set_xticklabels(CART)
    ax.set_yticks(range(3)); ax.set_yticklabels([f'force @ {d}' for d in dirs])
    ax.set_xlabel('damping axis $\\Lambda_{ii}$'); ax.set_ylabel('applied collision direction')
    ax.set_title('(a) collision-avg damping $\\Lambda_{ii}$\nbright only on the diagonal → directions decoupled')
    for i in range(3):
        for j in range(6):
            v = LcM[i,j]
            ax.text(j,i, f'{v:.0e}', ha='center', va='center', fontsize=8,
                    color='black' if v>LM*0.3 else 'white')
        ax.add_patch(plt.Rectangle((i-0.5,i-0.5),1,1, fill=False, ec='cyan', lw=2.5))
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label=r'$\Lambda_{ii}$')

    # (b) 每个施力方向: 受力轴 vs 健康轴 RMSE (顺从只发生在受力轴)
    ax2 = fig.add_subplot(gs[0,1])
    x = np.arange(3); w=0.38
    hit  = [rmsM[i,i] for i in range(3)]                       # 受力轴误差(顺从)
    other = [np.delete(rmsM[i,:3],i).mean() for i in range(3)]  # 其余平动轴均值(保精度)
    ax2.bar(x-w/2, hit,  w, label='forced axis (yields)', color='tab:red')
    ax2.bar(x+w/2, other,w, label='other axes (preserved)', color='tab:blue')
    ax2.set_xticks(x); ax2.set_xticklabels([f'@ {d}' for d in dirs])
    ax2.set_ylabel('tracking RMSE [mm]'); ax2.set_xlabel('applied collision direction')
    ax2.set_title('(b) compliance localizes to the forced axis\nforced axis yields, the other two stay precise')
    ax2.legend(fontsize=9)
    for i in range(3):
        ax2.text(i-w/2, hit[i]+0.4, f'{hit[i]:.1f}', ha='center', fontsize=8)
        ax2.text(i+w/2, other[i]+0.4, f'{other[i]:.1f}', ha='center', fontsize=8)

    fig.subplots_adjust(top=0.84)
    fig.savefig(a.output+'/fig_multidir_independence.png', dpi=130, bbox_inches='tight')
    fig.savefig(a.output+'/fig_multidir_independence.pdf', bbox_inches='tight')
    print('saved →', a.output+'/fig_multidir_independence.png/pdf')

if __name__ == '__main__':
    main()
