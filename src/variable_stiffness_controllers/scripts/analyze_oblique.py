#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实验 1-3 延伸: 45° 斜向碰撞独立性 (每个施力方向单独一张图)

每张图 = 左: 该次碰撞期的阻尼对角热力图 (1×6, 看哪两个分量轴被点亮、正交轴守 base)
         右: X/Y/Z 三个平动轴的跟踪误差时域曲线 (高亮正交轴)

输出:
  fig_oblique_XY.png  — XY 平面 45° 力; 受力 X,Y; 正交 Z
  fig_oblique_XZ.png  — XZ 平面 45° 力; 受力 X,Z; 正交 Y

CSV 列: 0=t, 20-25=err(6,m), 73-78=Fx(6), 87-92=Λ对角(6)
"""
import csv, argparse, os
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

CART = ['X','Y','Z','RX','RY','RZ']
LB, LM = 1e-4, 0.1
TRA = ['X','Y','Z']
COL = {'X':'tab:red', 'Y':'tab:green', 'Z':'tab:blue'}

def load_series(fn):
    with open(fn) as f:
        r = csv.reader(f); next(r)
        rows = [x for x in r if len(x) > 92]
    t   = np.array([float(x[0]) for x in rows])
    err = np.array([[float(x[20+i]) for i in range(6)] for x in rows])*1e3  # mm
    Lc  = np.array([[float(x[87+i]) for i in range(6)] for x in rows])
    return t, err, Lc

def one_fig(name, fn, forced, orth, out, cs, ce):
    if not os.path.exists(fn):
        print(f'  [missing] {fn} — 跳过'); return
    t, err, Lc = load_series(fn)
    coll = (t > cs) & (t <= ce)
    lc = Lc[coll].mean(0)
    fmean = np.abs(err[coll][:, forced]).mean()
    o_pre  = np.abs(err[(t>4)&(t<cs), orth]).mean()
    o_coll = np.abs(err[coll, orth]).mean()
    verdict = 'PRESERVED (decoupled)' if o_coll < 0.25*fmean else 'residual Jacobian coupling'
    print(f'力@{name}: 点亮={[CART[i] for i in range(6) if lc[i]>LM*0.3]}  '
          f'正交{CART[orth]} pre={o_pre:.1f}→coll={o_coll:.1f}mm ({o_coll/max(fmean,1e-9)*100:.0f}%) → {verdict}')

    fig = plt.figure(figsize=(13,4.8))
    gs = fig.add_gridspec(1,2, width_ratios=[1,1.35], wspace=0.28)
    fig.suptitle(f'Oblique 45° collision in {name} plane — damping opens only on '
                 f'{CART[forced[0]]} & {CART[forced[1]]}, orthogonal {CART[orth]} stays at base',
                 fontweight='bold', fontsize=12)

    # ── 左: 1×6 阻尼热力图 ──
    ax = fig.add_subplot(gs[0,0])
    im = ax.imshow(lc.reshape(1,6), norm=LogNorm(vmin=LB*0.5, vmax=LM*1.2),
                   cmap='inferno', aspect='auto')
    ax.set_xticks(range(6)); ax.set_xticklabels(CART)
    ax.set_yticks([0]); ax.set_yticklabels([f'force @ {name}'])
    ax.set_title('(a) collision-avg damping $\\Lambda_{ii}$\nforced axes lit, orthogonal stays stiff')
    for j in range(6):
        ax.text(j,0,f'{lc[j]:.0e}',ha='center',va='center',fontsize=8,
                color='black' if lc[j]>LM*0.3 else 'white')
    for fi in forced:   # 受力轴红框
        ax.add_patch(plt.Rectangle((fi-0.5,-0.5),1,1,fill=False,ec='red',lw=2.5))
    ax.add_patch(plt.Rectangle((orth-0.5,-0.5),1,1,fill=False,ec='cyan',lw=2.5))  # 正交轴青框
    ax.set_ylim(0.5,-0.5)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label=r'$\Lambda_{ii}$')

    # ── 右: X/Y/Z 跟踪误差曲线 ──
    ax2 = fig.add_subplot(gs[0,1])
    for k in range(3):
        is_orth = (k == orth)
        ax2.plot(t, np.abs(err[:,k]), color=COL[TRA[k]],
                 lw=2.4 if is_orth else 1.3, alpha=1.0 if is_orth else 0.7,
                 label=f'{TRA[k]}' + ('  ← orthogonal' if is_orth else '  (forced)'))
    ax2.axvspan(cs, ce, color='orange', alpha=0.12, label='collision window')
    ax2.axhline(o_pre, ls=':', c=COL[TRA[orth]], lw=1, alpha=0.7)
    ax2.set_xlabel('time [s]'); ax2.set_ylabel('|tracking error| [mm]')
    ax2.set_title(f'(b) per-axis tracking error\n'
                  f'orthogonal {CART[orth]}: pre {o_pre:.1f} → coll {o_coll:.1f} mm '
                  f'({o_coll/max(fmean,1e-9)*100:.0f}% of forced ~{fmean:.0f} mm) → {verdict}')
    ax2.legend(fontsize=9, loc='upper left'); ax2.grid(alpha=0.3)
    ax2.set_xlim(t.min(), t.max())

    fig.subplots_adjust(top=0.84)
    p = f'{out}/fig_oblique_{name}'
    fig.savefig(p+'.png', dpi=130, bbox_inches='tight')
    fig.savefig(p+'.pdf', bbox_inches='tight')
    print('  saved →', p+'.png/pdf')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='/tmp/adls_oneday/exp_multidir')
    ap.add_argument('--output', default='/tmp/adls_oneday/exp_multidir/figs')
    ap.add_argument('--cs', type=float, default=8.0)
    ap.add_argument('--ce', type=float, default=10.0)
    a = ap.parse_args()
    os.makedirs(a.output, exist_ok=True)
    # name, file, forced idx, orthogonal idx
    one_fig('XY', f'{a.dir}/m4_XY.csv', [0,1], 2, a.output, a.cs, a.ce)
    one_fig('XZ', f'{a.dir}/m4_XZ.csv', [0,2], 1, a.output, a.cs, a.ce)

if __name__ == '__main__':
    main()
