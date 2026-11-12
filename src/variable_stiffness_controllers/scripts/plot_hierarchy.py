#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务分级 + 零空间各向异性柔顺 + 无碰撞退化 验证图 (1.3 节融合控制律)

主任务: 末端跟踪 XY 圆;  次级任务(零空间): 关节构型驱向 q_pref (关节1/3偏置)。
扰动: t=8-10s 末端 Z 向 25N。对比 M4(各向异性 N) vs M3(标准 N=I-J⁺J)。

四子图:
 (a) 末端 X 跟踪误差 — 碰撞前≈0: 次级任务在动但主任务不受扰 → 任务分级成立;
     同时碰撞前 M4≡M3≡标准DLS → 无碰撞退化。
 (b) 末端 Z 跟踪误差 — M4 退让 / M3 刚性 → 各向异性优先级。
 (c) 次级任务代价 ‖q-q_pref‖ — 持续下降 → 次级任务在零空间完成。
 (d) ‖J·N‖ — M4 碰撞时由 0 跃起(零空间在受力方向打开), M3 恒 0。

CSV 列: 0=t, 20-25=err(6,m), 26=‖J·N‖, 87-92=Λ对角, 93=‖q-q_pref‖, 94=‖dq_null‖
"""
import csv, argparse, os
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load(fn):
    with open(fn) as f:
        r=csv.reader(f); next(r); rows=[x for x in r if len(x)>94]
    t=np.array([float(x[0]) for x in rows])
    err=np.array([[float(x[20+i]) for i in range(6)] for x in rows])*1e3  # mm
    JN=np.array([float(x[26]) for x in rows])
    qcost=np.array([float(x[93]) for x in rows])
    dqnull=np.array([float(x[94]) for x in rows])
    return t,err,JN,qcost,dqnull

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--m3', required=True); ap.add_argument('--m4', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--cs', type=float, default=8.0); ap.add_argument('--ce', type=float, default=10.0)
    a=ap.parse_args(); os.makedirs(a.output, exist_ok=True)

    d3=load(a.m3); d4=load(a.m4)
    t3,e3,jn3,qc3,dn3=d3; t4,e4,jn4,qc4,dn4=d4

    fig,ax=plt.subplots(2,2,figsize=(13,8))
    fig.suptitle('Task hierarchy + anisotropic null-space compliance + no-collision degradation (§1.3)',
                 fontweight='bold', fontsize=13)
    for a_ in ax.flat: a_.axvspan(a.cs,a.ce,color='orange',alpha=0.12)

    # (a) 末端 X 误差
    ax[0,0].plot(t3, np.abs(e3[:,0]), label='M3 (standard N)', color='tab:green', lw=1.5)
    ax[0,0].plot(t4, np.abs(e4[:,0]), label='M4 (anisotropic N)', color='tab:red', lw=1.5)
    pre = np.abs(e4[(t4>4)&(t4<a.cs),0]).mean()
    ax[0,0].set_title(f'(a) main-task X error — pre-collision ≈{pre:.1f} mm\n'
                      'secondary task active yet main task undisturbed → hierarchy holds\n'
                      '(pre-collision M4≡standard DLS → no-collision degradation)', fontsize=9)
    ax[0,0].set_ylabel('|X error| [mm]'); ax[0,0].legend(fontsize=8); ax[0,0].grid(alpha=0.3)

    # (b) 末端 Z 误差
    ax[0,1].plot(t3, np.abs(e3[:,2]), label='M3 (rigid)', color='tab:green', lw=1.5)
    ax[0,1].plot(t4, np.abs(e4[:,2]), label='M4 (yields)', color='tab:red', lw=1.5)
    ax[0,1].set_title('(b) collision Z error — M4 yields / M3 rigid\n→ anisotropic task priority', fontsize=9)
    ax[0,1].set_ylabel('|Z error| [mm]'); ax[0,1].legend(fontsize=8); ax[0,1].grid(alpha=0.3)

    # (c) 次级任务代价
    ax[1,0].plot(t3, qc3, label='M3', color='tab:green', lw=1.5)
    ax[1,0].plot(t4, qc4, label='M4', color='tab:red', lw=1.5)
    q0=qc4[t4<2].mean() if (t4<2).any() else qc4[0]
    qmin=qc4[(t4>2)&(t4<a.cs)].min() if ((t4>2)&(t4<a.cs)).any() else qc4.min()
    trend='↓ converging' if qmin < q0-0.02 else 'limited by 1-DOF null space'
    ax[1,0].set_title(f'(c) secondary cost ‖q − q_pref‖  (init {q0:.2f} → min {qmin:.2f}, {trend})\n'
                      'null-space posture task drives q → q_pref', fontsize=9)
    ax[1,0].set_xlabel('time [s]'); ax[1,0].set_ylabel('‖q − q_pref‖ [rad]'); ax[1,0].legend(fontsize=8); ax[1,0].grid(alpha=0.3)

    # (d) ‖J·N‖
    ax[1,1].plot(t3, jn3, label='M3 (N=I−J⁺J, ≡0)', color='tab:green', lw=1.5)
    ax[1,1].plot(t4, jn4, label='M4 (anisotropic)', color='tab:red', lw=1.8)
    jn_pk = jn4[(t4>a.cs)&(t4<=a.ce)].max()
    ax[1,1].set_title(f'(d) ‖J·N‖ — M4 jumps 0→{jn_pk:.2f} at collision\n'
                      '(null space opens in forced direction), M3 stays 0', fontsize=9)
    ax[1,1].set_xlabel('time [s]'); ax[1,1].set_ylabel('‖J·N‖'); ax[1,1].legend(fontsize=8); ax[1,1].grid(alpha=0.3)

    fig.tight_layout(rect=[0,0,1,0.95])
    fig.savefig(a.output+'/fig_hierarchy.png', dpi=130, bbox_inches='tight')
    fig.savefig(a.output+'/fig_hierarchy.pdf', bbox_inches='tight')
    print('saved →', a.output+'/fig_hierarchy.png/pdf')
    print(f'(a) M4 碰撞前X误差={pre:.1f}mm  (d) M4 ‖J·N‖峰值={jn_pk:.2f}, M3={jn3.max():.3f}')

if __name__=='__main__':
    main()
