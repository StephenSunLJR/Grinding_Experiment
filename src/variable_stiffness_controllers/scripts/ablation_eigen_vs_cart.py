#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实验 7: 特征方向阻尼(旧 v5.x) vs 笛卡尔基阻尼(v6.2) 消融

核心论点: 旧方案把阻尼定义在 SVD 特征方向上 (Λ_cart = U·diag(λ²_k)·U^T),
U 的列不与笛卡尔轴对齐 → 受力方向(Z)的高阻尼经 U…U^T 旋转**泄漏**进健康轴(X),
即非对角项 + X 对角项被污染抬高 → errX 溃散。
v6.2 把阻尼直接定义在笛卡尔基 (Λ_cart = diag(λ²_i)) → 纯对角、零泄漏。

本脚本为**离线解析**对比: 用真实 Franka(modified DH, 与控制器 kinematics_utils.hpp 一致)
在圆轨迹跟踪初始构型下的雅可比 J, 对同一个 Z 向 25N 末端碰撞, 分别按两套算法
算出阻尼矩阵 Λ_cart, 对比其结构(热图)与对健康轴 X 的污染。
不依赖 Gazebo / 不需重编译。
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

# ── 参数 (与 config/adls_experiment.yaml 一致) ──
LB   = 1e-4      # lambda2_base
LM   = 0.1       # lambda2_max
FTH  = 2.0       # F_th
BW   = 10.0      # beta_w
BC   = 10.0      # beta_c
W0   = 1.0
C0   = 1.0
RN   = np.array([0.5,0.5,0.5,0.5,0.3,0.3,0.3])  # r_noise
EPS  = 1e-6      # alpha_eps

CART = ['X','Y','Z','RX','RY','RZ']

def sigmoid(x): return 1.0/(1.0+np.exp(-x))

# ── Franka Panda modified-DH 几何雅可比 (与 kinematics_utils.hpp 完全一致) ──
# a, alpha, d  (theta = q_i)；末端含固定法兰 H8 (0,0,0.107)
DH = [(0.0,      0.0,      0.333),
      (0.0,     -np.pi/2,  0.0  ),
      (0.0,      np.pi/2,  0.316),
      (0.0825,   np.pi/2,  0.0  ),
      (-0.0825, -np.pi/2,  0.384),
      (0.0,      np.pi/2,  0.0  ),
      (0.088,    np.pi/2,  0.0  )]
H_FLANGE = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.107],[0,0,0,1]])

def dh_modified(a, alpha, d, theta):
    ct,st,ca,sa = np.cos(theta),np.sin(theta),np.cos(alpha),np.sin(alpha)
    return np.array([[ct,    -st,    0,    a     ],
                     [st*ca,  ct*ca,-sa,  -d*sa  ],
                     [st*sa,  ct*sa, ca,   d*ca  ],
                     [0,      0,     0,    1     ]])

def jacobian(q):
    """几何雅可比 6x7 (世界系, 末端含法兰)。"""
    T = np.eye(4)
    origins = [T[:3,3].copy()]   # o_0
    zaxes   = [T[:3,2].copy()]   # z_0
    for i,(a,al,d) in enumerate(DH):
        T = T @ dh_modified(a,al,d,q[i])
        origins.append(T[:3,3].copy())
        zaxes.append(T[:3,2].copy())
    T = T @ H_FLANGE
    o_e = T[:3,3]
    J = np.zeros((6,7))
    for i in range(7):
        z = zaxes[i]; o = origins[i]
        J[:3,i] = np.cross(z, o_e - o)   # 线速度
        J[3:,i] = z                       # 角速度
    return J

# ════════════════════════════════════════════════════════════
# 两套阻尼算法
# ════════════════════════════════════════════════════════════
def damping_v62(F_ext):
    """v6.2 笛卡尔基: λ²_i 由 |F_i| 驱动, Λ = diag(λ²)。"""
    lam = np.zeros(6)
    for i in range(6):
        cen = abs(F_ext[i]) - FTH
        wi  = W0*(1-sigmoid(BW*cen)) + 1e-6
        ci  = C0*sigmoid(BC*cen) + 1e-6
        lam[i] = (wi*LB + ci*LM)/(wi+ci)
    return np.diag(lam), lam

def damping_v5x(J, r_tilde):
    """旧 v5.x 特征方向: α_k 投影到 SVD 方向, λ²_k 驱动后 Λ = U·diag(λ²_k)·U^T。"""
    U,s,Vt = np.linalg.svd(J, full_matrices=False)   # J(6x7): U(6x6) s(6) Vt(6x7)
    V1 = Vt.T                                          # (7x6) 右奇异向量
    lam = np.zeros(6)
    for k in range(6):
        co = s[k]/(s[k]**2 + LB)
        a_raw = co * V1[:,k].dot(r_tilde)              # Eq 3.6
        a_th  = co * np.sum(np.abs(V1[:,k])*RN) + EPS   # Eq 3.7a
        a = (a_raw - np.sign(a_raw)*a_th) if abs(a_raw) >= a_th else 0.0  # 死区
        cen = abs(a) - a_th
        wi  = W0*(1-sigmoid(BW*cen)) + 1e-6
        ci  = C0*sigmoid(BC*cen) + 1e-6
        lam[k] = (wi*LB + ci*LM)/(wi+ci)
    Lc = U @ np.diag(lam) @ U.T                        # ← 几何纠缠在此发生
    return Lc, lam, U

# ════════════════════════════════════════════════════════════
def main():
    import os, argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default='/tmp/adls_oneday/exp7/figs')
    out_dir = ap.parse_args().output; os.makedirs(out_dir, exist_ok=True)

    # 圆轨迹跟踪初始构型 (与 Gazebo spawn 一致)
    q = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4])
    J = jacobian(q)

    # 末端 Z 向 -25N 碰撞 → 关节残差 r̃ = J^T F, 笛卡尔力经 DLS 反推 F_ext
    F_world = np.array([0,0,-25.0,0,0,0])
    r_tilde = J.T @ F_world
    F_ext = np.linalg.solve(J@J.T + LB*np.eye(6), J@r_tilde)   # 与控制器 STEP 3b 一致

    Lc62, lam62        = damping_v62(F_ext)
    Lc5x, lam5x, U     = damping_v5x(J, r_tilde)

    print('F_ext (recovered):', np.round(F_ext,2))
    print('v6.2 λ²_diag     :', np.round(lam62,5))
    print('v5.x λ²_k (SVD)  :', np.round(lam5x,5))
    print('v5.x Λ_cart 对角  :', np.round(np.diag(Lc5x),5))
    # 泄漏定量: 非受力轴(X,Y,RX,RY,RZ)上被抬高的阻尼
    leak5x = np.diag(Lc5x).copy(); leak5x[2] = 0     # 去掉 Z(受力轴)
    offdiag = np.abs(Lc5x - np.diag(np.diag(Lc5x))).max()
    print(f'v5.x 健康轴泄漏 (对角, 去Z)峰值: {leak5x.max():.5f}  ({leak5x.max()/LB:.0f}× base)')
    print(f'v5.x 非对角项峰值: {offdiag:.5f}  ({offdiag/LB:.0f}× base)')

    # ── 绘图 ──
    fig = plt.figure(figsize=(15,5.2))
    gs = fig.add_gridspec(1,3, width_ratios=[1,1,1.15], wspace=0.32)
    fig.suptitle('Ablation: SVD-eigen-direction damping (old v5.x)  vs  Cartesian-basis damping (v6.2)\n'
                 'same end-effector Z-collision (-25 N), same Jacobian',
                 fontweight='bold', fontsize=12)

    vmin, vmax = LB*0.5, LM*1.5
    def heat(ax, M, title):
        im = ax.imshow(np.abs(M), norm=LogNorm(vmin=vmin, vmax=vmax), cmap='inferno')
        ax.set_xticks(range(6)); ax.set_xticklabels(CART, fontsize=9)
        ax.set_yticks(range(6)); ax.set_yticklabels(CART, fontsize=9)
        ax.set_title(title, fontsize=10)
        for i in range(6):
            for j in range(6):
                v = abs(M[i,j])
                if v > vmin*2:
                    ax.text(j,i,f'{v:.0e}' if v<0.01 else f'{v:.2f}',
                            ha='center',va='center',fontsize=6.5,
                            color='white' if v<LM*0.3 else 'black')
        return im

    a0 = fig.add_subplot(gs[0,0])
    heat(a0, Lc5x, '(a) v5.x  $\\Lambda=U\\,\\mathrm{diag}(\\lambda^2_k)\\,U^T$\nX, Z, RY all damped  → LEAKAGE')
    for d in [0,2,4]:   # X, Z, RY 对角被点亮
        a0.add_patch(plt.Rectangle((d-0.5,d-0.5),1,1, fill=False, ec='cyan', lw=2.2))

    a1 = fig.add_subplot(gs[0,1])
    im = heat(a1, Lc62, '(b) v6.2  $\\Lambda=\\mathrm{diag}(\\lambda^2_i)$\nonly Z damped  → NO LEAKAGE')
    a1.add_patch(plt.Rectangle((2-0.5,2-0.5),1,1, fill=False, ec='cyan', lw=2.2))
    cax = fig.add_axes([0.635,0.18,0.008,0.6]); fig.colorbar(im, cax=cax, label=r'$|\Lambda_{ij}|$')

    # (c) 六个笛卡尔轴对角阻尼 — 直接看"阻尼漏到哪些轴"
    a2 = fig.add_subplot(gs[0,2])
    x = np.arange(6); w=0.38
    a2.bar(x-w/2, np.diag(Lc5x), w, label='v5.x (eigen)', color='tab:red')
    a2.bar(x+w/2, np.diag(Lc62), w, label='v6.2 (Cartesian)', color='tab:green')
    a2.axhline(LB, ls=':', c='gray', label=r'$\lambda^2_{base}$ (stiff)')
    a2.axhline(LM, ls='--', c='k', lw=0.8, alpha=0.4, label=r'$\lambda^2_{max}$ (compliant)')
    a2.set_yscale('log'); a2.set_xticks(x); a2.set_xticklabels(CART, fontsize=9)
    a2.set_ylabel(r'diagonal damping $\Lambda_{ii}$'); a2.set_ylim(vmin, vmax)
    a2.set_xlabel('Cartesian axis')
    a2.set_title('(c) per-axis diagonal damping (Z-collision)\n'
                 'v5.x: X & RY leak to $\\lambda^2_{max}$ / v6.2: only Z', fontsize=10)
    a2.legend(fontsize=8, loc='center right')
    a2.text(0, np.diag(Lc5x)[0]*1.2, '1000×', ha='center', fontsize=8, color='tab:red', fontweight='bold')

    fig.subplots_adjust(top=0.80)
    fig.savefig(out_dir+'/fig_ablation_eigen_vs_cart.png', dpi=130, bbox_inches='tight')
    fig.savefig(out_dir+'/fig_ablation_eigen_vs_cart.pdf', bbox_inches='tight')
    print('\nsaved →', out_dir+'/fig_ablation_eigen_vs_cart.png/pdf')

if __name__ == '__main__':
    main()
