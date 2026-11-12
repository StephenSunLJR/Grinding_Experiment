#!/usr/bin/env python3
"""
A-DLS 实验可视化通用工具
=========================

定义 /adls/diagnostics 话题的字段布局，并提供通用绘图函数。
字段布局必须与 A_DLS.cpp 中发布的 Float64MultiArray 一致。
"""

import os
import numpy as np
# NOTE: matplotlib backend NOT set here — let each script decide:
#   - analyze_exp{1,2a,3b}.py use 'Agg' for headless PDF generation
#   - live_dashboard.py uses 'TkAgg' for interactive window
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


# ============================================================================
# 诊断字段布局（与 A_DLS.cpp 必须一致）
# ============================================================================
DIAG_FIELDS = {
    't':              slice(0, 1),
    'alpha':          slice(1, 7),       # 6D, task-space collision projection
    'lambda2':        slice(7, 13),      # 6D, per-direction A-DLS damping
    'r_corr':         slice(13, 20),     # 7D, GMO residual (after baseline correction)
    'error':          slice(20, 26),     # 6D, Cartesian error (target - actual)
    'JN_norm':        slice(26, 27),     # 1D, nullspace leakage
    'sigma':          slice(27, 33),     # 6D, Jacobian singular values
    'position':       slice(33, 36),     # 3D, current EE position
    'position_d':     slice(36, 39),     # 3D, target EE position
    'dx_d':           slice(39, 45),     # 6D, feedforward velocity
    'dq':             slice(45, 52),     # 7D, actual joint velocities
    'dq_cmd':         slice(52, 59),     # 7D, commanded joint velocities
    'tau_cmd':        slice(59, 66),     # 7D, commanded joint torques
    'r_raw':          slice(66, 73),     # 7D, raw GMO residual (before baseline)
    'F_ext_applied':  slice(73, 79),     # 6D, injected external force
    'k_star':         slice(79, 80),     # 1D, FDI link index (0-based)
    'method':         slice(80, 81),     # 1D, method id (1=M1, 2=M2, 3=M3, 4=M4)
    'alpha_raw':      slice(81, 87),     # 6D, alpha before deadzone
    'Lambda_cart':    slice(87, 93),     # 6D, Cartesian Λ diagonal
}

METHOD_NAMES = {1: 'M1 (Moore-Penrose)', 2: 'M2 (Scalar DLS)',
                3: 'M3 (Null-space only)', 4: 'M4 (A-DLS)'}
METHOD_COLORS = {1: '#1f77b4', 2: '#ff7f0e', 3: '#2ca02c', 4: '#d62728'}
METHOD_SHORTS = {1: 'M1', 2: 'M2', 3: 'M3', 4: 'M4'}

# NOTE: λ²_k corresponds to the k-th SVD principal direction of the Jacobian,
# NOT the Cartesian X/Y/Z axes. These directions are abstract task-space basis
# vectors (columns of U in J = U·S·V^T), ordered by descending singular value.
# So λ²_1 = "largest-σ direction (most controllable)", λ²_6 = "smallest-σ
# direction (near-singular)".
AXIS_LABELS = ['dir 1 (σ_max)', 'dir 2', 'dir 3', 'dir 4', 'dir 5', 'dir 6 (σ_min)']


# ============================================================================
# 数据加载与字段提取
# ============================================================================
def load_csv(csv_path: str) -> np.ndarray:
    """加载 record_diagnostics.py 生成的 CSV 文件。"""
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 93:
        raise ValueError(
            f"CSV has {data.shape[1]} columns but expected 93. "
            "Did you rebuild the controller after updating diagnostic publishing?"
        )
    return data


def extract(data: np.ndarray, field: str) -> np.ndarray:
    """根据字段名提取对应的列。返回 (N,) 或 (N, K) 数组。"""
    if field not in DIAG_FIELDS:
        raise KeyError(f"Unknown field: {field}. Available: {list(DIAG_FIELDS)}")
    sl = DIAG_FIELDS[field]
    col = data[:, sl]
    return col.squeeze()


# ============================================================================
# 通用绘图样式
# ============================================================================
def setup_plot_style():
    """统一绘图样式（论文用）。"""
    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 11,
        'axes.titlesize': 12,
        'axes.labelsize': 11,
        'legend.fontsize': 9,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'figure.figsize': (10, 6),
        'figure.dpi': 100,
        'savefig.dpi': 150,
        'savefig.bbox': 'tight',
        'lines.linewidth': 1.2,
        'axes.grid': True,
        'grid.alpha': 0.3,
    })


def shade_collision_region(ax, t_start: float, t_end: float, alpha: float = 0.15):
    """碰撞区间灰色背景标注（实验设计 §1.6 Fig 1）。"""
    ax.axvspan(t_start, t_end, color='gray', alpha=alpha, zorder=0,
               label='Collision interval')


def save_figure(fig, output_dir: str, name: str):
    """保存图片到指定目录，PDF + PNG 双格式。"""
    os.makedirs(output_dir, exist_ok=True)
    pdf_path = os.path.join(output_dir, f'{name}.pdf')
    png_path = os.path.join(output_dir, f'{name}.png')
    fig.savefig(pdf_path)
    fig.savefig(png_path)
    print(f'  → saved {pdf_path}')
    print(f'  → saved {png_path}')


# ============================================================================
# 计算指标（实验设计 §1.4 量化指标）
# ============================================================================
def compute_rmse(error_array: np.ndarray, t: np.ndarray,
                 t_start: float, t_end: float) -> np.ndarray:
    """在时间窗口 [t_start, t_end] 内计算逐维 RMSE。
    error_array: (N, 6) 笛卡尔误差
    返回: (6,) 各方向 RMSE
    """
    mask = (t >= t_start) & (t <= t_end)
    if mask.sum() == 0:
        return np.zeros(error_array.shape[1])
    return np.sqrt(np.mean(error_array[mask] ** 2, axis=0))


def compute_eta(rmse_per_axis: np.ndarray,
                collision_axis: int = 2) -> float:
    """方向选择性指数 η = mean(健康方向 RMSE) / 碰撞方向 RMSE。
    η << 1 = 优秀（健康方向几乎无影响）
    η ≈ 1 = 各向同性退化（坏）
    """
    healthy = np.array([i for i in range(3) if i != collision_axis])
    healthy_rmse = rmse_per_axis[healthy].mean()
    collision_rmse = rmse_per_axis[collision_axis]
    return healthy_rmse / (collision_rmse + 1e-9)


def compute_force_metrics(r_corr: np.ndarray, F_applied: np.ndarray,
                          t: np.ndarray,
                          t_start: float, t_end: float) -> dict:
    """计算碰撞期间的安全指标。"""
    mask = (t >= t_start) & (t <= t_end)
    r_during = np.linalg.norm(r_corr[mask], axis=1, ord=np.inf)
    return {
        'r_peak': r_during.max() if r_during.size else 0.0,
        'r_steady': r_during[len(r_during) // 2:].mean() if r_during.size else 0.0,
        'F_max': np.linalg.norm(F_applied[mask], axis=1).max() if mask.sum() else 0.0,
    }


# ============================================================================
# 入口函数自检
# ============================================================================
if __name__ == '__main__':
    print('A-DLS Plot Utilities')
    print(f'Total fields: 93')
    for name, sl in DIAG_FIELDS.items():
        print(f'  [{sl.start:2d}-{sl.stop-1:2d}] {name}')
