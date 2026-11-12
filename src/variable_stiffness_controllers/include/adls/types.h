#pragma once
/**
 * @file types.h
 * @brief Common type definitions and parameter structures for A-DLS controller.
 *
 * Paper mapping:
 *   n = 7 DOF, m = 6 task-space dimensions
 *   q ∈ R⁷, dq ∈ R⁷, τ ∈ R⁷
 *   J ∈ R^{6×7}, U ∈ R^{6×6}, S = diag(σ₁...σ₆), V₁ ∈ R^{7×6}
 *   r ∈ R⁷ (GMO residual), α ∈ R⁶ (collision projection)
 *   Λ ∈ R^{6×6} (anisotropic damping matrix)
 */

#include <Eigen/Dense>
#include <array>

namespace adls {

constexpr int N_JOINTS = 7;
constexpr int M_TASK = 6;

// ── Common Eigen type aliases ──
using Vector7d = Eigen::Matrix<double, N_JOINTS, 1>;
using Vector6d = Eigen::Matrix<double, M_TASK, 1>;
using Matrix7d = Eigen::Matrix<double, N_JOINTS, N_JOINTS>;
using Matrix6d = Eigen::Matrix<double, M_TASK, M_TASK>;
using Matrix67d = Eigen::Matrix<double, M_TASK, N_JOINTS>;
using Matrix76d = Eigen::Matrix<double, N_JOINTS, M_TASK>;
using Diag6d = Eigen::DiagonalMatrix<double, M_TASK>;
using Diag7d = Eigen::DiagonalMatrix<double, N_JOINTS>;

/**
 * @brief Global A-DLS controller parameters.
 *
 * All values match the experiment design document (Public Settings table).
 */
struct ADLSParams {
    // ── GMO Observer gains (§2 Eq 2.9) ──
    Vector7d K_O{{40.0, 40.0, 40.0, 30.0, 25.0, 20.0, 20.0}};

    // ── FDI thresholds (§3.1 Eq 3.2) ──
    Vector7d tau_th_base{{0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3}};
    double kappa = 0.1;
    double epsilon_fdi = 0.02;

    // ── SVD投影死区阈值 (§3.3 Eq 3.7a) ──
    // LOW threshold: lets collision signals pass to A-DLS.
    // Adaptive baseline (τ=0.5s) absorbs slow model mismatch; remaining
    // fast residual is ~0.3-2.6 Nm. r_noise≈0.5 blocks false triggers
    // from the residual floor (~2.6→α=0.8<α_th=1.05) while passing
    // collision signals (25N→α=2.5>α_th). Real Franka: 0.06-0.20.
    Vector7d r_noise{{0.5, 0.5, 0.5, 0.5, 0.3, 0.3, 0.3}};

    // ── 零空间柔顺死区阈值 (§6.2 Eq 6.2) ──
    // HIGH threshold: per-joint deadzone for nullspace compliance.
    // Must be high enough to block ALL residual during normal motion
    // (otherwise q̇_comp = r/K_n drives unwanted nullspace drift).
    // Real Franka: 0.06-0.20. Gazebo: 5.0 to block 2-5 Nm residual.
    Vector7d r_noise_nullspace{{5.0, 5.0, 5.0, 5.0, 3.0, 3.0, 3.0}};

    // ── DLS damping bounds (§4.2.3) ──
    // lambda2_max=0.5 causes cascade in Gazebo: SVD direction 4 (sigma≈0.3)
    // is pitch-dominant, so lambda2=0.5 → 85% CLIK gain drop → pitch
    // compliance → arm rotates → r_corr grows → all dirs activate → runaway.
    // 0.1 gives 53% max gain reduction (direction 4): moderate Z/pitch
    // compliance without cascade. Real Franka (smaller model mismatch) would
    // tolerate 0.3–0.5.
    double lambda2_base = 1e-4;
    double lambda2_max = 0.1;

    // ── Weight function params (§4.2.3 Eq 4.12–4.13) ──
    double w0 = 1.0;
    double c0 = 1.0;
    double beta_w = 10.0;
    double beta_c = 10.0;

    // ── Asymmetric EMA (§4.5 Eq 4.19a) ──
    double gamma_attack = 0.5;   // ~2 ms at 1 kHz
    double gamma_decay = 0.02;   // ~50 ms at 1 kHz

    // ── Null-space EMA (§6.2 Eq 6.2b) ──
    double gamma_n = 0.05;       // ~20 ms

    // ── Acceleration EMA (§6.3) ──
    // For velocity-error-based ddq (not numerical diff), light filtering is enough.
    // gamma_a = 0.5 → ~2ms time constant, near-direct response, minimal lag.
    double gamma_a = 0.5;

    // ── CLIK P-gain (§6.1 Eq 6.1c) ──
    // Increased back toward paper's nominal value now that nullspace compliance
    // is disabled in static-hold mode (no more spurious residual feeding back).
    // Higher K_P → tighter position tracking, less steady-state drift.
    Vector6d K_P{{30.0, 30.0, 30.0, 15.0, 15.0, 15.0}};

    // ── Null-space compliance stiffness (§6.2 Eq 6.2a) ──
    // NOTE: Stiffer than paper's nominal value to remain stable under Gazebo
    // model-mismatch GMO residuals. q̇_comp = K_n⁻¹ · r_dz, so larger K_n
    // → smaller compliance velocity per N·m of residual.
    Vector7d K_n{{5.0, 5.0, 5.0, 5.0, 4.0, 3.0, 3.0}};

    // ── Joint PD gains for CTC (§6.3) ──
    Vector7d K_jP{{80.0, 80.0, 80.0, 60.0, 40.0, 30.0, 20.0}};
    Vector7d K_jD{{8.0, 8.0, 8.0, 6.0, 4.0, 3.0, 2.0}};

    // ── Numerical safeguards ──
    double alpha_th_epsilon = 1e-6;

    // ── Torque limits (Franka Panda spec) ──
    Vector7d tau_max{{30.0, 30.0, 30.0, 30.0, 8.0, 8.0, 8.0}};
    double delta_tau_max = 10.0;   // [N·m/ms] — relaxed from 1.0 so gravity comp doesn't lag

    /// Create params for a specific comparison method (M1–M4).
    static ADLSParams forMethod(const std::string& method) {
        ADLSParams p;
        if (method == "M1") {  // Strict priority (Moore-Penrose, no nullspace)
            p.lambda2_max = 1e-4;
            p.K_n.setZero();
        } else if (method == "M2") {  // Scalar DLS — constant lambda2=0.5 everywhere
            p.lambda2_base = 0.5;
            p.lambda2_max = 0.5;   // must match base so sigmoid has no effect
        } else if (method == "M3") {  // Null-space only
            p.lambda2_max = 1e-4;
        }
        // M4 uses defaults
        return p;
    }
};

}  // namespace adls
