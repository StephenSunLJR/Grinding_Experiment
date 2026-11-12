#pragma once
/**
 * @file adls_damping.h
 * @brief Anisotropic DLS Damping Matrix Λ(r) — core theoretical innovation (§4).
 *
 * Replaces scalar λ²I in classical DLS with a diagonal matrix driven by
 * collision projection α — enabling direction-selective compliance.
 *
 * Pipeline:
 *   Step 1: w_k ↓, c_k ↑ ← Sigmoid(|α_k|)          (Eq 4.12–4.13)
 *   Step 2: λ²*_k = (w_k·λ²_base + c_k·λ²_max)/(w_k+c_k)  (Eq 4.9)
 *            → closed-form solution to convex optimization (Eq 4.8)
 *   Step 3: Λ_raw = diag(λ²₁,...,λ²₆) in U-coordinates
 *   Step 4: Λ_cart_raw = U Λ_raw Uᵀ                  (transform to Cartesian)
 *   Step 5: Asymmetric EMA: γ_attack >> γ_decay       (Eq 4.19–4.19a)
 *
 * Properties (§4.4):
 *   P1: λ²_base ≤ λ²_k ≤ λ²_max  (boundedness via convex combination)
 *   P2: Lipschitz continuous in α_k
 *   P3: Direction-selective: collision direction → λ²_max, healthy → λ²_base
 *   P4: dΛ/dt bounded (guarantees stability of time-varying damping)
 */

#include "types.h"
#include <cmath>
#include <algorithm>

// C++14 compatible clamp
namespace adls { namespace detail {
template<typename T> T clamp_val(T v, T lo, T hi) {
    return std::max(lo, std::min(v, hi));
}
}}  // namespace adls::detail

namespace adls {

inline double sigmoid(double x) {
    // Clamped to avoid overflow
    return 1.0 / (1.0 + std::exp(-detail::clamp_val(x, -50.0, 50.0)));
}

class ADLSDamping {
public:
    ADLSDamping(const ADLSParams& p)
        : lambda2_base_(p.lambda2_base), lambda2_max_(p.lambda2_max),
          w0_(p.w0), c0_(p.c0),
          beta_w_(p.beta_w), beta_c_(p.beta_c),
          gamma_attack_(p.gamma_attack), gamma_decay_(p.gamma_decay) {}

    void reset() {
        Lambda_cart_prev_.setZero();
        initialized_ = false;
    }

    /**
     * Compute A-DLS damping matrix in Cartesian space (EMA-filtered).
     * @param alpha     task-space collision projection (6×1), after deadzone
     * @param alpha_th  per-direction threshold from SVDProjector (6×1)
     * @param U         SVD left singular vectors (6×6)
     * @return Λ_cart   Cartesian damping matrix (6×6, SPD, filtered)
     */
    Matrix6d compute(const Vector6d& alpha, const Vector6d& alpha_th,
                     const Matrix6d& U) {
        // ── Step 1: Sigmoid-based weights (Paper Eq 4.12–4.13) ──
        //
        // CRITICAL: sigmoid input must be β·(|α| - α_th), NOT β·|α|.
        // Without the α_th offset, sigmoid(0) = 0.5 means 50% damping is
        // applied even when |α| = 0 (no collision), breaking CLIK tracking.
        //
        // With α_th offset:
        //   |α| = 0       → sigmoid(-β·α_th) ≈ 0 → w=w0, c≈0 → λ²=λ_base (NO damping)
        //   |α| = α_th    → sigmoid(0) = 0.5 → 50% transition
        //   |α| ≫ α_th    → sigmoid → 1 → w≈0, c=c0 → λ²=λ_max (FULL damping)
        Vector6d w_k, c_k;
        for (int k = 0; k < M_TASK; ++k) {
            double a = std::abs(alpha[k]);
            double centered = a - alpha_th[k];   // sigmoid centered at threshold
            w_k[k] = w0_ * (1.0 - sigmoid(beta_w_ * centered)) + 1e-6;
            c_k[k] = c0_ * sigmoid(beta_c_ * centered) + 1e-6;
        }
        debug_w_ = w_k;
        debug_c_ = c_k;

        // ── Step 2: Closed-form optimal damping (Eq 4.9) ──
        // λ²*_k = (w_k·λ²_base + c_k·λ²_max) / (w_k + c_k)
        // Natural convex combination → auto-bounded in [λ²_base, λ²_max]
        Vector6d lambda2_raw;
        lambda2_raw = (w_k.array() * lambda2_base_ + c_k.array() * lambda2_max_)
                      / (w_k.array() + c_k.array());
        debug_lambda2_raw_ = lambda2_raw;

        // ── Step 3–4: Λ_cart_raw = U diag(λ²_k) Uᵀ (Eq 4.19) ──
        Matrix6d Lambda_cart_raw = U * lambda2_raw.asDiagonal() * U.transpose();

        // ── Step 5: Asymmetric EMA filter (Eq 4.19–4.19a) ──
        if (!initialized_) {
            Lambda_cart_prev_ = Lambda_cart_raw;
            initialized_ = true;
            return Lambda_cart_raw;
        }

        Matrix6d Lambda_cart;
        for (int i = 0; i < M_TASK; ++i) {
            for (int j = 0; j < M_TASK; ++j) {
                double gamma_ij = (Lambda_cart_raw(i, j) > Lambda_cart_prev_(i, j))
                                  ? gamma_attack_ : gamma_decay_;
                Lambda_cart(i, j) = (1.0 - gamma_ij) * Lambda_cart_prev_(i, j)
                                    + gamma_ij * Lambda_cart_raw(i, j);
            }
        }

        Lambda_cart_prev_ = Lambda_cart;
        return Lambda_cart;
    }

    /// Collision participation vector β (Eq 6.1a).
    Vector6d getBeta() const {
        double denom = lambda2_max_ - lambda2_base_;
        Vector6d beta;
        for (int i = 0; i < M_TASK; ++i) {
            double val = (Lambda_cart_prev_(i, i) - lambda2_base_) / denom;
            beta[i] = detail::clamp_val(val, 0.0, 1.0);
        }
        return beta;
    }

    // Diagnostic accessors
    Vector6d debug_lambda2_raw() const { return debug_lambda2_raw_; }
    Vector6d debug_w() const { return debug_w_; }
    Vector6d debug_c() const { return debug_c_; }

private:
    double lambda2_base_, lambda2_max_;
    double w0_, c0_;
    double beta_w_, beta_c_;
    double gamma_attack_, gamma_decay_;

    Matrix6d Lambda_cart_prev_{Matrix6d::Zero()};
    bool initialized_ = false;

    // Diagnostics
    Vector6d debug_lambda2_raw_{Vector6d::Zero()};
    Vector6d debug_w_{Vector6d::Zero()};
    Vector6d debug_c_{Vector6d::Zero()};
};

}  // namespace adls
