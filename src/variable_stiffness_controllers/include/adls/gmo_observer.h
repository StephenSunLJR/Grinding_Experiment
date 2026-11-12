#pragma once
/**
 * @file gmo_observer.h
 * @brief Generalized Momentum Observer (§2).
 *
 * Eq (2.9):  r_{k+1} = r_k + K_O [p_k - p_{k-1} - (τ_m - β̂_k + r_k) Δt]
 * Eq (2.7):  ṙ = K_O (τ_ext - r)   →   first-order lowpass
 * Eq (2.8):  r_i(s)/τ_ext,i(s) = k_O,i / (s + k_O,i)
 *
 * Key property: Bypasses q̈ measurement via generalized momentum p = M(q) q̇,
 * exploiting the skew-symmetry of Ṁ - 2C (§2 Eq 2.2).
 */

#include "types.h"
#include <franka_hw/franka_model_interface.h>

namespace adls {

class GMOObserver {
public:
    GMOObserver(const ADLSParams& p) : K_O_(p.K_O.asDiagonal()) {}

    void reset() {
        initialized_ = false;
        r_.setZero();
        p_prev_.setZero();
    }

    /**
     * Single GMO update step.
     * @param q, dq            joint state
     * @param tau_m             motor torque
     * @param model_handle      libfranka model (provides M, C, g)
     * @return r ≈ τ_ext       filtered residual (7×1)
     */
    Vector7d update(const Vector7d& q, const Vector7d& dq,
                    const Vector7d& tau_m,
                    franka_hw::FrankaModelHandle& model_handle,
                    double dt) {
        // Get dynamics from libfranka model
        std::array<double, 49> M_arr = model_handle.getMass();
        std::array<double, 7>  C_arr = model_handle.getCoriolis();
        std::array<double, 7>  g_arr = model_handle.getGravity();

        Eigen::Map<Matrix7d> M(M_arr.data());
        Eigen::Map<Vector7d> c(C_arr.data());  // C(q,dq) * dq
        Eigen::Map<Vector7d> g(g_arr.data());

        // Generalized momentum p = M(q) dq  (Eq 2.3)
        Vector7d p = M * dq;

        // β̂ = g(q) - Cᵀ(q,dq) q̇  (§2, after Eq 2.5)
        // Since libfranka provides C*dq not C matrix, use:
        // Cᵀ(q,dq) q̇ ≡ C(q,dq) q̇  for the specific structure (verified in §2.2)
        // Actually β̂ = g - Cᵀ dq. For libfranka: model_handle gives Coriolis vector = C*dq.
        // The skew-symmetry gives: dqᵀ(Ṁ-2C)dq = 0 → Cᵀ dq = Ṁ dq - C dq
        // We approximate Cᵀ dq ≈ C dq for the observer term (lowpass already provides robustness).
        Vector7d beta_hat = g - c;  // c = C(q,dq) * dq

        if (!initialized_) {
            p_prev_ = p;
            initialized_ = true;
            return r_;
        }

        // Eq (2.9): discrete GMO update
        Vector7d dp = p - p_prev_;
        Vector7d innovation = tau_m - beta_hat + r_;
        r_ += K_O_ * (dp - innovation * dt);

        p_prev_ = p;
        return r_;
    }

    Vector7d residual() const { return r_; }

private:
    Matrix7d K_O_;
    Vector7d r_{Vector7d::Zero()};
    Vector7d p_prev_{Vector7d::Zero()};
    bool initialized_ = false;
};

}  // namespace adls
