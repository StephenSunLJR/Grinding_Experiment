#pragma once
/**
 * @file nullspace_compliance.h
 * @brief Null-space local compliance for joint-level collision response (§6.2).
 *
 * Eq (6.2):  r_dz,j = deadzone(r_j, r_noise,j)
 * Eq (6.2a): q̇_comp,raw = K_n⁻¹ r_dz
 * Eq (6.2b): q̇_comp = (1-γ_n) q̇_comp_prev + γ_n q̇_comp,raw  (EMA smoothed)
 *
 * Design rationale: Deadzone is C⁰ (derivative discontinuity at threshold).
 * EMA eliminates torque pulses from numerical differentiation (×1/Δt).
 */

#include "types.h"
#include <cmath>

namespace adls {

class NullSpaceCompliance {
public:
    NullSpaceCompliance(const ADLSParams& p)
        : K_n_inv_(p.K_n.cwiseInverse().asDiagonal()),
          r_noise_(p.r_noise_nullspace),   // use HIGH threshold for nullspace
          gamma_n_(p.gamma_n) {}

    void reset() {
        q_comp_prev_.setZero();
        initialized_ = false;
    }

    /**
     * Compute null-space compliance velocity.
     * @param r  raw GMO residual (before FDI masking — per-joint response)
     */
    Vector7d compute(const Vector7d& r) {
        // Eq (6.2): element-wise deadzone
        Vector7d r_dz = Vector7d::Zero();
        for (int j = 0; j < N_JOINTS; ++j) {
            if (std::abs(r[j]) >= r_noise_[j]) {
                r_dz[j] = r[j] - std::copysign(r_noise_[j], r[j]);
            }
        }

        // Eq (6.2a): raw compliance velocity
        Vector7d q_comp_raw = K_n_inv_ * r_dz;

        // Safety cap: limit per-joint nullspace velocity (Franka spec: 2.175 rad/s)
        // Prevents runaway when GMO residual spikes from numerical / model issues.
        constexpr double Q_COMP_MAX = 0.3;  // rad/s — conservative
        for (int j = 0; j < N_JOINTS; ++j) {
            q_comp_raw[j] = std::max(-Q_COMP_MAX, std::min(Q_COMP_MAX, q_comp_raw[j]));
        }

        // Eq (6.2b): EMA filtering
        if (!initialized_) {
            q_comp_prev_ = q_comp_raw;
            initialized_ = true;
            return q_comp_raw;
        }

        Vector7d q_comp = (1.0 - gamma_n_) * q_comp_prev_ + gamma_n_ * q_comp_raw;
        q_comp_prev_ = q_comp;
        return q_comp;
    }

private:
    Matrix7d K_n_inv_;
    Vector7d r_noise_;
    double gamma_n_;
    Vector7d q_comp_prev_{Vector7d::Zero()};
    bool initialized_ = false;
};

}  // namespace adls
