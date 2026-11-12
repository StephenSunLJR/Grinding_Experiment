#pragma once
/**
 * @file fdi_isolator.h
 * @brief Fault Detection & Isolation — collision link identification (§3.1).
 *
 * Proposition 2 (De Luca & Mattone, 2005):
 *   Collision at link k → r_i ≠ 0 for i=1..k, r_j = 0 for j=k+1..n
 *
 * Eq (3.2):  k* = max{j : |r_j| > τ_th,j(t)}
 * Eq (3.2a): r̃ = r ⊙ m_aff   (masked residual)
 *
 * Dynamic threshold:
 *   τ_th,j(t) = τ_th,j^base + κ·|τ̂_F,j| + ε·|τ_m,j|
 */

#include "types.h"
#include <cmath>

namespace adls {

class FDIIsolator {
public:
    FDIIsolator(const ADLSParams& p)
        : tau_th_base_(p.tau_th_base),
          kappa_(p.kappa), epsilon_fdi_(p.epsilon_fdi) {}

    /**
     * FDI isolation step.
     * @param r       GMO residual (7×1)
     * @param tau_F   friction torque estimate (7×1)
     * @param tau_m   motor torque (7×1)
     * @return r_tilde   masked residual (healthy joints → 0)
     * @return m_aff     binary mask (1=affected, 0=healthy)
     * @return k_star    identified collision link (0-based, -1 = none)
     */
    struct FDIResult {
        Vector7d r_tilde;
        Vector7d m_aff;
        int k_star;
    };

    FDIResult isolate(const Vector7d& r, const Vector7d& tau_F,
                      const Vector7d& tau_m) {
        FDIResult res;
        res.r_tilde.setZero();
        res.m_aff.setZero();
        res.k_star = -1;

        // Dynamic thresholds (Eq 3.2)
        Vector7d tau_th = tau_th_base_
            + kappa_ * tau_F.cwiseAbs()
            + epsilon_fdi_ * tau_m.cwiseAbs();

        // Backward search from end-effector (Eq 3.2)
        for (int j = N_JOINTS - 1; j >= 0; --j) {
            if (std::abs(r[j]) > tau_th[j]) {
                res.k_star = j;
                break;
            }
        }

        // Generate mask (Eq 3.2a): affected = 1..k*
        if (res.k_star >= 0) {
            for (int j = 0; j <= res.k_star; ++j) {
                res.m_aff[j] = 1.0;
            }
        }

        res.r_tilde = r.cwiseProduct(res.m_aff);
        return res;
    }

private:
    Vector7d tau_th_base_;
    double kappa_;
    double epsilon_fdi_;
};

}  // namespace adls
