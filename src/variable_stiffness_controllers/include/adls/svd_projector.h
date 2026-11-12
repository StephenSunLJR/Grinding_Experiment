#pragma once
/**
 * @file svd_projector.h
 * @brief Jacobian SVD decomposition + GMO residual task-space projection (§3.2–3.3).
 *
 * §3.2: J = U S V₁ᵀ  (compact SVD, Eq 3.3)
 *   U: 6×6 orthogonal, columns = task-space principal directions
 *   S: 6×1 singular values σ₁ ≥ ... ≥ σ₆
 *   V₁: 7×6 column-orthogonal, V₁ᵀV₁ = I
 *
 * §3.3: α_k = σ_k/(σ_k²+λ²_base) · v_kᵀ · r̃  (Eq 3.6)
 *        α_th,k = adaptive threshold from joint-space noise (Eq 3.7a)
 *        α_k ← deadzone(α_raw,k, α_th,k)       (Eq 3.7)
 *
 * Physical meaning:
 *   α_k(t) = collision force projection onto task direction u_k
 *   |α_k| large → direction k needs compliance
 *   |α_k| ≈ 0  → direction k maintains full accuracy
 */

#include "types.h"
#include <Eigen/SVD>
#include <cmath>

namespace adls {

class SVDProjector {
public:
    SVDProjector(const ADLSParams& p)
        : lambda2_base_(p.lambda2_base),
          r_noise_(p.r_noise),
          epsilon_(p.alpha_th_epsilon) {}

    /**
     * Project masked GMO residual to task space.
     * @return alpha      deadzone-processed collision projection (6×1)
     * @return alpha_raw  raw projection before deadzone
     * @return alpha_th   adaptive thresholds
     * @return U, s, V1   SVD decomposition (for downstream reuse)
     */
    struct ProjectResult {
        Vector6d alpha;
        Vector6d alpha_raw;
        Vector6d alpha_th;
        Matrix6d U;
        Vector6d s;
        Eigen::Matrix<double, N_JOINTS, M_TASK> V1;
        Matrix67d J;
    };

    ProjectResult project(const Matrix67d& J, const Vector7d& r_tilde,
                          const Vector7d& m_aff) {
        ProjectResult res;
        res.J = J;

        // ── Compact SVD: J = U S V₁ᵀ (Eq 3.3) ──
        Eigen::JacobiSVD<Matrix67d> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        res.U = svd.matrixU();          // 6×6
        res.s = svd.singularValues();   // 6×1
        res.V1 = svd.matrixV().leftCols(M_TASK);  // 7×6 (explicitly take first 6 cols)

        // Symbol tracking (§4.5 Strategy B): enforce V₁ sign consistency
        if (V1_prev_.cols() > 0) {
            for (int k = 0; k < M_TASK; ++k) {
                if (res.V1.col(k).dot(V1_prev_.col(k)) < 0.0) {
                    res.V1.col(k) *= -1.0;
                    res.U.col(k) *= -1.0;
                }
            }
        }
        V1_prev_ = res.V1;

        // ── Eq (3.6): α_raw,k = σ_k / (σ_k² + λ²_base) · v_kᵀ · r̃ ──
        res.alpha_raw.setZero();
        for (int k = 0; k < M_TASK; ++k) {
            double coeff = res.s[k] / (res.s[k] * res.s[k] + lambda2_base_);
            res.alpha_raw[k] = coeff * res.V1.col(k).dot(r_tilde);
        }

        // ── Eq (3.7a): adaptive threshold ──
        // NOTE: m_aff is NOT applied here. The threshold must be based on the
        // full joint noise floor so that alpha_th > 0 even when k_star=-1
        // (no collision). If m_aff were used, alpha_th → 0 when no collision,
        // causing sigmoid(0)=0.5 → lambda2=0.25 permanent spurious damping.
        res.alpha_th.setZero();
        for (int k = 0; k < M_TASK; ++k) {
            double coeff = res.s[k] / (res.s[k] * res.s[k] + lambda2_base_);
            double noise_sum = 0.0;
            for (int j = 0; j < N_JOINTS; ++j) {
                noise_sum += std::abs(res.V1(j, k)) * r_noise_[j];
            }
            res.alpha_th[k] = coeff * noise_sum + epsilon_;
        }

        // ── Eq (3.7): deadzone ──
        res.alpha.setZero();
        for (int k = 0; k < M_TASK; ++k) {
            double a = res.alpha_raw[k];
            double th = res.alpha_th[k];
            if (std::abs(a) >= th) {
                res.alpha[k] = a - std::copysign(th, a);
            }
        }

        return res;
    }

private:
    double lambda2_base_;
    Vector7d r_noise_;
    double epsilon_;
    Eigen::Matrix<double, N_JOINTS, M_TASK> V1_prev_;  // for symbol tracking
};

}  // namespace adls
