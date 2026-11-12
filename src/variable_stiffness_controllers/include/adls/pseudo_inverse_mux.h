#pragma once
/**
 * @file pseudo_inverse_mux.h
 * @brief Four comparison pseudoinverse methods (M1–M4) — unified interface.
 *
 * M1: Moore-Penrose   J⁺ = V₁ S⁻¹ Uᵀ
 * M2: Scalar DLS      J⁺ = Jᵀ(JJᵀ + λ²_max I)⁻¹
 * M3: Moore-Penrose   (same as M1, nullspace compliance active)
 * M4: A-DLS (proposed) J⁺ = Jᵀ(JJᵀ + Λ_cart)⁻¹
 *
 * All return (J⁺_eff, N_eff) = (7×6, 7×7).
 */

#include "types.h"
#include <Eigen/SVD>

namespace adls {

struct PseudoInverseResult {
    Matrix76d J_plus;
    Matrix7d N;
};

/**
 * Compute effective pseudoinverse and nullspace projector.
 * @param J            Jacobian (6×7)
 * @param Lambda_cart  Cartesian damping matrix (6×6, for M2/M4)
 * @param method       "M1", "M2", "M3", or "M4"
 */
inline PseudoInverseResult computePseudoinverse(
    const Matrix67d& J, const Matrix6d& Lambda_cart, const std::string& method) {

    PseudoInverseResult res;

    if (method == "M1" || method == "M3") {
        // Moore-Penrose: J⁺ = V₁ S⁻¹ Uᵀ (Eq after 3.3)
        Eigen::JacobiSVD<Matrix67d> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto s = svd.singularValues();
        Matrix6d S_inv = Matrix6d::Zero();
        for (int k = 0; k < M_TASK; ++k) {
            if (s[k] > 1e-10) S_inv(k, k) = 1.0 / s[k];
        }
        // Explicit temporary to avoid Eigen expression template chain issues
        // matrixV() returns 7×7 (full) → take first 6 cols for V₁ (7×6)
        Eigen::Matrix<double, N_JOINTS, M_TASK> V1 = svd.matrixV().leftCols(M_TASK);
        Matrix6d SUT = S_inv * svd.matrixU().transpose();
        res.J_plus.noalias() = V1 * SUT;

    } else if (method == "M2" || method == "M4") {
        // Scalar DLS (M2) or A-DLS (M4):
        //   J⁺ = Jᵀ(JJᵀ + Λ)⁻¹   (Eq 4.4 for M4, λ²_max·I for M2)
        Matrix6d JJT = J * J.transpose();
        Matrix6d M_damped = JJT + Lambda_cart;
        res.J_plus.noalias() = J.transpose() * M_damped.inverse();

    } else {
        throw std::invalid_argument("Unknown method: " + method);
    }

    // Nullspace projector: N = I - J⁺J
    res.N = Matrix7d::Identity() - res.J_plus * J;

    return res;
}

}  // namespace adls
