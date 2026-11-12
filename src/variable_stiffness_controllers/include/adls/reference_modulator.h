#pragma once
/**
 * @file reference_modulator.h
 * @brief Slingshot effect mitigation via virtual trajectory modulation (§6.1.1).
 *
 * When collision damps a task direction, the position error x̃ = x_d - f(q)
 * accumulates. On collision removal, the sudden recovery of full gain produces
 * violent retraction — the "slingshot effect."
 *
 * Solution: modulate position error by collision participation β.
 *
 * Eq (6.1a): β_i = clip((Λ_cart,ii - λ²_base) / (λ²_max - λ²_base), 0, 1)
 * Eq (6.1b): x̃_eff = (1 - β) ⊙ x̃
 *
 * β_i = 0 → no collision → full tracking accuracy
 * β_i = 1 → full compliance → zero tracking error demand (reference follows actual)
 */

#include "types.h"

namespace adls {

class ReferenceModulator {
public:
    /**
     * Modulate tracking error to prevent slingshot.
     * @param error  x̃ = x_d - f(q)  (6×1)
     * @param beta   collision participation vector from ADLSDamping (6×1)
     * @return x̃_eff  modulated error (6×1)
     */
    static Vector6d modulate(const Vector6d& error, const Vector6d& beta) {
        // Eq (6.1b): x̃_eff = (1 - β) ⊙ x̃
        Vector6d factor = Vector6d::Ones() - beta;
        return error.cwiseProduct(factor);
    }
};

}  // namespace adls
