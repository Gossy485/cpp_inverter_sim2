#include "plant/InverterModel.hpp"

namespace olinv {

Vec3 InverterModel::phase_voltages_from_duty(const Vec3& duty_abc) const {
    const double v_a0 = (2.0 * duty_abc.a - 1.0) * (vdc_ * 0.5);
    const double v_b0 = (2.0 * duty_abc.b - 1.0) * (vdc_ * 0.5);
    const double v_c0 = (2.0 * duty_abc.c - 1.0) * (vdc_ * 0.5);
    const double v_cm = (v_a0 + v_b0 + v_c0) / 3.0;
    return {v_a0 - v_cm, v_b0 - v_cm, v_c0 - v_cm};
}

}  // namespace olinv
