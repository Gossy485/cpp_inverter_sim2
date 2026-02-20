#include "control/SPWM.hpp"

#include "control/Vec.hpp"

namespace olinv {

Vec3 SPWM::duty_from_phase_voltage(const Vec3& v_phase_ref) const {
    const double half_vdc = 0.5 * vdc_;
    const double m_a = clamp(v_phase_ref.a / half_vdc, -0.999, 0.999);
    const double m_b = clamp(v_phase_ref.b / half_vdc, -0.999, 0.999);
    const double m_c = clamp(v_phase_ref.c / half_vdc, -0.999, 0.999);
    return {0.5 * (m_a + 1.0), 0.5 * (m_b + 1.0), 0.5 * (m_c + 1.0)};
}

}  // namespace olinv
