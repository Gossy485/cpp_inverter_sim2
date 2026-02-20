#pragma once

#include "control/Vec.hpp"

namespace olinv {

class SPWM {
public:
    explicit SPWM(double vdc) : vdc_(vdc) {}

    Vec3 duty_from_phase_voltage(const Vec3& v_phase_ref) const;

private:
    double vdc_{48.0};
};

}  // namespace olinv
