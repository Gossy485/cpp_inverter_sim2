#pragma once

#include "control/Vec.hpp"

namespace olinv {

class InverterModel {
public:
    explicit InverterModel(double vdc) : vdc_(vdc) {}

    Vec3 phase_voltages_from_duty(const Vec3& duty_abc) const;

private:
    double vdc_{48.0};
};

}  // namespace olinv
