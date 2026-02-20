#pragma once

#include "control/PIController.hpp"

namespace olinv {

struct SpeedControlParams {
    double iq_min{-40.0};
    double iq_max{40.0};
};

class SpeedController {
public:
    explicit SpeedController(const SpeedControlParams& params);

    void set_pi_gains(double kp, double ki);
    void reset();

    double step(double omega_ref, double omega_meas, double dt);

private:
    PIController pi_;
};

}  // namespace olinv
