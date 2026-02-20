#pragma once

#include "control/Vec.hpp"

namespace olinv {

class PIController {
public:
    struct Limits {
        double u_min;
        double u_max;
    };

    static constexpr Limits kDefaultLimits{-1e9, 1e9};

    PIController() = default;
    PIController(double kp, double ki, Limits lim = kDefaultLimits) : kp_(kp), ki_(ki), lim_(lim) {}

    void set_gains(double kp, double ki) {
        kp_ = kp;
        ki_ = ki;
    }
    void set_limits(Limits lim) { lim_ = lim; }
    void reset(double integrator = 0.0) { i_ = integrator; }

    double update(double error, double dt);

private:
    double kp_{0.0};
    double ki_{0.0};
    Limits lim_{kDefaultLimits};
    double i_{0.0};
};

}  // namespace olinv
