#include "control/SpeedController.hpp"

namespace olinv {

SpeedController::SpeedController(const SpeedControlParams& params)
    : pi_(0.0, 0.0, PIController::Limits{params.iq_min, params.iq_max}) {}

void SpeedController::set_pi_gains(double kp, double ki) {
    pi_.set_gains(kp, ki);
}

void SpeedController::reset() {
    pi_.reset();
}

double SpeedController::step(double omega_ref, double omega_meas, double dt) {
    return pi_.update(omega_ref - omega_meas, dt);
}

}  // namespace olinv
