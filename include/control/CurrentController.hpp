#pragma once

#include "control/PIController.hpp"
#include "control/Vec.hpp"

namespace olinv {

struct CurrentControlParams {
    double R{0.05};
    double Ld{200e-6};
    double Lq{200e-6};
    double psi_f{0.015};
    double Vdc{48.0};
};

class CurrentController {
public:
    explicit CurrentController(const CurrentControlParams& p);

    void reset();
    void set_pi_gains(double kp_d, double ki_d, double kp_q, double ki_q);

    Vec2 step(const Vec2& i_dq_ref, const Vec2& i_dq_meas, double omega_e, double dt);

private:
    CurrentControlParams p_;
    PIController pi_d_;
    PIController pi_q_;

    Vec2 limit_vdq_spwm(const Vec2& v_dq) const;
};

}  // namespace olinv
