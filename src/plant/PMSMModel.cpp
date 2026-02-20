#include "plant/PMSMModel.hpp"

namespace olinv {

void PMSMModel::reset(const PMSMState& x0) {
    x_ = x0;
    x_.theta_e = wrap_2pi(x_.theta_e);
}

double PMSMModel::torque_e() const {
    const double p = static_cast<double>(p_.pole_pairs);
    return 1.5 * p * (p_.psi_f * x_.i_dq.y + (p_.Ld - p_.Lq) * x_.i_dq.x * x_.i_dq.y);
}

PMSMModel::Deriv PMSMModel::deriv(const PMSMState& x, const Vec2& v_dq) const {
    const double omega_e = static_cast<double>(p_.pole_pairs) * x.omega_m;
    const double di_d = (v_dq.x - p_.R * x.i_dq.x + omega_e * p_.Lq * x.i_dq.y) / p_.Ld;
    const double di_q = (v_dq.y - p_.R * x.i_dq.y - omega_e * (p_.Ld * x.i_dq.x + p_.psi_f)) / p_.Lq;

    const double ppp = static_cast<double>(p_.pole_pairs);
    const double te = 1.5 * ppp * (p_.psi_f * x.i_dq.y + (p_.Ld - p_.Lq) * x.i_dq.x * x.i_dq.y);
    const double domega_m = (te - p_.T_load - p_.B * x.omega_m) / p_.J;

    return {{di_d, di_q}, domega_m, omega_e};
}

void PMSMModel::step(const Vec2& v_dq, double dt) {
    const PMSMState x0 = x_;
    const Deriv k1 = deriv(x0, v_dq);

    PMSMState x2 = x0;
    x2.i_dq += k1.di_dq * (0.5 * dt);
    x2.omega_m += 0.5 * dt * k1.domega_m;
    x2.theta_e += 0.5 * dt * k1.dtheta_e;
    const Deriv k2 = deriv(x2, v_dq);

    PMSMState x3 = x0;
    x3.i_dq += k2.di_dq * (0.5 * dt);
    x3.omega_m += 0.5 * dt * k2.domega_m;
    x3.theta_e += 0.5 * dt * k2.dtheta_e;
    const Deriv k3 = deriv(x3, v_dq);

    PMSMState x4 = x0;
    x4.i_dq += k3.di_dq * dt;
    x4.omega_m += dt * k3.domega_m;
    x4.theta_e += dt * k3.dtheta_e;
    const Deriv k4 = deriv(x4, v_dq);

    x_.i_dq.x += (dt / 6.0) * (k1.di_dq.x + 2.0 * k2.di_dq.x + 2.0 * k3.di_dq.x + k4.di_dq.x);
    x_.i_dq.y += (dt / 6.0) * (k1.di_dq.y + 2.0 * k2.di_dq.y + 2.0 * k3.di_dq.y + k4.di_dq.y);
    x_.omega_m += (dt / 6.0) * (k1.domega_m + 2.0 * k2.domega_m + 2.0 * k3.domega_m + k4.domega_m);
    x_.theta_e += (dt / 6.0) * (k1.dtheta_e + 2.0 * k2.dtheta_e + 2.0 * k3.dtheta_e + k4.dtheta_e);
    x_.theta_e = wrap_2pi(x_.theta_e);
}

}  // namespace olinv
