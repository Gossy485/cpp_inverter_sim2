#include "sim/Sim.hpp"

#include <cmath>
#include <iostream>

namespace olinv {
namespace {
constexpr double kPi = 3.14159265358979323846;
}

Sim::Sim(const SimParams& sp, const PMSMParams& mp)
    : sp_(sp),
      motor_(mp),
      current_controller_(CurrentControlParams{mp.R, mp.Ld, mp.Lq, mp.psi_f, sp.Vdc}),
      speed_controller_(SpeedControlParams{-60.0, 60.0}),
      inverter_(sp.Vdc),
      spwm_(sp.Vdc),
      logger_(sp.csv_path) {
    const double w_cc = 2.0 * kPi * 1000.0;
    current_controller_.set_pi_gains(mp.Ld * w_cc, mp.R * w_cc, mp.Lq * w_cc, mp.R * w_cc);

    const double w_sc = 2.0 * kPi * 25.0;
    const double kt = 1.5 * static_cast<double>(mp.pole_pairs) * mp.psi_f;
    speed_controller_.set_pi_gains((mp.J * w_sc) / kt, (mp.J * w_sc * w_sc) / kt);

    motor_.reset(PMSMState{});
    logger_.write_header();
}

double Sim::max_trackable_speed() const {
    const PMSMParams& mp = motor_.params();
    const double omega_e_max = 0.5 * sp_.Vdc / mp.psi_f;
    const double omega_m_max = omega_e_max / static_cast<double>(mp.pole_pairs);
    return 0.9 * omega_m_max;
}

double Sim::speed_ref(double t) const {
    const double omega_ramp = (sp_.omega_ref_ramp_time <= 1e-12 || t >= sp_.omega_ref_ramp_time)
                                  ? sp_.omega_ref_step
                                  : sp_.omega_ref_step * (t / sp_.omega_ref_ramp_time);
    return clamp(omega_ramp, -max_trackable_speed(), max_trackable_speed());
}

void Sim::run() {
    const double dt_ctrl = 1.0 / sp_.f_ctrl;
    const double dt_sim = dt_ctrl / static_cast<double>(sp_.substeps);
    const int total_steps = static_cast<int>(std::ceil(sp_.t_end / dt_sim));

    Vec2 v_dq_cmd{0.0, 0.0};
    Vec3 duty_abc{0.5, 0.5, 0.5};
    Vec3 v_abc_ref{0.0, 0.0, 0.0};
    Vec3 v_abc_applied = inverter_.phase_voltages_from_duty(duty_abc);

    double t = 0.0;
    int ctrl_countdown = 0;
    bool printed_ref_clip_note = false;

    for (int k = 0; k < total_steps; ++k) {
        const Vec2 i_dq = motor_.state().i_dq;
        const double theta_e = motor_.state().theta_e;

        if (ctrl_countdown == 0) {
            const double omega_ref_raw = (sp_.omega_ref_ramp_time <= 1e-12 || t >= sp_.omega_ref_ramp_time)
                                             ? sp_.omega_ref_step
                                             : sp_.omega_ref_step * (t / sp_.omega_ref_ramp_time);
            const double omega_ref = speed_ref(t);
            if (!printed_ref_clip_note && std::abs(omega_ref_raw - omega_ref) > 1e-9) {
                std::cout << "Speed reference clipped from " << omega_ref_raw << " to " << omega_ref
                          << " rad/s (bus-voltage limit).\n";
                printed_ref_clip_note = true;
            }

            const double iq_ref = speed_controller_.step(omega_ref, motor_.state().omega_m, dt_ctrl);
            const Vec2 i_dq_ref{sp_.id_ref, iq_ref};

            v_dq_cmd = current_controller_.step(i_dq_ref, i_dq, motor_.omega_e(), dt_ctrl);
            v_abc_ref = inv_clarke(inv_park(v_dq_cmd, theta_e));
            duty_abc = spwm_.duty_from_phase_voltage(v_abc_ref);
            v_abc_applied = inverter_.phase_voltages_from_duty(duty_abc);

            const Vec3 i_abc = inv_clarke(inv_park(i_dq, theta_e));
            logger_.log(t,
                        omega_ref,
                        i_dq_ref,
                        i_dq,
                        i_abc,
                        v_dq_cmd,
                        v_abc_ref,
                        duty_abc,
                        v_abc_applied,
                        motor_.torque_e(),
                        motor_.state().omega_m);

            ctrl_countdown = sp_.substeps - 1;
        } else {
            --ctrl_countdown;
        }

        const Vec2 v_dq_applied = park(clarke(v_abc_applied), theta_e);
        motor_.step(v_dq_applied, dt_sim);
        t += dt_sim;
    }

    std::cout << "Simulation complete.\nCSV: " << sp_.csv_path << '\n';
}

}  // namespace olinv
