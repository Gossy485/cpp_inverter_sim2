#include "sim/Sim.hpp"

int main() {
    using namespace olinv;

    SimParams sim_params;
    sim_params.t_end = 0.3;
    sim_params.f_ctrl = 20000.0;
    sim_params.substeps = 10;
    sim_params.Vdc = 48.0;
    sim_params.id_ref = 0.0;
    sim_params.omega_ref_step = 280.0;
    sim_params.omega_ref_ramp_time = 0.05;
    sim_params.csv_path = "results/results.csv";

    PMSMParams motor_params;
    motor_params.R = 0.05;
    motor_params.Ld = 220e-6;
    motor_params.Lq = 220e-6;
    motor_params.psi_f = 0.015;
    motor_params.pole_pairs = 7;
    motor_params.J = 1.2e-4;
    motor_params.B = 1.0e-4;
    motor_params.T_load = 0.05;

    Sim(sim_params, motor_params).run();
    return 0;
}
