#pragma once

#include <string>

#include "control/CurrentController.hpp"
#include "control/SpeedController.hpp"
#include "control/Transforms.hpp"
#include "control/Vec.hpp"
#include "plant/PMSMModel.hpp"
#include "plant/InverterModel.hpp"
#include "control/SPWM.hpp"
#include "sim/Logger.hpp"

namespace olinv {

struct SimParams {
    double t_end{0.25};
    double f_ctrl{20000.0};
    int substeps{10};
    double Vdc{48.0};

    double id_ref{0.0};
    double omega_ref_step{250.0};
    double omega_ref_ramp_time{0.05};

    std::string csv_path{"results/results.csv"};
};

class Sim {
public:
    Sim(const SimParams& sp, const PMSMParams& mp);
    void run();

private:
    SimParams sp_;
    PMSMModel motor_;
    CurrentController current_controller_;
    SpeedController speed_controller_;
    InverterModel inverter_;
    SPWM spwm_;
    Logger logger_;

    double speed_ref(double t) const;
};

}  // namespace olinv
