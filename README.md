# C++ Inverter + PMSM Simulation

A compact field-oriented control simulation with:
- PMSM electrical/mechanical model (RK4 integration)
- Cascaded closed-loop controllers
  - outer speed PI loop (`omega -> iq_ref`)
  - inner current PI loop (`id/iq -> vd/vq`) with decoupling terms
- Clarke/Park transforms
- Inverter average model + SPWM duty generation
- Speed reference is automatically clipped to a voltage-feasible maximum based on `Vdc`, `psi_f`, and pole pairs
- Multi-rate simulation (control tick + motor integration substeps)
- CSV logging and Python plotting helper

## Project layout

```text
include/
  control/
    CurrentController.hpp
    PIController.hpp
    SpeedController.hpp
    SPWM.hpp
    Transforms.hpp
    Vec.hpp
  plant/
    InverterModel.hpp
    PMSMModel.hpp
  sim/
    Logger.hpp
    Sim.hpp
src/
  control/
  plant/
  sim/
```

## Build and run

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j
./inverter_sim
```

The simulation writes CSV data to `results/results.csv` (as configured in `src/main.cpp`).
