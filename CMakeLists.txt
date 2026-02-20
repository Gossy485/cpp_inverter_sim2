cmake_minimum_required(VERSION 3.16)
project(cpp_inverter_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(inverter_sim
    src/main.cpp
    src/control/CurrentController.cpp
    src/control/PIController.cpp
    src/control/SPWM.cpp
    src/control/SpeedController.cpp
    src/control/Transforms.cpp
    src/plant/InverterModel.cpp
    src/plant/PMSMModel.cpp
    src/sim/Logger.cpp
    src/sim/Sim.cpp
)

target_include_directories(inverter_sim PRIVATE include)

if(MSVC)
    target_compile_options(inverter_sim PRIVATE /W4 /permissive-)
else()
    target_compile_options(inverter_sim PRIVATE -Wall -Wextra -Wpedantic)
endif()
