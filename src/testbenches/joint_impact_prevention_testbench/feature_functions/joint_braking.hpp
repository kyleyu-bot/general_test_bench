#pragma once

#include "ethercat_core/data_types.hpp"

#include <string>

class Braking {
public:
    Braking() = default;

    // Capture all inputs needed by the JIP algorithm for one RT cycle.
    //   velocity_rad_s      — motor-side velocity (rad/s)
    //   position_rad        — output/joint position (rad)
    //   hardstop_pos_upper  — upper hardstop limit (rad)
    //   hardstop_pos_lower  — lower hardstop limit (rad)
    //   margin              — safety zone before hardstop triggers braking (rad)
    //   inertia             — estimated rotor inertia (kg·m²)
    void read(float velocity_rad_s,
              float position_rad,
              float hardstop_pos_upper,
              float hardstop_pos_lower,
              float margin,
              float inertia);

    // Apply braking commands into the outgoing system command for the given slave.
    void write(ethercat_core::SystemCommand& cmd, const std::string& slave_name);

private:
    float velocity_rad_s_     = 0.f;
    float position_rad_       = 0.f;
    float hardstop_pos_upper_ = 0.f;
    float hardstop_pos_lower_ = 0.f;
    float margin_             = 0.f;
    float inertia_            = 0.f;
};
