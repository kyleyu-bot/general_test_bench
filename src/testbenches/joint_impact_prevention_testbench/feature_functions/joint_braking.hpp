#pragma once

#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"

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
              float inertia,
              float max_current_a,
              float torque_abs_max,
              float gear_ratio);

    // Apply braking modifications directly to the drive command for this cycle.
    void write(ethercat_core::novanta::volcano::Command& drive_cmd);

private:
    float velocity_rad_s_     = 0.f;
    float position_rad_       = 0.f;
    float hardstop_pos_upper_ = 0.f;
    float hardstop_pos_lower_ = 0.f;
    float margin_             = 0.f;
    float inertia_            = 0.f;
    float max_current_a_      = 0.f;
    float torque_abs_max_     = 0.f;
    float gear_ratio_         = 1.f;
};
