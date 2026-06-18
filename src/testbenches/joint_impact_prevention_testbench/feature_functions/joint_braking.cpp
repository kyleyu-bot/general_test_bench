#include "joint_braking.hpp"

void Braking::read(float velocity_rad_s,
                   float position_rad,
                   float hardstop_pos_upper,
                   float hardstop_pos_lower,
                   float margin,
                   float inertia)
{
    velocity_rad_s_     = velocity_rad_s;
    position_rad_       = position_rad;
    hardstop_pos_upper_ = hardstop_pos_upper;
    hardstop_pos_lower_ = hardstop_pos_lower;
    margin_             = margin;
    inertia_            = inertia;
}

void Braking::write(ethercat_core::SystemCommand& cmd, const std::string& slave_name)
{
    (void)cmd;
    (void)slave_name;
    // JIP algorithm implementation goes here.
    // Inputs available: velocity_rad_s_, position_rad_,
    //                   hardstop_pos_upper_, hardstop_pos_lower_,
    //                   margin_, inertia_
}
