#include "joint_braking.hpp"
#include "math.h"

void Braking::read(float velocity_rad_s,
                   float position_rad,
                   float hardstop_pos_upper,
                   float hardstop_pos_lower,
                   float margin,
                   float inertia,
                   float max_current_a,
                   float torque_abs_max,
                   float gear_ratio)
{
    velocity_rad_s_     = velocity_rad_s;
    position_rad_       = position_rad;
    hardstop_pos_upper_ = hardstop_pos_upper;
    hardstop_pos_lower_ = hardstop_pos_lower;
    margin_             = margin;
    inertia_            = inertia;
    max_current_a_      = max_current_a;
    torque_abs_max_     = torque_abs_max;
    gear_ratio_         = gear_ratio;
}

void Braking::write(ethercat_core::novanta::volcano::Command& drive_cmd)
{
    // Dummy algorithm: sum all JIP parameters, clamp torque output to ±sum.
    static float rotational_energy; //= 0.5 * inertia_ * pow(velocity_rad_s_,2);
    static float distance_to_stop; // = rotational_energy / 2 / torque_abs_max_;
    static bool upper_braking = false;
    static bool lower_braking = false;
    static float braking_torque = 0;

    rotational_energy = 0.5 * inertia_ * pow(velocity_rad_s_,2);
    distance_to_stop = rotational_energy / 2 / torque_abs_max_ / pow(gear_ratio_, 2);

    if(abs(hardstop_pos_upper_ - position_rad_) <= (margin_ + distance_to_stop) && (velocity_rad_s_ > 10))    
    {
        upper_braking = true;
        braking_torque = rotational_energy / (margin_ + distance_to_stop);
        if(braking_torque >= 100)
        {
            braking_torque = 50;
        }
    }
    if(abs(hardstop_pos_lower_ - position_rad_) <= (margin_ + distance_to_stop) && (velocity_rad_s_ < 10))
    {
        lower_braking = true;
        braking_torque = rotational_energy / (margin_ + distance_to_stop);
        if(braking_torque >= 100)
        {
            braking_torque = 50;
        }
    }

    if(upper_braking)
    {
        drive_cmd.torque_loop_max_output = braking_torque * -1.0;
        if(abs(velocity_rad_s_) < 0.5)
        {
            upper_braking = false;
            drive_cmd.torque_loop_max_output = 100.0;
        }
    }
    if(lower_braking)
    {
        drive_cmd.torque_loop_min_output = braking_torque;
        if(abs(velocity_rad_s_) < 0.5)
        {
            lower_braking = false;
            drive_cmd.torque_loop_min_output = -100.0;
        }
    }
    // drive_cmd.torque_loop_max_output = -5.0;
    // drive_cmd.torque_loop_min_output = -100.0;
}
