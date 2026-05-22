#pragma once

#include "ethercat_core/devices/motor_drives/drive_bases/ds402/data_types.hpp"
#include <cstdint>

namespace ethercat_core::novanta::volcano {

using ds402::Cia402State;
using ds402::ModeOfOperation;

struct Command {
    ModeOfOperation mode_of_operation      = ModeOfOperation::NO_MODE;
    float           target_torque_nm       = 0.0f;
    float           target_velocity_mrevs    = 0.0f;  // 0x60FF, unit: mrev/s
    float           target_position__enc_cnt = 0.0f;  // 0x607A, unit: raw encoder counts
    float           torque_command_2022    = 0.0f;  // 0x2022 direct torque
    float           iq_setpoint_a          = 0.0f;  // 0x201A current quadrature set-point
    float           torque_kp              = 0.0f;  // 0x2523
    float           torque_loop_max_output = 0.0f;  // 0x2527
    float           torque_loop_min_output = 0.0f;  // 0x2528
    float           velocity_loop_kp       = 0.0f;  // 0x250A
    float           velocity_loop_ki       = 0.0f;  // 0x250B
    float           velocity_loop_kd       = 0.0f;  // 0x250C
    float           position_loop_kp       = 0.0f;  // 0x2511
    float           position_loop_ki       = 0.0f;  // 0x2512
    float           position_loop_kd       = 0.0f;  // 0x2513
    bool            enable_drive           = false;
    bool            clear_fault            = false;
    uint64_t        seq                    = 0;
    int64_t         stamp_ns               = 0;
};

struct DriveStatus {
    bool            online                    = false;
    bool            operational               = false;
    bool            faulted                   = false;
    uint8_t         al_state_code             = 0;
    Cia402State     cia402_state              = Cia402State::NOT_READY_TO_SWITCH_ON;
    uint16_t        status_word               = 0;
    int8_t          mode_of_operation_display = 0;
    uint16_t        error_code                = 0;

    bool            ready_to_switch_on  = false;
    bool            switched_on         = false;
    bool            operation_enabled   = false;
    bool            fault               = false;
    bool            voltage_enabled     = false;
    bool            quick_stop_active   = false;
    bool            switch_on_disabled  = false;
    bool            warning             = false;
    bool            remote              = false;
    bool            target_reached      = false;

    float           measured_torque_nm          = 0.0f;
    int32_t         measured_input_side_velocity_raw  = 0;    // 0x606C, unit: mrev/s
    int32_t         measured_output_side_position_raw_cnt = 0; // 0x6064, output side encoder
    int32_t         input_encoder_pos            = 0;  // 0x204A, input side encoder
    float           velocity_command_received    = 0.0f;
    int32_t         position_setpoint            = 0;
    int32_t         max_position                 = 0;
    int32_t         min_position                 = 0;
    float           max_velocity_abs             = 0.0f;
    float           bus_voltage                  = 0.0f;
    float           motor_temp                   = 0.0f;
    float           iq_actual                    = 0.0f;
    float           id_actual                    = 0.0f;
    float           idc_actual                   = 0.0f;
    float           iq_command                   = 0.0f;
    float           id_command                   = 0.0f;

    int64_t         dc_time_error_ns  = 0;
    int64_t         cycle_time_ns     = 0;
    uint64_t        seq               = 0;
    int64_t         stamp_ns          = 0;
};

} // namespace ethercat_core::novanta::volcano
