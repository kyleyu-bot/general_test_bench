#pragma once

// Novanta Volcano PDO layout
// ──────────────────────────────────────────────────────────────────────────
// RX PDO (master → drive), 55 bytes, format "<Hbiifffffffffff":
//   0x1600: 0x6040(U16), 0x6060(S8),  0x607A(S32), 0x60FF(S32),
//           0x2022(F32), 0x201A(F32), 0x2523(F32)
//   0x1601: 0x2527(F32), 0x2528(F32), 0x250A(F32), 0x250B(F32),
//           0x250C(F32), 0x2511(F32), 0x2512(F32), 0x2513(F32)
//
// TX PDO (drive → master), 55 bytes, format "<HbifhfHiiiffffff":
//   0x1A00: 0x6041(U16), 0x6061(S8), 0x6064(S32), 0x2060(F32),
//           0x6077(S16), 0x2063(F32), 0x603F(U16)
//   0x1A01: 0x606C(S32), 0x204A(S32), 0x2078(S32), 0x2079(F32),
//           0x203B(F32), 0x203C(F32), 0x2076(F32)
//   0x1A02: 0x2072(F32), 0x2073(F32)
// ──────────────────────────────────────────────────────────────────────────

#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"
#include "ethercat_core/devices/motor_drives/drive_bases/ds402/pdo.hpp"
#include <cstdint>
#include <vector>

namespace ethercat_core::novanta::volcano {

static constexpr int RX_PDO_SIZE        = 55;
static constexpr int TX_PDO_SIZE        = 55;
static constexpr int LEGACY_TX_PDO_SIZE = 16;

struct PdoScaling {
    float torque_lsb_per_nm       = 10.0f;
    float velocity_lsb_per_rad_s  = 1000.0f;
    float position_lsb_per_rad    = 10000.0f;
};

#pragma pack(push, 1)

struct RxPdo {
    uint16_t controlword;            // 0x6040
    int8_t   mode_of_operation;      // 0x6060
    int32_t  target_position;        // 0x607A
    int32_t  target_velocity;        // 0x60FF
    float    torque_command_2022;    // 0x2022
    float    iq_setpoint;            // 0x201A
    float    torque_kp;              // 0x2523
    float    torque_loop_max_output; // 0x2527
    float    torque_loop_min_output; // 0x2528
    float    velocity_loop_kp;       // 0x250A
    float    velocity_loop_ki;       // 0x250B
    float    velocity_loop_kd;       // 0x250C
    float    position_loop_kp;       // 0x2511
    float    position_loop_ki;       // 0x2512
    float    position_loop_kd;       // 0x2513
};
static_assert(sizeof(RxPdo) == RX_PDO_SIZE, "RxPdo size mismatch");

struct TxPdo {
    uint16_t statusword;          // 0x6041
    int8_t   mode_display;        // 0x6061
    int32_t  measured_output_encoder_position_raw;    // 0x6064, output side encoder
    float    bus_voltage;         // 0x2060
    int16_t  estimated_torque;    // 0x6077
    float    motor_temp;          // 0x2063
    uint16_t error_code;          // 0x603F
    int32_t  motor_velocity;      // 0x606C
    int32_t  input_encoder_pos;   // 0x204A
    int32_t  position_setpoint;   // 0x2078
    float    velocity_setpoint;   // 0x2079
    float    iq_actual;           // 0x203B
    float    id_actual;           // 0x203C
    float    idc_actual;          // 0x2076
    float    iq_command;          // 0x2072
    float    id_command;          // 0x2073
};
static_assert(sizeof(TxPdo) == TX_PDO_SIZE, "TxPdo size mismatch");

#pragma pack(pop)

std::vector<uint8_t> packCommand(
    const Command& cmd,
    uint16_t       current_status_word = 0,
    const PdoScaling* scaling          = nullptr
);

DriveStatus unpackStatus(
    const uint8_t* data,
    int            size,
    uint64_t       seq           = 0,
    int64_t        stamp_ns      = 0,
    int64_t        cycle_time_ns = 0,
    int64_t        dc_error_ns   = 0,
    const PdoScaling* scaling    = nullptr
);

} // namespace ethercat_core::novanta::volcano
