#pragma once

#include "ethercat_core/loop.hpp"
#include "ethercat_core/data_types.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/default_adapter_factory.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"
#include "ethercat_core/devices/motor_drives/drive_bases/ds402/data_types.hpp"
#include "pdo_log.hpp"
#include "feature_functions/joint_braking.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

// ── Testbench parameter defaults ──────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY    = "config/ethercat_device_config/topology.singlejoint1.json";
static constexpr const char* DEFAULT_DRIVE_SLAVE = "main_drive";
static constexpr double      DEFAULT_PUB_HZ      = 200.0;
static constexpr double      DEFAULT_FAULT_RESET = 0.5;

// ── DS402 state name ──────────────────────────────────────────────────────────

inline const char* cia402Name(ethercat_core::ds402::Cia402State s) {
    using ethercat_core::ds402::Cia402State;
    switch (s) {
    case Cia402State::NOT_READY_TO_SWITCH_ON: return "NOT_READY";
    case Cia402State::SWITCH_ON_DISABLED:     return "SW_ON_DISABLED";
    case Cia402State::READY_TO_SWITCH_ON:     return "READY";
    case Cia402State::SWITCHED_ON:            return "SWITCHED_ON";
    case Cia402State::OPERATION_ENABLED:      return "OP_ENABLED";
    case Cia402State::QUICK_STOP_ACTIVE:      return "QUICK_STOP";
    case Cia402State::FAULT_REACTION_ACTIVE:  return "FAULT_REACTION";
    case Cia402State::FAULT:                  return "FAULT";
    }
    return "UNKNOWN";
}

// ── Shared command state (written by ROS2 subscriber, read by RT callback) ────

struct CommandState {
    float    main_speed    = 0.0f;  // rad/s, converted to mrev/s before sending
    float    main_position = 0.0f;  // rad, converted to enc_cnt before sending
    float    main_torque   = 0.0f;
    float    main_current  = 0.0f;
    bool     main_enable   = false;
    bool     fault_reset   = false;
    int8_t   main_mode     = static_cast<int8_t>(ethercat_core::ds402::ModeOfOperation::CYCLIC_SYNC_VELOCITY);
    // Control gains (seeded from startup SDO; overridable via /jipt/command)
    float    main_torque_kp      = 0.0f;
    float    main_torque_max_out = 0.0f;
    float    main_torque_min_out = 0.0f;
    float    main_vel_kp         = 0.0f;
    float    main_vel_ki         = 0.0f;
    float    main_vel_kd         = 0.0f;
    float    main_pos_kp         = 0.0f;
    float    main_pos_ki         = 0.0f;
    float    main_pos_kd         = 0.0f;
    float    main_max_current_a  = 0.0f;
    // One-shot log-rotation flag — triggers drain thread to close and reopen CSV.
    bool     save_log            = false;
    // Estimated rotor inertia (kg·m²) — forwarded to joint braking feature.
    float    inertia             = 0.0f;
    // Joint hardstop limits and safety margin (rad) — used by JIP algorithm.
    float    hardstop_pos_upper  = 0.0f;
    float    hardstop_pos_lower  = 0.0f;
    float    margin              = 0.0f;
};

// ── Per-drive gain snapshot ────────────────────────────────────────────────────

struct DriveGains {
    float torque_kp              = 0.0f;
    float torque_loop_max_output = 0.0f;
    float torque_loop_min_output = 0.0f;
    float velocity_loop_kp       = 0.0f;
    float velocity_loop_ki       = 0.0f;
    float velocity_loop_kd       = 0.0f;
    float position_loop_kp       = 0.0f;
    float position_loop_ki       = 0.0f;
    float position_loop_kd       = 0.0f;
    float max_current_a          = 0.0f;
    float gear_ratio             = 1.0f;
};

// ── Joint-impact-prevention testbench helper ──────────────────────────────────
//
// Encapsulates single Novanta/Volcano main_drive logic: command building,
// per-cycle PDO log capture, drive JSON telemetry, and debug prints.

class JointImpactPreventionTestbench {
public:
    JointImpactPreventionTestbench(
        const std::string& drive_slave,
        int                drive_soem_idx,
        int                main_out_enc_bits
    );

    // Reads startup SDO gains and seeds cmd_state under cmd_mutex.
    void extractAndSeedGains(
        ethercat_core::MasterRuntime& rt,
        CommandState& cmd_state,
        std::mutex&   cmd_mutex
    );

    // Returns the CycleCallback for loop.setCycleCallback().
    ethercat_core::EthercatLoop::CycleCallback makeCallback(
        CommandState&  cmd_state,
        std::mutex&    cmd_mutex,
        dyno::PdoLogBuffer<200>& log_buf,
        std::chrono::steady_clock::time_point reset_end
    );

    // Serialise one PdoLogRecord to a CSV row string (no trailing newline).
    // Only main drive fields are populated; dut/sensor columns are zero.
    // Column order matches dyno::PDO_LOG_CSV_HEADER.
    static std::string serializeToCsvRow(const dyno::PdoLogRecord& r);

    // Build a drive telemetry JSON string for ROS2 publishing.
    static std::string makeDriveJson(
        const std::string& slave_name,
        int   soem_idx,
        const ethercat_core::SystemStatus& status,
        int   out_enc_bits,
        const DriveGains& gains
    );

    // Debug printf to stdout.
    void printDebug(
        const ethercat_core::SystemStatus& status,
        const ethercat_core::LoopStats&    stats,
        const CommandState& cmd
    ) const;

    int  driveIdx()       const { return drive_soem_idx_; }
    int  mainOutEncBits() const { return main_out_enc_bits_; }

    // Set once by extractAndSeedGains; read-only after that.
    float sdo_torque_abs_max_ = 0.0f;
    float sdo_gear_ratio_     = 1.0f;

    // RT-thread outputs — written after braking_.write(), safe to read from ROS2 thread.
    std::atomic<float> rt_torque_max_out_{0.0f};
    std::atomic<float> rt_torque_min_out_{0.0f};

private:
    DriveGains extractGains_(const ethercat_core::MasterRuntime& rt,
                             const std::string& slave_name) const;

    std::string drive_slave_;
    int         drive_soem_idx_;
    int         main_out_enc_bits_;
    Braking     braking_;
};
