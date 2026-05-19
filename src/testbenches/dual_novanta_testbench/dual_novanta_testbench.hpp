#pragma once

#include "ethercat_core/loop.hpp"
#include "ethercat_core/data_types.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/adapter.hpp"
#include "ethercat_core/devices/motor_drives/drive_bases/ds402/data_types.hpp"
#include "pdo_log.hpp"
#include "testbench_utils/function_generator.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

// ── Testbench parameter defaults ──────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY      = "config/ethercat_device_config/topology.dyno2.template6.json";
static constexpr const char* DEFAULT_DRIVE_SLAVE   = "main_drive";
static constexpr const char* DEFAULT_DUT_SLAVE     = "dut";
static constexpr const char* DEFAULT_ENCODER_SLAVE = "encoder_interface";
static constexpr const char* DEFAULT_TORQUE_SLAVE  = "analog_input_interface";
static constexpr const char* DEFAULT_IO_SLAVE      = "digital_IO";
static constexpr double      DEFAULT_PUB_HZ        = 200.0;
static constexpr double      DEFAULT_FAULT_RESET   = 0.5;

// ── DS402 state name (inline so bridge shutdown code can use it) ──────────────

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
    float    dut_speed     = 0.0f;
    float    main_position = 0.0f;  // rad, converted to enc_cnt before sending
    float    dut_position  = 0.0f;
    float    main_torque   = 0.0f;
    float    dut_torque    = 0.0f;
    float    main_current  = 0.0f;
    float    dut_current   = 0.0f;
    bool     main_enable   = false;
    bool     dut_enable    = false;
    bool     fault_reset   = false;
    bool     hold_output1  = false;
    int8_t   main_mode     = static_cast<int8_t>(ethercat_core::ds402::ModeOfOperation::CYCLIC_SYNC_VELOCITY);
    int8_t   dut_mode      = static_cast<int8_t>(ethercat_core::ds402::ModeOfOperation::CYCLIC_SYNC_VELOCITY);
    // Control gains (seeded from startup SDO; overridable via /dyno/command)
    float    main_torque_kp       = 0.0f;
    float    main_torque_max_out  = 0.0f;
    float    main_torque_min_out  = 0.0f;
    float    main_vel_kp          = 0.0f;
    float    main_vel_ki          = 0.0f;
    float    main_vel_kd          = 0.0f;
    float    main_pos_kp          = 0.0f;
    float    main_pos_ki          = 0.0f;
    float    main_pos_kd          = 0.0f;
    float    main_max_current_a   = 0.0f;
    float    dut_torque_kp        = 0.0f;
    float    dut_torque_max_out   = 0.0f;
    float    dut_torque_min_out   = 0.0f;
    float    dut_vel_kp           = 0.0f;
    float    dut_vel_ki           = 0.0f;
    float    dut_vel_kd           = 0.0f;
    float    dut_pos_kp           = 0.0f;
    float    dut_pos_ki           = 0.0f;
    float    dut_pos_kd           = 0.0f;
    float    dut_max_current_a    = 0.0f;
    // Torque sensor ADC scale (Nm); use current value as default so omitted
    // command messages leave the scale unchanged.
    float    ch1_torque_scale     = 200.0f;  // matches Elm3002Adapter ch1 default
    float    ch2_torque_scale     = 20.0f;   // matches Elm3002Adapter ch2 default
    // One-shot zero flags — cleared by the bridge after applying.
    bool     zero_torque_ch1      = false;
    bool     zero_torque_ch2      = false;
    // One-shot log-rotation flag — triggers drain thread to close and reopen CSV.
    bool     save_log             = false;
    // Function generator config (main drive)
    bool     main_fg_enable       = false;
    int      main_fg_waveform     = 0;    // WaveformType as int
    int      main_fg_control_type = 0;    // ControlType as int
    float    main_fg_amplitude    = 0.0f;
    float    main_fg_frequency    = 1.0f;
    float    main_fg_offset       = 0.0f;
    float    main_fg_phase        = 0.0f;
    // Function generator config (DUT drive)
    bool     dut_fg_enable        = false;
    int      dut_fg_waveform      = 0;
    int      dut_fg_control_type  = 0;
    float    dut_fg_amplitude     = 0.0f;
    float    dut_fg_frequency     = 1.0f;
    float    dut_fg_offset        = 0.0f;
    float    dut_fg_phase         = 0.0f;
    float    main_fg_chirp_f_low  = 0.1f;
    float    main_fg_chirp_f_high = 10.0f;
    float    main_fg_chirp_dur    = 10.0f;
    float    dut_fg_chirp_f_low   = 0.1f;
    float    dut_fg_chirp_f_high  = 10.0f;
    float    dut_fg_chirp_dur     = 10.0f;
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
};

// ── Dual-Novanta testbench helper ─────────────────────────────────────────────
//
// Encapsulates all Novanta/Volcano-specific logic: command building, per-cycle
// PDO log capture, CSV serialisation, drive JSON telemetry, and debug prints.
// Bridge code stays device-agnostic; only this class knows about drive types.

class DualNovantaTestbench {
public:
    DualNovantaTestbench(
        const std::string& drive_slave,
        const std::string& dut_slave,
        const std::string& encoder_slave,
        const std::string& torque_slave,
        const std::string& io_slave,
        bool               dut_present,
        ethercat_core::beckhoff::elm3002::Elm3002Adapter* elm3002,
        int  drive_soem_idx,
        int  dut_soem_idx,
        int  main_out_enc_bits,
        int  dut_out_enc_bits
    );

    // Reads startup SDO gains and seeds cmd_state under cmd_mutex.
    void extractAndSeedGains(
        ethercat_core::MasterRuntime& rt,
        CommandState& cmd_state,
        std::mutex&   cmd_mutex
    );

    // Returns the CycleCallback for loop.setCycleCallback().
    // The callback captures cmd_state, cmd_mutex, log_buf, reset_end by reference.
    // All referenced objects must outlive the EthercatLoop.
    ethercat_core::EthercatLoop::CycleCallback makeCallback(
        CommandState&  cmd_state,
        std::mutex&    cmd_mutex,
        dyno::PdoLogBuffer<200>& log_buf,
        std::chrono::steady_clock::time_point reset_end
    );

    // Serialise one PdoLogRecord to a CSV row string (no trailing newline).
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

    // Debug printf to stdout — call only when --debug flag is set.
    void printDebug(
        const ethercat_core::SystemStatus& status,
        const ethercat_core::LoopStats&    stats,
        const CommandState& cmd,
        uint32_t enc,
        double ch1_t, double ch2_t
    ) const;

    // Accessors used by bridge main() for torque operations and SDO routing.
    ethercat_core::beckhoff::elm3002::Elm3002Adapter* elm3002()      const { return elm3002_; }
    int  driveIdx()       const { return drive_soem_idx_; }
    int  dutIdx()         const { return dut_soem_idx_; }
    bool dutPresent()     const { return dut_present_; }
    int  mainOutEncBits() const { return main_out_enc_bits_; }
    int  dutOutEncBits()  const { return dut_out_enc_bits_; }

    // Last per-cycle FG output in natural units (A / Nm / rad/s / rad).
    // Written by the RT callback; safe to read from the pub thread.
    float lastMainFgOut() const { return last_main_fg_out_.load(std::memory_order_relaxed); }
    float lastDutFgOut()  const { return last_dut_fg_out_.load(std::memory_order_relaxed); }

    void setGearRatios(float main_gr, float dut_gr) {
        main_gear_ratio_ = main_gr;
        dut_gear_ratio_  = dut_gr;
    }

private:
    DriveGains extractGains_(const ethercat_core::MasterRuntime& rt,
                              const std::string& slave_name) const;

    std::string drive_slave_, dut_slave_, encoder_slave_, torque_slave_, io_slave_;
    bool        dut_present_;
    ethercat_core::beckhoff::elm3002::Elm3002Adapter* elm3002_;
    int         drive_soem_idx_, dut_soem_idx_;
    int         main_out_enc_bits_, dut_out_enc_bits_;

    testbench_utils::FunctionGenerator fg_main_;
    testbench_utils::FunctionGenerator fg_dut_;
    int32_t  main_captured_pos_ = 0;
    int32_t  dut_captured_pos_  = 0;
    int8_t   prev_main_mode_    = 0;
    int8_t   prev_dut_mode_     = 0;

    std::atomic<float> last_main_fg_out_{0.f};
    std::atomic<float> last_dut_fg_out_{0.f};

    float main_gear_ratio_ = 1.f;
    float dut_gear_ratio_  = 1.f;
};
