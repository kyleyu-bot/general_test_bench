// pdo_log.hpp — Ring-buffered PDO snapshot for offline logging.
//
// PdoLogRecord holds a flat snapshot of all TxPDO and RxPDO fields for the
// main drive and DUT, plus EtherCAT loop metadata, captured once per publish
// cycle in bridge_ros2.cpp.
//
// PdoLogBuffer<Depth> is a thread-safe FIFO ring buffer:
//   push()  — insert newest record; drops oldest when full.
//   pop()   — remove and return oldest record (nullopt when empty).

#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>

namespace dyno {

// ── PDO snapshot ──────────────────────────────────────────────────────────────

struct PdoLogRecord {
    // ── EtherCAT loop metadata ───────────────────────────────────────────────
    uint64_t cycle_count    = 0;   ///< monotonically increasing cycle index
    int64_t  stamp_ns       = 0;   ///< CLOCK_MONOTONIC from SystemStatus::stamp_ns
    int      wkc            = 0;   ///< working counter
    int64_t  cycle_time_ns  = 0;   ///< last cycle execution time (ns)
    int64_t  dc_error_ns    = 0;   ///< distributed-clock error (ns)
    int64_t  period_ns      = 0;   ///< measured period between cycle starts (ns)

    // ── Main drive — TxPDO (drive → master) ─────────────────────────────────
    uint16_t main_tx_statusword          = 0;
    int8_t   main_tx_mode_display        = 0;    ///< 0x6061
    int32_t  main_tx_output_enc_pos      = 0;    ///< 0x6064  output-side encoder (cnt)
    float    main_tx_bus_voltage         = 0.f;  ///< 0x2060  (V)
    float    main_tx_torque_nm           = 0.f;  ///< 0x6077  estimated torque (Nm)
    float    main_tx_motor_temp          = 0.f;  ///< 0x2063  (°C)
    uint16_t main_tx_error_code          = 0;    ///< 0x603F
    int32_t  main_tx_motor_velocity      = 0;    ///< 0x606C  (mrev/s)
    int32_t  main_tx_input_enc_pos       = 0;    ///< 0x204A  input-side encoder (cnt)
    int32_t  main_tx_position_setpoint   = 0;    ///< 0x2078
    float    main_tx_velocity_setpoint   = 0.f;  ///< 0x2079
    float    main_tx_iq_actual           = 0.f;  ///< 0x203B  (A)
    float    main_tx_id_actual           = 0.f;  ///< 0x203C  (A)
    float    main_tx_idc_actual          = 0.f;  ///< 0x2076  (A)
    float    main_tx_iq_command          = 0.f;  ///< 0x2072  (A)
    float    main_tx_id_command          = 0.f;  ///< 0x2073  (A)

    // ── Main drive — RxPDO (master → drive) ─────────────────────────────────
    int8_t   main_rx_mode_of_operation   = 0;    ///< 0x6060
    int32_t  main_rx_target_position     = 0;    ///< 0x607A  (raw engineering units)
    int32_t  main_rx_target_velocity     = 0;    ///< 0x60FF  (mrev/s)
    float    main_rx_torque_command      = 0.f;  ///< 0x2022  direct torque (Nm)
    float    main_rx_iq_command          = 0.f;  ///< 0x201A  current quadrature set-point (A)
    float    main_rx_torque_kp           = 0.f;  ///< 0x2523
    float    main_rx_torque_max_out      = 0.f;  ///< 0x2527
    float    main_rx_torque_min_out      = 0.f;  ///< 0x2528
    float    main_rx_vel_kp              = 0.f;  ///< 0x250A
    float    main_rx_vel_ki              = 0.f;  ///< 0x250B
    float    main_rx_vel_kd              = 0.f;  ///< 0x250C
    float    main_rx_pos_kp              = 0.f;  ///< 0x2511
    float    main_rx_pos_ki              = 0.f;  ///< 0x2512
    float    main_rx_pos_kd              = 0.f;  ///< 0x2513
    bool     main_rx_enable              = false;

    // ── DUT — TxPDO (drive → master) ────────────────────────────────────────
    uint16_t dut_tx_statusword           = 0;
    int8_t   dut_tx_mode_display         = 0;
    int32_t  dut_tx_output_enc_pos       = 0;
    float    dut_tx_bus_voltage          = 0.f;
    float    dut_tx_torque_nm            = 0.f;
    float    dut_tx_motor_temp           = 0.f;
    uint16_t dut_tx_error_code           = 0;
    int32_t  dut_tx_motor_velocity       = 0;
    int32_t  dut_tx_input_enc_pos        = 0;
    int32_t  dut_tx_position_setpoint    = 0;
    float    dut_tx_velocity_setpoint    = 0.f;
    float    dut_tx_iq_actual            = 0.f;
    float    dut_tx_id_actual            = 0.f;
    float    dut_tx_idc_actual           = 0.f;
    float    dut_tx_iq_command           = 0.f;
    float    dut_tx_id_command           = 0.f;

    // ── DUT — RxPDO (master → drive) ────────────────────────────────────────
    int8_t   dut_rx_mode_of_operation    = 0;
    int32_t  dut_rx_target_position      = 0;
    int32_t  dut_rx_target_velocity      = 0;
    float    dut_rx_torque_command       = 0.f;
    float    dut_rx_iq_command           = 0.f;
    float    dut_rx_torque_kp            = 0.f;
    float    dut_rx_torque_max_out       = 0.f;
    float    dut_rx_torque_min_out       = 0.f;
    float    dut_rx_vel_kp               = 0.f;
    float    dut_rx_vel_ki               = 0.f;
    float    dut_rx_vel_kd               = 0.f;
    float    dut_rx_pos_kp               = 0.f;
    float    dut_rx_pos_ki               = 0.f;
    float    dut_rx_pos_kd               = 0.f;
    bool     dut_rx_enable               = false;

    // ── Sensors ───────────────────────────────────────────────────────────────
    uint32_t encoder_count   = 0;
    float    torque_ch1_nm   = 0.f;
    float    torque_ch2_nm   = 0.f;
    float    main_gear_ratio = 1.f;
    float    dut_gear_ratio  = 1.f;
};

// CSV column header matching the field order in record_to_csv() (bridge_ros2.cpp).
inline constexpr const char* PDO_LOG_CSV_HEADER =
    "cycle_count,stamp_ns,wkc,cycle_time_ns,dc_error_ns,period_ns,"
    // main tx
    "main_tx_statusword,main_tx_mode_display,main_tx_output_enc_pos,"
    "main_tx_bus_voltage,main_tx_torque_nm,main_tx_motor_temp,main_tx_error_code,"
    "main_tx_motor_velocity,main_tx_input_enc_pos,main_tx_position_setpoint,"
    "main_tx_velocity_setpoint,main_tx_iq_actual,main_tx_id_actual,"
    "main_tx_idc_actual,main_tx_iq_command,main_tx_id_command,"
    // main rx
    "main_rx_mode_of_operation,main_rx_target_position,main_rx_target_velocity,"
    "main_rx_torque_command,main_rx_iq_command,main_rx_torque_kp,main_rx_torque_max_out,main_rx_torque_min_out,"
    "main_rx_vel_kp,main_rx_vel_ki,main_rx_vel_kd,"
    "main_rx_pos_kp,main_rx_pos_ki,main_rx_pos_kd,main_rx_enable,"
    // dut tx
    "dut_tx_statusword,dut_tx_mode_display,dut_tx_output_enc_pos,"
    "dut_tx_bus_voltage,dut_tx_torque_nm,dut_tx_motor_temp,dut_tx_error_code,"
    "dut_tx_motor_velocity,dut_tx_input_enc_pos,dut_tx_position_setpoint,"
    "dut_tx_velocity_setpoint,dut_tx_iq_actual,dut_tx_id_actual,"
    "dut_tx_idc_actual,dut_tx_iq_command,dut_tx_id_command,"
    // dut rx
    "dut_rx_mode_of_operation,dut_rx_target_position,dut_rx_target_velocity,"
    "dut_rx_torque_command,dut_rx_iq_command,dut_rx_torque_kp,dut_rx_torque_max_out,dut_rx_torque_min_out,"
    "dut_rx_vel_kp,dut_rx_vel_ki,dut_rx_vel_kd,"
    "dut_rx_pos_kp,dut_rx_pos_ki,dut_rx_pos_kd,dut_rx_enable,"
    // sensors
    "encoder_count,torque_ch1_nm,torque_ch2_nm,main_gear_ratio,dut_gear_ratio";


// ── Ring buffer ───────────────────────────────────────────────────────────────

/// Thread-safe fixed-depth ring buffer (FIFO).
/// push() drops the oldest record silently when full.
/// pop() returns and removes the oldest record, or nullopt when empty.
template <std::size_t Depth>
class PdoLogBuffer {
    static_assert(Depth > 0, "Depth must be > 0");

public:
    PdoLogBuffer() = default;

    void push(const PdoLogRecord& rec) noexcept {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_[head_] = rec;
        head_ = advance(head_);
        if (count_ < Depth) {
            ++count_;
        } else {
            tail_ = advance(tail_);   // overwrite oldest
        }
    }

    std::optional<PdoLogRecord> pop() noexcept {
        std::lock_guard<std::mutex> lk(mtx_);
        if (count_ == 0) return std::nullopt;
        PdoLogRecord rec = buf_[tail_];
        tail_ = advance(tail_);
        --count_;
        return rec;
    }

    std::size_t size() const noexcept {
        std::lock_guard<std::mutex> lk(mtx_);
        return count_;
    }

    bool empty() const noexcept {
        std::lock_guard<std::mutex> lk(mtx_);
        return count_ == 0;
    }

private:
    static constexpr std::size_t advance(std::size_t i) noexcept {
        return (i + 1 == Depth) ? 0 : i + 1;
    }

    mutable std::mutex              mtx_;
    std::array<PdoLogRecord, Depth> buf_{};
    std::size_t                     head_  = 0;
    std::size_t                     tail_  = 0;
    std::size_t                     count_ = 0;
};

} // namespace dyno
