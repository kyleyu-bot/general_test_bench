#include "dual_novanta_testbench.hpp"

#include "ethercat_core/devices/beckhoff/el2004/data_types.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/data_types.hpp"
#include "ethercat_core/devices/beckhoff/el5032/data_types.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"

extern "C" {
#include "ethercat.h"
}

#include <nlohmann/json.hpp>

#include <any>
#include <cmath>
#include <limits>
#include <sstream>

using namespace ethercat_core;
using namespace ethercat_core::novanta::volcano;
using ModeOfOperation = ethercat_core::ds402::ModeOfOperation;
using WaveformType    = testbench_utils::WaveformType;
using ControlType     = testbench_utils::ControlType;
using json            = nlohmann::json;

// ── Constructor ───────────────────────────────────────────────────────────────

DualNovantaTestbench::DualNovantaTestbench(
    const std::string& drive_slave,
    const std::string& dut_slave,
    const std::string& encoder_slave,
    const std::string& torque_slave,
    const std::string& io_slave,
    bool               dut_present,
    beckhoff::elm3002::Elm3002Adapter* elm3002,
    int  drive_soem_idx,
    int  dut_soem_idx,
    int  main_out_enc_bits,
    int  dut_out_enc_bits)
    : drive_slave_(drive_slave)
    , dut_slave_(dut_slave)
    , encoder_slave_(encoder_slave)
    , torque_slave_(torque_slave)
    , io_slave_(io_slave)
    , dut_present_(dut_present)
    , elm3002_(elm3002)
    , drive_soem_idx_(drive_soem_idx)
    , dut_soem_idx_(dut_soem_idx)
    , main_out_enc_bits_(main_out_enc_bits)
    , dut_out_enc_bits_(dut_out_enc_bits)
{}

// ── Private: gain extraction ──────────────────────────────────────────────────

DriveGains DualNovantaTestbench::extractGains_(
    const MasterRuntime& rt, const std::string& slave_name) const
{
    DriveGains g;
    auto it = rt.startup_params.find(slave_name);
    if (it == rt.startup_params.end()) return g;
    const auto& p = it->second;
    auto get = [&](const char* k) -> float {
        auto pit = p.find(k); return pit != p.end() ? pit->second : 0.0f;
    };
    const float kt = get("motor_kt");
    g.torque_kp              = (std::abs(kt) > 1e-9f) ? (1.0f / kt) : 0.0f;
    g.torque_loop_max_output = get("torque_loop_max_output");
    g.torque_loop_min_output = get("torque_loop_min_output");
    g.velocity_loop_kp       = get("velocity_loop_kp");
    g.velocity_loop_ki       = get("velocity_loop_ki");
    g.velocity_loop_kd       = get("velocity_loop_kd");
    g.position_loop_kp       = get("position_loop_kp");
    g.position_loop_ki       = get("position_loop_ki");
    g.position_loop_kd       = get("position_loop_kd");
    g.max_current_a          = get("max_current");
    return g;
}

// ── extractAndSeedGains ───────────────────────────────────────────────────────

void DualNovantaTestbench::extractAndSeedGains(
    MasterRuntime& rt,
    CommandState&  cmd_state,
    std::mutex&    cmd_mutex)
{
    const DriveGains main_gains = extractGains_(rt, drive_slave_);
    const DriveGains dut_gains  = extractGains_(rt, dut_slave_);

    // sensor_ratio = (rev at position sensor) / (rev at velocity sensor)
    // gear_ratio   = 1 / sensor_ratio
    auto read_gear_ratio = [&](const std::string& slave) -> float {
        auto it = rt.startup_params.find(slave);
        if (it == rt.startup_params.end()) return 1.f;
        auto pit = it->second.find("sensor_ratio");
        if (pit == it->second.end() || std::abs(pit->second) < 1e-6f) return 1.f;
        return 1.f / pit->second;
    };
    main_gear_ratio_ = read_gear_ratio(drive_slave_);
    dut_gear_ratio_  = read_gear_ratio(dut_slave_);

    std::lock_guard<std::mutex> lk(cmd_mutex);
    cmd_state.main_torque_kp      = main_gains.torque_kp;
    cmd_state.main_torque_max_out = main_gains.torque_loop_max_output;
    cmd_state.main_torque_min_out = main_gains.torque_loop_min_output;
    cmd_state.main_vel_kp         = main_gains.velocity_loop_kp;
    cmd_state.main_vel_ki         = main_gains.velocity_loop_ki;
    cmd_state.main_vel_kd         = main_gains.velocity_loop_kd;
    cmd_state.main_pos_kp         = main_gains.position_loop_kp;
    cmd_state.main_pos_ki         = main_gains.position_loop_ki;
    cmd_state.main_pos_kd         = main_gains.position_loop_kd;
    cmd_state.main_max_current_a  = main_gains.max_current_a;
    cmd_state.dut_torque_kp       = dut_gains.torque_kp;
    cmd_state.dut_torque_max_out  = dut_gains.torque_loop_max_output;
    cmd_state.dut_torque_min_out  = dut_gains.torque_loop_min_output;
    cmd_state.dut_vel_kp          = dut_gains.velocity_loop_kp;
    cmd_state.dut_vel_ki          = dut_gains.velocity_loop_ki;
    cmd_state.dut_vel_kd          = dut_gains.velocity_loop_kd;
    cmd_state.dut_pos_kp          = dut_gains.position_loop_kp;
    cmd_state.dut_pos_ki          = dut_gains.position_loop_ki;
    cmd_state.dut_pos_kd          = dut_gains.position_loop_kd;
    cmd_state.dut_max_current_a   = dut_gains.max_current_a;
}

// ── makeCallback ──────────────────────────────────────────────────────────────

EthercatLoop::CycleCallback DualNovantaTestbench::makeCallback(
    CommandState&  cmd_state,
    std::mutex&    cmd_mutex,
    dyno::PdoLogBuffer<200>& log_buf,
    std::chrono::steady_clock::time_point reset_end)
{
    return [this, &cmd_state, &cmd_mutex, &log_buf, reset_end]
        (const SystemStatus& status, const LoopStats& stats) -> SystemCommand
    {
        const bool in_reset = std::chrono::steady_clock::now() < reset_end;

        CommandState cmd;
        { std::lock_guard<std::mutex> lk(cmd_mutex); cmd = cmd_state; }

        static constexpr float TWO_PI = 2.0f * static_cast<float>(M_PI);
        Command main_cmd;
        main_cmd.mode_of_operation       = static_cast<ModeOfOperation>(cmd.main_mode);
        main_cmd.target_velocity_mrevs    = cmd.main_speed * 1000.0f / TWO_PI;
        main_cmd.target_position__enc_cnt = cmd.main_position
                                            * static_cast<float>(1LL << main_out_enc_bits_) / TWO_PI;
        main_cmd.target_torque_nm        = cmd.main_torque;
        main_cmd.torque_command_2022     = cmd.main_torque;
        main_cmd.iq_setpoint_a           = cmd.main_current;
        main_cmd.enable_drive            = !in_reset && cmd.main_enable;
        main_cmd.clear_fault             = in_reset || cmd.fault_reset;
        main_cmd.torque_kp               = cmd.main_torque_kp;
        main_cmd.torque_loop_max_output  = cmd.main_torque_max_out;
        main_cmd.torque_loop_min_output  = cmd.main_torque_min_out;
        main_cmd.velocity_loop_kp        = cmd.main_vel_kp;
        main_cmd.velocity_loop_ki        = cmd.main_vel_ki;
        main_cmd.velocity_loop_kd        = cmd.main_vel_kd;
        main_cmd.position_loop_kp        = cmd.main_pos_kp;
        main_cmd.position_loop_ki        = cmd.main_pos_ki;
        main_cmd.position_loop_kd        = cmd.main_pos_kd;

        Command dut_cmd;
        dut_cmd.mode_of_operation       = static_cast<ModeOfOperation>(cmd.dut_mode);
        dut_cmd.target_velocity_mrevs    = cmd.dut_speed * 1000.0f / TWO_PI;
        dut_cmd.target_position__enc_cnt = cmd.dut_position
                                           * static_cast<float>(1LL << dut_out_enc_bits_) / TWO_PI;
        dut_cmd.target_torque_nm        = cmd.dut_torque;
        dut_cmd.torque_command_2022     = cmd.dut_torque;
        dut_cmd.iq_setpoint_a           = cmd.dut_current;
        dut_cmd.enable_drive            = !in_reset && cmd.dut_enable;
        dut_cmd.clear_fault             = in_reset || cmd.fault_reset;
        dut_cmd.torque_kp               = cmd.dut_torque_kp;
        dut_cmd.torque_loop_max_output  = cmd.dut_torque_max_out;
        dut_cmd.torque_loop_min_output  = cmd.dut_torque_min_out;
        dut_cmd.velocity_loop_kp        = cmd.dut_vel_kp;
        dut_cmd.velocity_loop_ki        = cmd.dut_vel_ki;
        dut_cmd.velocity_loop_kd        = cmd.dut_vel_kd;
        dut_cmd.position_loop_kp        = cmd.dut_pos_kp;
        dut_cmd.position_loop_ki        = cmd.dut_pos_ki;
        dut_cmd.position_loop_kd        = cmd.dut_pos_kd;

        // ── Function generator ─────────────────────────────────────────────────
        const double dt_s = (stats.last_period_ns > 0)
                            ? static_cast<double>(stats.last_period_ns) * 1e-9 : 1e-3;

        fg_main_.setWaveformType(static_cast<WaveformType>(cmd.main_fg_waveform));
        fg_main_.setControlType(static_cast<ControlType>(cmd.main_fg_control_type));
        fg_main_.setAmplitude(cmd.main_fg_amplitude);
        fg_main_.setFrequency(cmd.main_fg_frequency);
        fg_main_.setOffset(cmd.main_fg_offset);
        fg_main_.setPhase(cmd.main_fg_phase);
        fg_main_.setChirpLowFrequency(cmd.main_fg_chirp_f_low);
        fg_main_.setChirpHighFrequency(cmd.main_fg_chirp_f_high);
        fg_main_.setChirpDuration(cmd.main_fg_chirp_dur);
        if (cmd.main_fg_enable && !fg_main_.isEnabled()) {
            if (static_cast<ControlType>(cmd.main_fg_control_type) == ControlType::POSITION) {
                auto it = status.by_slave.find(drive_slave_);
                if (it != status.by_slave.end() && it->second.has_value())
                    main_captured_pos_ = std::any_cast<const DriveStatus&>(it->second)
                                             .measured_output_side_position_raw_cnt;
            }
            fg_main_.enable();
        } else if (!cmd.main_fg_enable && fg_main_.isEnabled()) {
            fg_main_.stop(false);
            main_cmd.mode_of_operation = ModeOfOperation::NO_MODE;
        }
        fg_main_.update(dt_s);

        fg_dut_.setWaveformType(static_cast<WaveformType>(cmd.dut_fg_waveform));
        fg_dut_.setControlType(static_cast<ControlType>(cmd.dut_fg_control_type));
        fg_dut_.setAmplitude(cmd.dut_fg_amplitude);
        fg_dut_.setFrequency(cmd.dut_fg_frequency);
        fg_dut_.setOffset(cmd.dut_fg_offset);
        fg_dut_.setPhase(cmd.dut_fg_phase);
        fg_dut_.setChirpLowFrequency(cmd.dut_fg_chirp_f_low);
        fg_dut_.setChirpHighFrequency(cmd.dut_fg_chirp_f_high);
        fg_dut_.setChirpDuration(cmd.dut_fg_chirp_dur);
        if (cmd.dut_fg_enable && !fg_dut_.isEnabled()) {
            if (static_cast<ControlType>(cmd.dut_fg_control_type) == ControlType::POSITION) {
                auto it = status.by_slave.find(dut_slave_);
                if (it != status.by_slave.end() && it->second.has_value())
                    dut_captured_pos_ = std::any_cast<const DriveStatus&>(it->second)
                                            .measured_output_side_position_raw_cnt;
            }
            fg_dut_.enable();
        } else if (!cmd.dut_fg_enable && fg_dut_.isEnabled()) {
            fg_dut_.stop(false);
            dut_cmd.mode_of_operation = ModeOfOperation::NO_MODE;
        }
        fg_dut_.update(dt_s);

        // Determine effective mode (FG control type may override bridge mode)
        auto eff_mode = [](ModeOfOperation base, const testbench_utils::FunctionGenerator& fg) {
            if (!fg.isEnabled()) return base;
            switch (fg.getControlType()) {
            case ControlType::POSITION: return ModeOfOperation::CYCLIC_SYNC_POSITION;
            case ControlType::VELOCITY: return ModeOfOperation::CYCLIC_SYNC_VELOCITY;
            case ControlType::TORQUE:
            case ControlType::CURRENT:  return ModeOfOperation::CURRENT;
            default:                    return base;
            }
        };
        const ModeOfOperation eff_main = eff_mode(main_cmd.mode_of_operation, fg_main_);
        const ModeOfOperation eff_dut  = eff_mode(dut_cmd.mode_of_operation,  fg_dut_);

        // Position mode transition → capture current output encoder position
        auto is_pos_mode = [](ModeOfOperation m) {
            return m == ModeOfOperation::CYCLIC_SYNC_POSITION ||
                   m == ModeOfOperation::PROFILE_POSITION;
        };
        if (is_pos_mode(eff_main) &&
            !is_pos_mode(static_cast<ModeOfOperation>(prev_main_mode_))) {
            auto it = status.by_slave.find(drive_slave_);
            if (it != status.by_slave.end() && it->second.has_value())
                main_captured_pos_ = std::any_cast<const DriveStatus&>(it->second)
                                         .measured_output_side_position_raw_cnt;
        }
        if (is_pos_mode(eff_dut) &&
            !is_pos_mode(static_cast<ModeOfOperation>(prev_dut_mode_))) {
            auto it = status.by_slave.find(dut_slave_);
            if (it != status.by_slave.end() && it->second.has_value())
                dut_captured_pos_ = std::any_cast<const DriveStatus&>(it->second)
                                        .measured_output_side_position_raw_cnt;
        }
        prev_main_mode_ = static_cast<int8_t>(eff_main);
        prev_dut_mode_  = static_cast<int8_t>(eff_dut);

        // Apply FG override to drive command fields
        auto apply_fg = [&](Command& drive_cmd,
                            const testbench_utils::FunctionGenerator& fg,
                            int32_t captured_pos, int out_enc_bits) {
            if (!fg.isEnabled()) return;
            const double val = fg.getValue();
            switch (fg.getControlType()) {
            case ControlType::POSITION:
                drive_cmd.target_position__enc_cnt =
                    static_cast<float>(captured_pos) +
                    static_cast<float>(val * static_cast<double>(1LL << out_enc_bits) / (2.0 * M_PI));
                drive_cmd.mode_of_operation = ModeOfOperation::CYCLIC_SYNC_POSITION;
                break;
            case ControlType::VELOCITY:
                drive_cmd.target_velocity_mrevs =
                    static_cast<float>(val * 1000.0 / (2.0 * M_PI));
                drive_cmd.mode_of_operation = ModeOfOperation::CYCLIC_SYNC_VELOCITY;
                break;
            case ControlType::TORQUE:
                drive_cmd.target_torque_nm = static_cast<float>(val);
                drive_cmd.mode_of_operation = ModeOfOperation::CYCLIC_SYNC_TORQUE;
                break;
            case ControlType::CURRENT:
                drive_cmd.iq_setpoint_a = static_cast<float>(val);
                drive_cmd.mode_of_operation = ModeOfOperation::CURRENT;
                break;
            default: break;
            }
        };
        apply_fg(main_cmd, fg_main_, main_captured_pos_, main_out_enc_bits_);
        apply_fg(dut_cmd,  fg_dut_,  dut_captured_pos_,  dut_out_enc_bits_);

        last_main_fg_out_.store(fg_main_.isEnabled()
            ? static_cast<float>(fg_main_.getValue()) : 0.f,
            std::memory_order_relaxed);
        last_dut_fg_out_.store(fg_dut_.isEnabled()
            ? static_cast<float>(fg_dut_.getValue()) : 0.f,
            std::memory_order_relaxed);

        beckhoff::el2004::Command io_cmd;
        io_cmd.output_1 = cmd.hold_output1;

        SystemCommand sys_cmd;
        sys_cmd.by_slave[drive_slave_] = main_cmd;
        if (dut_present_) sys_cmd.by_slave[dut_slave_] = dut_cmd;
        sys_cmd.by_slave[io_slave_]    = io_cmd;

        // Capture per-cycle log record from the exact data of this RT cycle.
        dyno::PdoLogRecord rec;
        rec.cycle_count   = stats.cycle_count;
        rec.stamp_ns      = status.stamp_ns;
        rec.wkc           = stats.last_wkc;
        rec.cycle_time_ns = stats.last_cycle_time_ns;
        rec.dc_error_ns   = stats.last_dc_error_ns;
        rec.period_ns     = stats.last_period_ns;

        auto fill_tx = [&](const DriveStatus& ds, bool is_main) {
            if (is_main) {
                rec.main_tx_statusword        = ds.status_word;
                rec.main_tx_mode_display      = ds.mode_of_operation_display;
                rec.main_tx_output_enc_pos    = ds.measured_output_side_position_raw_cnt;
                rec.main_tx_bus_voltage       = ds.bus_voltage;
                rec.main_tx_torque_nm         = ds.measured_torque_nm;
                rec.main_tx_motor_temp        = ds.motor_temp;
                rec.main_tx_error_code        = ds.error_code;
                rec.main_tx_motor_velocity    = ds.measured_input_side_velocity_raw;
                rec.main_tx_input_enc_pos     = ds.input_encoder_pos;
                rec.main_tx_position_setpoint = ds.position_setpoint;
                rec.main_tx_velocity_setpoint = ds.velocity_command_received;
                rec.main_tx_iq_actual         = ds.iq_actual;
                rec.main_tx_id_actual         = ds.id_actual;
                rec.main_tx_idc_actual        = ds.idc_actual;
                rec.main_tx_iq_command        = ds.iq_command;
                rec.main_tx_id_command        = ds.id_command;
            } else {
                rec.dut_tx_statusword         = ds.status_word;
                rec.dut_tx_mode_display       = ds.mode_of_operation_display;
                rec.dut_tx_output_enc_pos     = ds.measured_output_side_position_raw_cnt;
                rec.dut_tx_bus_voltage        = ds.bus_voltage;
                rec.dut_tx_torque_nm          = ds.measured_torque_nm;
                rec.dut_tx_motor_temp         = ds.motor_temp;
                rec.dut_tx_error_code         = ds.error_code;
                rec.dut_tx_motor_velocity     = ds.measured_input_side_velocity_raw;
                rec.dut_tx_input_enc_pos      = ds.input_encoder_pos;
                rec.dut_tx_position_setpoint  = ds.position_setpoint;
                rec.dut_tx_velocity_setpoint  = ds.velocity_command_received;
                rec.dut_tx_iq_actual          = ds.iq_actual;
                rec.dut_tx_id_actual          = ds.id_actual;
                rec.dut_tx_idc_actual         = ds.idc_actual;
                rec.dut_tx_iq_command         = ds.iq_command;
                rec.dut_tx_id_command         = ds.id_command;
            }
        };

        auto main_log_it = status.by_slave.find(drive_slave_);
        if (main_log_it != status.by_slave.end() && main_log_it->second.has_value())
            fill_tx(std::any_cast<const DriveStatus&>(main_log_it->second), true);

        auto dut_log_it = status.by_slave.find(dut_slave_);
        if (dut_present_ && dut_log_it != status.by_slave.end() && dut_log_it->second.has_value())
            fill_tx(std::any_cast<const DriveStatus&>(dut_log_it->second), false);

        // RxPDO — log the final command structs (after FG overrides) so recorded values
        // match what was actually transmitted to the drives.
        rec.main_rx_mode_of_operation = static_cast<int8_t>(main_cmd.mode_of_operation);
        rec.main_rx_target_position   = static_cast<int32_t>(main_cmd.target_position__enc_cnt);
        rec.main_rx_target_velocity   = static_cast<int32_t>(main_cmd.target_velocity_mrevs);
        rec.main_rx_torque_command    = main_cmd.target_torque_nm;
        rec.main_rx_iq_command        = main_cmd.iq_setpoint_a;
        rec.main_rx_torque_kp         = main_cmd.torque_kp;
        rec.main_rx_torque_max_out    = main_cmd.torque_loop_max_output;
        rec.main_rx_torque_min_out    = main_cmd.torque_loop_min_output;
        rec.main_rx_vel_kp            = main_cmd.velocity_loop_kp;
        rec.main_rx_vel_ki            = main_cmd.velocity_loop_ki;
        rec.main_rx_vel_kd            = main_cmd.velocity_loop_kd;
        rec.main_rx_pos_kp            = main_cmd.position_loop_kp;
        rec.main_rx_pos_ki            = main_cmd.position_loop_ki;
        rec.main_rx_pos_kd            = main_cmd.position_loop_kd;
        rec.main_rx_enable            = cmd.main_enable;
        rec.dut_rx_mode_of_operation  = static_cast<int8_t>(dut_cmd.mode_of_operation);
        rec.dut_rx_target_position    = static_cast<int32_t>(dut_cmd.target_position__enc_cnt);
        rec.dut_rx_target_velocity    = static_cast<int32_t>(dut_cmd.target_velocity_mrevs);
        rec.dut_rx_torque_command     = dut_cmd.target_torque_nm;
        rec.dut_rx_iq_command         = dut_cmd.iq_setpoint_a;
        rec.dut_rx_torque_kp          = dut_cmd.torque_kp;
        rec.dut_rx_torque_max_out     = dut_cmd.torque_loop_max_output;
        rec.dut_rx_torque_min_out     = dut_cmd.torque_loop_min_output;
        rec.dut_rx_vel_kp             = dut_cmd.velocity_loop_kp;
        rec.dut_rx_vel_ki             = dut_cmd.velocity_loop_ki;
        rec.dut_rx_vel_kd             = dut_cmd.velocity_loop_kd;
        rec.dut_rx_pos_kp             = dut_cmd.position_loop_kp;
        rec.dut_rx_pos_ki             = dut_cmd.position_loop_ki;
        rec.dut_rx_pos_kd             = dut_cmd.position_loop_kd;
        rec.dut_rx_enable             = cmd.dut_enable;

        auto enc_it = status.by_slave.find(encoder_slave_);
        if (enc_it != status.by_slave.end() && enc_it->second.has_value())
            rec.encoder_count = std::any_cast<const beckhoff::el5032::Data&>(
                enc_it->second).encoder_count_25bit;

        auto torque_it = status.by_slave.find(torque_slave_);
        if (torque_it != status.by_slave.end() && torque_it->second.has_value()) {
            const auto& d = std::any_cast<const beckhoff::elm3002::Data&>(torque_it->second);
            rec.torque_ch1_nm = elm3002_->scaledTorqueCh1(d);
            rec.torque_ch2_nm = elm3002_->scaledTorqueCh2(d);
        }

        rec.main_gear_ratio = main_gear_ratio_;
        rec.dut_gear_ratio  = dut_gear_ratio_;

        log_buf.push(rec);
        return sys_cmd;
    };
}

// ── serializeToCsvRow ─────────────────────────────────────────────────────────

std::string DualNovantaTestbench::serializeToCsvRow(const dyno::PdoLogRecord& r)
{
    std::ostringstream o;
    // metadata
    o << r.cycle_count   << ',' << r.stamp_ns      << ',' << r.wkc         << ','
      << r.cycle_time_ns << ',' << r.dc_error_ns   << ',' << r.period_ns   << ',';
    // main tx
    o << r.main_tx_statusword        << ',' << static_cast<int>(r.main_tx_mode_display)  << ','
      << r.main_tx_output_enc_pos    << ',' << r.main_tx_bus_voltage        << ','
      << r.main_tx_torque_nm         << ',' << r.main_tx_motor_temp         << ','
      << r.main_tx_error_code        << ',' << r.main_tx_motor_velocity     << ','
      << r.main_tx_input_enc_pos     << ',' << r.main_tx_position_setpoint  << ','
      << r.main_tx_velocity_setpoint << ',' << r.main_tx_iq_actual          << ','
      << r.main_tx_id_actual         << ',' << r.main_tx_idc_actual         << ','
      << r.main_tx_iq_command        << ',' << r.main_tx_id_command         << ',';
    // main rx
    o << static_cast<int>(r.main_rx_mode_of_operation) << ','
      << r.main_rx_target_position   << ',' << r.main_rx_target_velocity    << ','
      << r.main_rx_torque_command    << ',' << r.main_rx_iq_command         << ','
      << r.main_rx_torque_kp         << ','
      << r.main_rx_torque_max_out    << ',' << r.main_rx_torque_min_out     << ','
      << r.main_rx_vel_kp            << ',' << r.main_rx_vel_ki             << ','
      << r.main_rx_vel_kd            << ',' << r.main_rx_pos_kp             << ','
      << r.main_rx_pos_ki            << ',' << r.main_rx_pos_kd             << ','
      << static_cast<int>(r.main_rx_enable) << ',';
    // dut tx
    o << r.dut_tx_statusword         << ',' << static_cast<int>(r.dut_tx_mode_display)   << ','
      << r.dut_tx_output_enc_pos     << ',' << r.dut_tx_bus_voltage         << ','
      << r.dut_tx_torque_nm          << ',' << r.dut_tx_motor_temp          << ','
      << r.dut_tx_error_code         << ',' << r.dut_tx_motor_velocity      << ','
      << r.dut_tx_input_enc_pos      << ',' << r.dut_tx_position_setpoint   << ','
      << r.dut_tx_velocity_setpoint  << ',' << r.dut_tx_iq_actual           << ','
      << r.dut_tx_id_actual          << ',' << r.dut_tx_idc_actual          << ','
      << r.dut_tx_iq_command         << ',' << r.dut_tx_id_command          << ',';
    // dut rx
    o << static_cast<int>(r.dut_rx_mode_of_operation) << ','
      << r.dut_rx_target_position    << ',' << r.dut_rx_target_velocity     << ','
      << r.dut_rx_torque_command     << ',' << r.dut_rx_iq_command          << ','
      << r.dut_rx_torque_kp          << ','
      << r.dut_rx_torque_max_out     << ',' << r.dut_rx_torque_min_out      << ','
      << r.dut_rx_vel_kp             << ',' << r.dut_rx_vel_ki              << ','
      << r.dut_rx_vel_kd             << ',' << r.dut_rx_pos_kp              << ','
      << r.dut_rx_pos_ki             << ',' << r.dut_rx_pos_kd              << ','
      << static_cast<int>(r.dut_rx_enable) << ',';
    // sensors
    o << r.encoder_count  << ',' << r.torque_ch1_nm   << ',' << r.torque_ch2_nm << ','
      << r.main_gear_ratio << ',' << r.dut_gear_ratio;
    return o.str();
}

// ── makeDriveJson ─────────────────────────────────────────────────────────────

std::string DualNovantaTestbench::makeDriveJson(
    const std::string& slave_name,
    int   soem_idx,
    const SystemStatus& status,
    int   out_enc_bits,
    const DriveGains& gains)
{
    json j;
    const int al_raw = (soem_idx >= 1) ? static_cast<int>(ec_slave[soem_idx].state) : 0;
    j["al"]     = alStateName(al_raw);
    j["al_num"] = al_raw & 0x0F;
    auto it = status.by_slave.find(slave_name);
    if (it != status.by_slave.end() && it->second.has_value()) {
        const auto& ds = std::any_cast<const DriveStatus&>(it->second);
        const double enc_to_rad = 2.0 * M_PI / static_cast<double>(1LL << out_enc_bits);
        j["state"]              = cia402Name(ds.cia402_state);
        j["cmd_vel_rad_per_s"]  = static_cast<double>(ds.velocity_command_received) * (2.0 * M_PI);
        j["cmd_vel_rev_per_s"]  = static_cast<double>(ds.velocity_command_received);
        j["fb_vel_raw"]         = ds.measured_input_side_velocity_raw;
        j["fb_vel_rad_per_s"]   = static_cast<double>(ds.measured_input_side_velocity_raw)
                                  * (2.0 * M_PI / 1000.0);
        j["mode"]               = static_cast<int>(ds.mode_of_operation_display);
        j["sw"]                 = ds.status_word;
        j["err"]                = ds.error_code;
        j["output_enc_pos_raw_cnt"] = ds.measured_output_side_position_raw_cnt;
        j["output_pos_rad"]     = static_cast<double>(ds.measured_output_side_position_raw_cnt)
                                  * enc_to_rad;
        j["in_enc_pos"]         = ds.input_encoder_pos;
        j["pos_setpoint_raw_enc_cnt"] = ds.position_setpoint;
        j["pos_setpoint_rad"]   = static_cast<double>(ds.position_setpoint) * enc_to_rad;
        j["fb_torque"]          = ds.measured_torque_nm;
        j["bus_voltage"]        = ds.bus_voltage;
        j["motor_temp"]         = ds.motor_temp;
        j["iq_actual"]          = ds.iq_actual;
        j["id_actual"]          = ds.id_actual;
        j["idc_actual"]         = ds.idc_actual;
        j["iq_command"]         = ds.iq_command;
        j["id_command"]         = ds.id_command;
        // Limits in natural units for GUI slider ranging
        j["max_velocity_abs_rad_s"] = static_cast<double>(ds.max_velocity_abs)
                                      * (2.0 * M_PI / 1000.0);
        j["min_position_rad"]   = (ds.min_position == std::numeric_limits<int32_t>::min())
            ? -1000.0 : static_cast<double>(ds.min_position) * enc_to_rad;
        j["max_position_rad"]   = (ds.max_position == std::numeric_limits<int32_t>::max())
            ?  1000.0 : static_cast<double>(ds.max_position) * enc_to_rad;
        // Keep raw limits for backward compat
        j["max_velocity_abs"]   = ds.max_velocity_abs;
        j["min_position"]       = ds.min_position;
        j["max_position"]       = ds.max_position;
        j["max_current_a"]      = static_cast<double>(gains.max_current_a);
        // Control gains (current values being sent to drive)
        j["torque_kp"]  = static_cast<double>(gains.torque_kp);
        j["torque_max"] = static_cast<double>(gains.torque_loop_max_output);
        j["torque_min"] = static_cast<double>(gains.torque_loop_min_output);
        j["vel_kp"]     = static_cast<double>(gains.velocity_loop_kp);
        j["vel_ki"]     = static_cast<double>(gains.velocity_loop_ki);
        j["vel_kd"]     = static_cast<double>(gains.velocity_loop_kd);
        j["pos_kp"]     = static_cast<double>(gains.position_loop_kp);
        j["pos_ki"]     = static_cast<double>(gains.position_loop_ki);
        j["pos_kd"]     = static_cast<double>(gains.position_loop_kd);
    } else {
        j["state"] = "unavailable";
    }
    return j.dump();
}

// ── printDebug ────────────────────────────────────────────────────────────────

void DualNovantaTestbench::printDebug(
    const SystemStatus& status,
    const LoopStats&    stats,
    const CommandState& cmd,
    uint32_t enc,
    double ch1_t, double ch2_t) const
{
    // main_drive
    auto main_it = status.by_slave.find(drive_slave_);
    if (main_it != status.by_slave.end() && main_it->second.has_value()) {
        const auto& ds = std::any_cast<const DriveStatus&>(main_it->second);
        std::printf(
            "[main] cycle=%lu wkc=%d "
            "al=%s state=%s "
            "cmd_60FF=%.3f speed_606C=%d "
            "mode_6061=%d sw=0x%04X err=0x%04X "
            "bus_v=%.2f "
            "vel_kp=%.4f vel_ki=%.4f torque_kp=%.4f\n",
            static_cast<unsigned long>(stats.cycle_count),
            stats.last_wkc,
            alStateName(static_cast<int>(ec_slave[drive_soem_idx_].state)).c_str(),
            cia402Name(ds.cia402_state),
            static_cast<double>(cmd.main_speed),
            ds.measured_input_side_velocity_raw,
            static_cast<int>(ds.mode_of_operation_display),
            static_cast<unsigned>(ds.status_word),
            static_cast<unsigned>(ds.error_code),
            static_cast<double>(ds.bus_voltage),
            static_cast<double>(cmd.main_vel_kp),
            static_cast<double>(cmd.main_vel_ki),
            static_cast<double>(cmd.main_torque_kp)
        );
    }
    // dut
    auto dut_it = status.by_slave.find(dut_slave_);
    if (dut_present_ && dut_it != status.by_slave.end() && dut_it->second.has_value()) {
        const auto& ds = std::any_cast<const DriveStatus&>(dut_it->second);
        std::printf(
            "[ dut] cycle=%lu wkc=%d "
            "al=%s state=%s "
            "cmd_60FF=%.3f speed_606C=%d "
            "mode_6061=%d sw=0x%04X err=0x%04X "
            "bus_v=%.2f "
            "vel_kp=%.4f vel_ki=%.4f torque_kp=%.4f\n",
            static_cast<unsigned long>(stats.cycle_count),
            stats.last_wkc,
            alStateName(dut_present_ ? static_cast<int>(ec_slave[dut_soem_idx_].state) : 0).c_str(),
            cia402Name(ds.cia402_state),
            static_cast<double>(cmd.dut_speed),
            ds.measured_input_side_velocity_raw,
            static_cast<int>(ds.mode_of_operation_display),
            static_cast<unsigned>(ds.status_word),
            static_cast<unsigned>(ds.error_code),
            static_cast<double>(ds.bus_voltage),
            static_cast<double>(cmd.dut_vel_kp),
            static_cast<double>(cmd.dut_vel_ki),
            static_cast<double>(cmd.dut_torque_kp)
        );
    }
    // encoder + torque
    std::printf(
        "[sens] enc=%u ch1_t=%.4f ch2_t=%.4f\n",
        enc, ch1_t, ch2_t
    );
    std::printf(
        "[timing] rt_period=%.3f ms | rt_cycle=%.3f ms | wakeup_lat=%.3f ms\n",
        static_cast<double>(stats.last_period_ns)         * 1e-6,
        static_cast<double>(stats.last_cycle_time_ns)     * 1e-6,
        static_cast<double>(stats.last_wakeup_latency_ns) * 1e-6
    );
}
