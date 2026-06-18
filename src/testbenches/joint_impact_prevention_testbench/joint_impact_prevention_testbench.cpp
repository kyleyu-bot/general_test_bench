#include "joint_impact_prevention_testbench.hpp"

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
using json            = nlohmann::json;

// ── Constructor ───────────────────────────────────────────────────────────────

JointImpactPreventionTestbench::JointImpactPreventionTestbench(
    const std::string& drive_slave,
    int                drive_soem_idx,
    int                main_out_enc_bits)
    : drive_slave_(drive_slave)
    , drive_soem_idx_(drive_soem_idx)
    , main_out_enc_bits_(main_out_enc_bits)
{}

// ── Private: gain extraction ──────────────────────────────────────────────────

DriveGains JointImpactPreventionTestbench::extractGains_(
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

void JointImpactPreventionTestbench::extractAndSeedGains(
    MasterRuntime& rt,
    CommandState&  cmd_state,
    std::mutex&    cmd_mutex)
{
    const DriveGains gains = extractGains_(rt, drive_slave_);

    std::lock_guard<std::mutex> lk(cmd_mutex);
    cmd_state.main_torque_kp      = gains.torque_kp;
    cmd_state.main_torque_max_out = gains.torque_loop_max_output;
    cmd_state.main_torque_min_out = gains.torque_loop_min_output;
    cmd_state.main_vel_kp         = gains.velocity_loop_kp;
    cmd_state.main_vel_ki         = gains.velocity_loop_ki;
    cmd_state.main_vel_kd         = gains.velocity_loop_kd;
    cmd_state.main_pos_kp         = gains.position_loop_kp;
    cmd_state.main_pos_ki         = gains.position_loop_ki;
    cmd_state.main_pos_kd         = gains.position_loop_kd;
    cmd_state.main_max_current_a  = gains.max_current_a;
}

// ── makeCallback ──────────────────────────────────────────────────────────────

EthercatLoop::CycleCallback JointImpactPreventionTestbench::makeCallback(
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

        SystemCommand sys_cmd;
        sys_cmd.by_slave[drive_slave_] = main_cmd;

        // Capture per-cycle log record.
        dyno::PdoLogRecord rec;
        rec.cycle_count   = stats.cycle_count;
        rec.stamp_ns      = status.stamp_ns;
        rec.wkc           = stats.last_wkc;
        rec.cycle_time_ns = stats.last_cycle_time_ns;
        rec.dc_error_ns   = stats.last_dc_error_ns;
        rec.period_ns     = stats.last_period_ns;

        auto it = status.by_slave.find(drive_slave_);
        if (it != status.by_slave.end() && it->second.has_value()) {
            const auto& ds = std::any_cast<const DriveStatus&>(it->second);

            const float velocity_rad_s = static_cast<float>(ds.measured_input_side_velocity_raw)
                                         * (TWO_PI / 1000.0f);
            const float position_rad   = static_cast<float>(ds.measured_output_side_position_raw_cnt)
                                         * (TWO_PI / static_cast<float>(1LL << main_out_enc_bits_));

            braking_.read(velocity_rad_s, position_rad,
                          cmd.hardstop_pos_upper, cmd.hardstop_pos_lower,
                          cmd.margin, cmd.inertia);
            braking_.write(sys_cmd, drive_slave_);

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
        }

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

        rec.jip_inertia            = cmd.inertia;
        rec.jip_hardstop_pos_upper = cmd.hardstop_pos_upper;
        rec.jip_hardstop_pos_lower = cmd.hardstop_pos_lower;
        rec.jip_margin             = cmd.margin;

        log_buf.push(rec);
        return sys_cmd;
    };
}

// ── serializeToCsvRow ─────────────────────────────────────────────────────────

std::string JointImpactPreventionTestbench::serializeToCsvRow(const dyno::PdoLogRecord& r)
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
    // dut tx (all zero — single-drive testbench)
    o << r.dut_tx_statusword         << ',' << static_cast<int>(r.dut_tx_mode_display)   << ','
      << r.dut_tx_output_enc_pos     << ',' << r.dut_tx_bus_voltage         << ','
      << r.dut_tx_torque_nm          << ',' << r.dut_tx_motor_temp          << ','
      << r.dut_tx_error_code         << ',' << r.dut_tx_motor_velocity      << ','
      << r.dut_tx_input_enc_pos      << ',' << r.dut_tx_position_setpoint   << ','
      << r.dut_tx_velocity_setpoint  << ',' << r.dut_tx_iq_actual           << ','
      << r.dut_tx_id_actual          << ',' << r.dut_tx_idc_actual          << ','
      << r.dut_tx_iq_command         << ',' << r.dut_tx_id_command          << ',';
    // dut rx (all zero)
    o << static_cast<int>(r.dut_rx_mode_of_operation) << ','
      << r.dut_rx_target_position    << ',' << r.dut_rx_target_velocity     << ','
      << r.dut_rx_torque_command     << ',' << r.dut_rx_iq_command          << ','
      << r.dut_rx_torque_kp          << ','
      << r.dut_rx_torque_max_out     << ',' << r.dut_rx_torque_min_out      << ','
      << r.dut_rx_vel_kp             << ',' << r.dut_rx_vel_ki              << ','
      << r.dut_rx_vel_kd             << ',' << r.dut_rx_pos_kp              << ','
      << r.dut_rx_pos_ki             << ',' << r.dut_rx_pos_kd              << ','
      << static_cast<int>(r.dut_rx_enable) << ',';
    // sensors (all zero — no external sensors in this testbench)
    o << r.encoder_count  << ',' << r.torque_ch1_nm   << ',' << r.torque_ch2_nm << ','
      << r.main_gear_ratio << ',' << r.dut_gear_ratio << ',';
    // jip algorithm parameters
    o << r.jip_inertia            << ',' << r.jip_hardstop_pos_upper << ','
      << r.jip_hardstop_pos_lower << ',' << r.jip_margin;
    return o.str();
}

// ── makeDriveJson ─────────────────────────────────────────────────────────────

std::string JointImpactPreventionTestbench::makeDriveJson(
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
        j["max_velocity_abs_rad_s"] = static_cast<double>(ds.max_velocity_abs)
                                      * (2.0 * M_PI / 1000.0);
        j["min_position_rad"]   = (ds.min_position == std::numeric_limits<int32_t>::min())
            ? -1000.0 : static_cast<double>(ds.min_position) * enc_to_rad;
        j["max_position_rad"]   = (ds.max_position == std::numeric_limits<int32_t>::max())
            ?  1000.0 : static_cast<double>(ds.max_position) * enc_to_rad;
        j["max_velocity_abs"]   = ds.max_velocity_abs;
        j["min_position"]       = ds.min_position;
        j["max_position"]       = ds.max_position;
        j["max_current_a"]      = static_cast<double>(gains.max_current_a);
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

void JointImpactPreventionTestbench::printDebug(
    const SystemStatus& status,
    const LoopStats&    stats,
    const CommandState& cmd) const
{
    auto it = status.by_slave.find(drive_slave_);
    if (it != status.by_slave.end() && it->second.has_value()) {
        const auto& ds = std::any_cast<const DriveStatus&>(it->second);
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
    std::printf(
        "[timing] rt_period=%.3f ms | rt_cycle=%.3f ms | wakeup_lat=%.3f ms\n",
        static_cast<double>(stats.last_period_ns)         * 1e-6,
        static_cast<double>(stats.last_cycle_time_ns)     * 1e-6,
        static_cast<double>(stats.last_wakeup_latency_ns) * 1e-6
    );
}
