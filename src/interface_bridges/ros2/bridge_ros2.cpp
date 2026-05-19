// Dyno ROS2 Bridge
//
// Single C++ process: runs the EtherCAT loop and publishes telemetry directly
// to ROS2 topics using rclcpp.  Subscribes to a command topic to receive
// speed/enable commands from other ROS2 nodes or scripts.
//
// Build with colcon (after building the parent CMake project first):
//   cd <repo_root>
//   cmake -S . -B build && cmake --build build
//   source /opt/ros/humble/setup.bash
//   colcon build --packages-select dyno_ros2_bridge
//                --base-paths src/interface_bridges/ros2
//
// Run:
//   source install/setup.bash
//   sudo ros2 run dyno_ros2_bridge bridge_ros2 [--ros-args -p topology:=...]
//
// Topics published (std_msgs/String, JSON):
//   /dyno/main_drive/status
//   /dyno/dut/status
//   /dyno/loop/stats
//
// Topics published (primitive):
//   /dyno/encoder/count     (std_msgs/UInt32)
//   /dyno/torque/ch1        (std_msgs/Float64)
//   /dyno/torque/ch2        (std_msgs/Float64)
//
// Topics subscribed:
//   /dyno/command           (std_msgs/String, JSON)
//     Fields: main_speed, dut_speed, main_enable, dut_enable,
//             fault_reset, hold_output1

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/loop.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/default_adapter_factory.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/adapter.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/data_types.hpp"
#include "ethercat_core/devices/beckhoff/el5032/data_types.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"
#include "ethercat_core/devices/motor_drives/drive_bases/ds402/data_types.hpp"

#include "pdo_log.hpp"
#include "dual_novanta_testbench/dual_novanta_testbench.hpp"

extern "C" {
#include "ethercat.h"
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <nlohmann/json.hpp>

#include <zlib.h>

#include <algorithm>
#include <fstream>
#include <cmath>
#include <pwd.h>
#include <limits>
#include <any>
#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <signal.h>
#include <cstring>
#include <memory>
#include <mutex>
#include <set>
#include <pthread.h>
#include <sched.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

using namespace ethercat_core;
using namespace ethercat_core::novanta::volcano;
using Cia402State     = ethercat_core::ds402::Cia402State;
using ModeOfOperation = ethercat_core::ds402::ModeOfOperation;
using json            = nlohmann::json;

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{false};
static std::atomic<bool> g_rotate_log{false};
static std::string       g_script_name;
static std::mutex        g_script_name_mtx;
static void onSignal(int) { g_shutdown.store(true); }

// ── Shared command state (written by ROS2 subscriber, read by main loop) ──────

// CommandState, DriveGains, DEFAULT_* constants, and cia402Name are defined in
// dual_novanta_testbench.hpp (included above).

static std::mutex      g_cmd_mutex;
static CommandState    g_cmd_state;
static bool            g_loop_running = false;

// ── SDO request / response (written by ROS2 subscriber, executed by main loop) ─

struct SdoRequest {
    bool     pending      = false;
    bool     is_write     = false;
    bool     is_pre_op    = false;  // pre_op_all / pre_op_off
    bool     is_store_all = false;  // atomic: pre-op → write 0x26DB → return to OP
    int      slave_idx    = 0;
    uint16_t index        = 0;
    uint8_t  subindex     = 0;
    int      size         = 4;   // bytes: 1, 2, or 4
    int64_t  value        = 0;   // for writes; for pre_op: 0=enter, 1=exit
};
struct SdoResponse {
    std::string op;
    uint16_t    index    = 0;
    uint8_t     subindex = 0;
    int         size     = 0;
    bool        success  = false;
    int64_t     value    = 0;
    std::string error;
};

static std::mutex   g_sdo_mutex;
static SdoRequest   g_sdo_req;

// ── ROS2 node ─────────────────────────────────────────────────────────────────

class DynoBridgeNode : public rclcpp::Node {
public:
    DynoBridgeNode() : Node("dyno_ros2_bridge") {
        // Publishers
        pub_main_  = create_publisher<std_msgs::msg::String>("/dyno/main_drive/status", 10);
        pub_dut_   = create_publisher<std_msgs::msg::String>("/dyno/dut/status",         10);
        pub_stats_ = create_publisher<std_msgs::msg::String>("/dyno/loop/stats",          10);
        pub_enc_   = create_publisher<std_msgs::msg::UInt32>("/dyno/encoder/count",       10);
        pub_ch1_t_ = create_publisher<std_msgs::msg::Float64>("/dyno/torque/ch1",         10);
        pub_ch2_t_ = create_publisher<std_msgs::msg::Float64>("/dyno/torque/ch2",         10);
        pub_sdo_   = create_publisher<std_msgs::msg::String>("/dyno/sdo_response",        10);
        pub_bus_   = create_publisher<std_msgs::msg::String>("/dyno/bus_status",          10);
        pub_rt_cmd_ = create_publisher<std_msgs::msg::String>("/dyno/rt_command",         10);

        // Command subscriber
        sub_cmd_ = create_subscription<std_msgs::msg::String>(
            "/dyno/command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    const json j = json::parse(msg->data);
                    std::lock_guard<std::mutex> lk(g_cmd_mutex);
                    g_cmd_state.main_speed    = j.value("main_velocity", j.value("main_speed", 0.0f));
                    g_cmd_state.dut_speed     = j.value("dut_velocity",  j.value("dut_speed",  0.0f));
                    g_cmd_state.main_position = j.value("main_position", 0.0f);
                    g_cmd_state.dut_position  = j.value("dut_position",  0.0f);
                    g_cmd_state.main_torque   = j.value("main_torque",   0.0f);
                    g_cmd_state.dut_torque    = j.value("dut_torque",    0.0f);
                    g_cmd_state.main_current  = j.value("main_iqcommand",
                                                        j.value("main_iq_command",
                                                                j.value("main_current", 0.0f)));
                    g_cmd_state.dut_current   = j.value("dut_iqcommand",
                                                        j.value("dut_iq_command",
                                                                j.value("dut_current", 0.0f)));
                    g_cmd_state.main_enable   = j.value("main_enable",   false);
                    g_cmd_state.dut_enable    = j.value("dut_enable",    false);
                    g_cmd_state.fault_reset   = j.value("fault_reset",   false);
                    g_cmd_state.hold_output1  = j.value("hold_output1",  false);
                    g_cmd_state.main_mode     = static_cast<int8_t>(j.value("main_mode", static_cast<int>(ModeOfOperation::CYCLIC_SYNC_VELOCITY)));
                    g_cmd_state.dut_mode      = static_cast<int8_t>(j.value("dut_mode",  static_cast<int>(ModeOfOperation::CYCLIC_SYNC_VELOCITY)));
                    // Gains: use current value as default so omitted fields persist
                    g_cmd_state.main_torque_kp      = j.value("main_torque_kp",      g_cmd_state.main_torque_kp);
                    g_cmd_state.main_torque_max_out = j.value("main_torque_max",      g_cmd_state.main_torque_max_out);
                    g_cmd_state.main_torque_min_out = j.value("main_torque_min",      g_cmd_state.main_torque_min_out);
                    g_cmd_state.main_vel_kp         = j.value("main_vel_kp",          g_cmd_state.main_vel_kp);
                    g_cmd_state.main_vel_ki         = j.value("main_vel_ki",          g_cmd_state.main_vel_ki);
                    g_cmd_state.main_vel_kd         = j.value("main_vel_kd",          g_cmd_state.main_vel_kd);
                    g_cmd_state.main_pos_kp         = j.value("main_pos_kp",          g_cmd_state.main_pos_kp);
                    g_cmd_state.main_pos_ki         = j.value("main_pos_ki",          g_cmd_state.main_pos_ki);
                    g_cmd_state.main_pos_kd         = j.value("main_pos_kd",          g_cmd_state.main_pos_kd);
                    g_cmd_state.dut_torque_kp       = j.value("dut_torque_kp",        g_cmd_state.dut_torque_kp);
                    g_cmd_state.dut_torque_max_out  = j.value("dut_torque_max",       g_cmd_state.dut_torque_max_out);
                    g_cmd_state.dut_torque_min_out  = j.value("dut_torque_min",       g_cmd_state.dut_torque_min_out);
                    g_cmd_state.dut_vel_kp          = j.value("dut_vel_kp",           g_cmd_state.dut_vel_kp);
                    g_cmd_state.dut_vel_ki          = j.value("dut_vel_ki",           g_cmd_state.dut_vel_ki);
                    g_cmd_state.dut_vel_kd          = j.value("dut_vel_kd",           g_cmd_state.dut_vel_kd);
                    g_cmd_state.dut_pos_kp          = j.value("dut_pos_kp",           g_cmd_state.dut_pos_kp);
                    g_cmd_state.dut_pos_ki          = j.value("dut_pos_ki",           g_cmd_state.dut_pos_ki);
                    g_cmd_state.dut_pos_kd          = j.value("dut_pos_kd",           g_cmd_state.dut_pos_kd);
                    g_cmd_state.ch1_torque_scale    = j.value("ch1_torque_scale",     g_cmd_state.ch1_torque_scale);
                    g_cmd_state.ch2_torque_scale    = j.value("ch2_torque_scale",     g_cmd_state.ch2_torque_scale);
                    // One-shot: OR with current so a true is never lost between snapshots.
                    g_cmd_state.zero_torque_ch1    |= j.value("zero_torque_ch1", false);
                    g_cmd_state.zero_torque_ch2    |= j.value("zero_torque_ch2", false);
                    g_cmd_state.save_log           |= j.value("save_log",        false);
                    if (j.contains("script_name")) {
                        std::lock_guard<std::mutex> snlk(g_script_name_mtx);
                        g_script_name = j.value("script_name", "");
                    }
                    // Function generator config
                    g_cmd_state.main_fg_enable       = j.value("main_fg_enable",       false);
                    g_cmd_state.main_fg_waveform     = j.value("main_fg_waveform",     g_cmd_state.main_fg_waveform);
                    g_cmd_state.main_fg_control_type = j.value("main_fg_control_type", g_cmd_state.main_fg_control_type);
                    g_cmd_state.main_fg_amplitude    = j.value("main_fg_amplitude",    g_cmd_state.main_fg_amplitude);
                    g_cmd_state.main_fg_frequency    = j.value("main_fg_frequency",    g_cmd_state.main_fg_frequency);
                    g_cmd_state.main_fg_offset       = j.value("main_fg_offset",       g_cmd_state.main_fg_offset);
                    g_cmd_state.main_fg_phase        = j.value("main_fg_phase",        g_cmd_state.main_fg_phase);
                    g_cmd_state.dut_fg_enable        = j.value("dut_fg_enable",        false);
                    g_cmd_state.dut_fg_waveform      = j.value("dut_fg_waveform",      g_cmd_state.dut_fg_waveform);
                    g_cmd_state.dut_fg_control_type  = j.value("dut_fg_control_type",  g_cmd_state.dut_fg_control_type);
                    g_cmd_state.dut_fg_amplitude     = j.value("dut_fg_amplitude",     g_cmd_state.dut_fg_amplitude);
                    g_cmd_state.dut_fg_frequency     = j.value("dut_fg_frequency",     g_cmd_state.dut_fg_frequency);
                    g_cmd_state.dut_fg_offset        = j.value("dut_fg_offset",        g_cmd_state.dut_fg_offset);
                    g_cmd_state.dut_fg_phase         = j.value("dut_fg_phase",         g_cmd_state.dut_fg_phase);
                    g_cmd_state.main_fg_chirp_f_low  = j.value("main_fg_chirp_f_low",  g_cmd_state.main_fg_chirp_f_low);
                    g_cmd_state.main_fg_chirp_f_high = j.value("main_fg_chirp_f_high", g_cmd_state.main_fg_chirp_f_high);
                    g_cmd_state.main_fg_chirp_dur    = j.value("main_fg_chirp_dur",    g_cmd_state.main_fg_chirp_dur);
                    g_cmd_state.dut_fg_chirp_f_low   = j.value("dut_fg_chirp_f_low",   g_cmd_state.dut_fg_chirp_f_low);
                    g_cmd_state.dut_fg_chirp_f_high  = j.value("dut_fg_chirp_f_high",  g_cmd_state.dut_fg_chirp_f_high);
                    g_cmd_state.dut_fg_chirp_dur     = j.value("dut_fg_chirp_dur",     g_cmd_state.dut_fg_chirp_dur);
                } catch (...) {
                    RCLCPP_WARN(get_logger(), "Failed to parse /dyno/command JSON");
                }
            }
        );

        // SDO request subscriber — stores request; main loop executes and publishes response.
        sub_sdo_ = create_subscription<std_msgs::msg::String>(
            "/dyno/sdo_request", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    const json j = json::parse(msg->data);
                    const std::string op_str = j.value("op", "read");
                    SdoRequest req;
                    req.pending      = true;
                    req.is_write     = (op_str == "write");
                    req.is_pre_op    = (op_str == "pre_op_all" || op_str == "pre_op_off");
                    req.is_store_all = (op_str == "store_all");
                    const std::string drv = j.value("drive", "main");
                    req.slave_idx = (drv == "dut") ? dut_soem_idx_ : drive_soem_idx_;
                    req.index     = static_cast<uint16_t>(
                                        std::stoul(j.value("index", "0"), nullptr, 16));
                    req.subindex  = static_cast<uint8_t>(
                                        std::stoul(j.value("subindex", "0"), nullptr, 16));
                    req.size      = j.value("size", 4);
                    req.value     = req.is_pre_op
                                    ? (op_str == "pre_op_off" ? int64_t{1} : int64_t{0})
                                    : j.value("value", int64_t{0});
                    std::lock_guard<std::mutex> lk(g_sdo_mutex);
                    g_sdo_req = req;
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "Bad /dyno/sdo_request: %s", e.what());
                }
            }
        );

        RCLCPP_INFO(get_logger(), "DynoBridgeNode ready.");
    }

    void setSlaveIndices(int drive_idx, int dut_idx) {
        drive_soem_idx_ = drive_idx;
        dut_soem_idx_   = dut_idx;
    }

    void publishSdoResponse(const SdoResponse& resp) {
        std::ostringstream idx_ss, val_ss;
        idx_ss << "0x" << std::hex << std::uppercase << std::setw(4)
               << std::setfill('0') << resp.index;
        val_ss << "0x" << std::hex << std::uppercase << resp.value;
        json jr;
        jr["op"]        = resp.op;
        jr["index"]     = idx_ss.str();
        jr["subindex"]  = resp.subindex;
        jr["size"]      = resp.size;
        jr["success"]   = resp.success;
        jr["value"]     = resp.value;
        jr["value_hex"] = val_ss.str();
        jr["error"]     = resp.error;
        std_msgs::msg::String out;
        out.data = jr.dump();
        pub_sdo_->publish(out);
    }

    void publishBusStatus() {
        json arr = json::array();
        for (int i = 1; i <= ec_slavecount; ++i) {
            json s;
            s["idx"]  = i;
            s["name"] = std::string(ec_slave[i].name);
            s["al"]   = alStateName(static_cast<int>(ec_slave[i].state));
            arr.push_back(s);
        }
        std_msgs::msg::String msg;
        msg.data = arr.dump();
        pub_bus_->publish(msg);
    }

    void publishTelemetry(
        uint64_t cycle, int wkc, double cycle_us,
        const std::string& main_json,
        const std::string& dut_json,
        uint32_t enc,
        double ch1_t, double ch2_t)
    {
        {
            std_msgs::msg::String msg;
            msg.data = main_json;
            pub_main_->publish(msg);
        }
        {
            std_msgs::msg::String msg;
            msg.data = dut_json;
            pub_dut_->publish(msg);
        }
        {
            std_msgs::msg::String msg;
            msg.data = json{
                {"cycle",   cycle},
                {"wkc",     wkc},
                {"cycle_us", cycle_us},
            }.dump();
            pub_stats_->publish(msg);
        }
        {
            std_msgs::msg::UInt32 msg;
            msg.data = enc;
            pub_enc_->publish(msg);
        }
        {
            std_msgs::msg::Float64 msg;
            msg.data = ch1_t;
            pub_ch1_t_->publish(msg);
        }
        {
            std_msgs::msg::Float64 msg;
            msg.data = ch2_t;
            pub_ch2_t_->publish(msg);
        }
    }

    // Publish the effective per-cycle command, substituting the FG output when
    // the function generator is enabled.
    void publishRtCommand(const CommandState& cmd,
                          float main_fg_out, float dut_fg_out)
    {
        using CT = testbench_utils::ControlType;

        auto fill = [](json& j, const std::string& pfx,
                       bool fg_en, int fg_ct, float fg_out,
                       float raw_vel, float raw_pos,
                       float raw_torque, float raw_current) {
            if (fg_en) {
                switch (static_cast<CT>(fg_ct)) {
                case CT::VELOCITY: j[pfx + "velocity"] = fg_out;  break;
                case CT::POSITION: j[pfx + "position"] = fg_out;  break;
                case CT::TORQUE:   j[pfx + "torque"]   = fg_out;  break;
                case CT::CURRENT:  j[pfx + "current"]  = fg_out;  break;
                default: break;
                }
            } else {
                j[pfx + "velocity"] = raw_vel;
                j[pfx + "position"] = raw_pos;
                j[pfx + "torque"]   = raw_torque;
                j[pfx + "current"]  = raw_current;
            }
        };

        json j;
        fill(j, "main_", cmd.main_fg_enable, cmd.main_fg_control_type, main_fg_out,
             cmd.main_speed, cmd.main_position, cmd.main_torque, cmd.main_current);
        fill(j, "dut_",  cmd.dut_fg_enable,  cmd.dut_fg_control_type,  dut_fg_out,
             cmd.dut_speed,  cmd.dut_position,  cmd.dut_torque,  cmd.dut_current);

        std_msgs::msg::String msg;
        msg.data = j.dump();
        pub_rt_cmd_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_main_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_dut_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_stats_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr   pub_enc_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  pub_ch1_t_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  pub_ch2_t_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_sdo_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_bus_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_rt_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sdo_;
    int drive_soem_idx_ = 1;
    int dut_soem_idx_   = 2;
};

// ── main ──────────────────────────────────────────────────────────────────────

/// Chown path to the original unprivileged user.
/// Prefers DYNO_ORIGINAL_USER (set by dyno_gui.py before the nested sudo -E
/// that launches this process) because sudo always overwrites SUDO_USER with
/// the user who ran *that* sudo invocation — which is root when the GUI
/// (already root) re-invokes sudo to start the bridge.
static void chown_to_sudo_user(const std::string& path)
{
    const char* sudo_user = std::getenv("DYNO_ORIGINAL_USER");
    if (!sudo_user || !*sudo_user) sudo_user = std::getenv("SUDO_USER");
    if (!sudo_user || !*sudo_user) return;
    const struct passwd* pw = getpwnam(sudo_user);
    if (!pw) return;
    chown(path.c_str(), pw->pw_uid, pw->pw_gid);
}

/// Create test_data_log/YYYY-MM-DD/HHMMSS[_script_name]/dyno_pdo.csv, making all parent dirs.
static std::string make_run_csv_path(rclcpp::Logger logger,
                                     const std::string& script_name = "")
{
    std::time_t t    = std::time(nullptr);
    std::tm*    tm_  = std::localtime(&t);
    char date_buf[16], time_buf[8];
    std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d", tm_);
    std::strftime(time_buf, sizeof(time_buf), "%H%M%S",   tm_);
    std::string suffix;
    if (!script_name.empty()) {
        suffix = "_";
        for (char c : script_name)
            suffix += (std::isalnum(static_cast<unsigned char>(c)) ? c : '_');
        while (!suffix.empty() && suffix.back() == '_') suffix.pop_back();
    }
    std::string run_dir = std::string("test_data_log/") + date_buf + "/" + time_buf + suffix;
    std::error_code ec;
    std::filesystem::create_directories(run_dir, ec);
    if (ec) {
        RCLCPP_WARN(logger, "Could not create log dir '%s': %s",
                    run_dir.c_str(), ec.message().c_str());
    }
    chown_to_sudo_user(std::string("test_data_log/") + date_buf);
    chown_to_sudo_user(run_dir);
    return run_dir + "/dyno_pdo.csv.gz";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // ROS2 node runs on its own thread via a SingleThreadedExecutor so it
    // doesn't block the main EtherCAT loop.
    auto node     = std::make_shared<DynoBridgeNode>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    std::thread ros_thread([&executor]() { executor->spin(); });

    // Read ROS2 parameters (set via --ros-args -p key:=value).
    auto get_str = [&](const char* name, const char* def) -> std::string {
        node->declare_parameter(name, def);
        return node->get_parameter(name).as_string();
    };
    auto get_dbl = [&](const char* name, double def) -> double {
        node->declare_parameter(name, def);
        return node->get_parameter(name).as_double();
    };
    auto get_int = [&](const char* name, int def) -> int {
        node->declare_parameter(name, def);
        return node->get_parameter(name).as_int();
    };

    const std::string topology      = get_str("topology",      DEFAULT_TOPOLOGY);
    const std::string drive_slave   = get_str("drive_slave",   DEFAULT_DRIVE_SLAVE);
    const std::string dut_slave     = get_str("dut_slave",     DEFAULT_DUT_SLAVE);
    const std::string encoder_slave = get_str("encoder_slave", DEFAULT_ENCODER_SLAVE);
    const std::string torque_slave  = get_str("torque_slave",  DEFAULT_TORQUE_SLAVE);
    const std::string io_slave      = get_str("io_slave",      DEFAULT_IO_SLAVE);
    const double      pub_hz        = get_dbl("pub_hz",        DEFAULT_PUB_HZ);
    const double      fault_reset_s = get_dbl("fault_reset_s", DEFAULT_FAULT_RESET);
    const int         rt_priority   = get_int("rt_priority",   95);
    const std::string cpu_affinity_str = get_str("cpu_affinity", "2");
    const bool        debug_print   = get_int("debug",         0) != 0;

    // EtherCAT init.
    MasterConfig cfg;
    try {
        cfg = loadTopology(topology);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Topology load failed: %s", e.what());
        executor->cancel();
        ros_thread.join();
        return 1;
    }

    EthercatMaster master(cfg, ethercat_core::makeDefaultAdapterFactory());
    MasterRuntime* rt = nullptr;
    try {
        rt = &master.initialize();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Master init failed: %s", e.what());
        executor->cancel();
        ros_thread.join();
        return 1;
    }

    // Required slaves — fatal if missing.
    for (const auto& name : {drive_slave, encoder_slave, torque_slave, io_slave}) {
        if (rt->adapters.find(name) == rt->adapters.end()) {
            RCLCPP_FATAL(node->get_logger(), "Required slave '%s' not found in topology.", name.c_str());
            master.close();
            executor->cancel();
            ros_thread.join();
            return 1;
        }
    }

    // DUT is optional.
    const bool dut_present = rt->adapters.find(dut_slave) != rt->adapters.end();
    if (dut_present) {
        RCLCPP_INFO(node->get_logger(), "DUT slave '%s' found.", dut_slave.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "DUT slave '%s' not found — running without DUT.", dut_slave.c_str());
    }

    auto* elm3002 = dynamic_cast<beckhoff::elm3002::Elm3002Adapter*>(
        rt->adapters.at(torque_slave).get());
    if (!elm3002) {
        RCLCPP_FATAL(node->get_logger(), "Slave '%s' is not an ELM3002.", torque_slave.c_str());
        master.close();
        executor->cancel();
        ros_thread.join();
        return 1;
    }

    const int drive_soem_idx = rt->slave_index.at(drive_slave);
    const int dut_soem_idx   = dut_present ? rt->slave_index.at(dut_slave) : -1;

    RCLCPP_INFO(node->get_logger(),
        "[init] SOEM indices — %s=%d  %s=%d",
        drive_slave.c_str(), drive_soem_idx,
        dut_slave.c_str(), dut_soem_idx);

    node->setSlaveIndices(drive_soem_idx, dut_soem_idx);

    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority = std::clamp(rt_priority, 0, 99);
    // Parse comma-separated CPU affinity list (e.g. "2" or "2,3").
    if (!cpu_affinity_str.empty()) {
        std::istringstream ss(cpu_affinity_str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try { rt_cfg.cpu_affinity.insert(std::stoi(token)); }
            catch (...) {
                RCLCPP_WARN(node->get_logger(),
                    "Ignoring invalid cpu_affinity token: '%s'", token.c_str());
            }
        }
    }

    // Use sigaction instead of std::signal so we reliably override the handler
    // that rclcpp::init() installs via sigaction for SIGINT.
    struct sigaction sa{};
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);

    // ── CSV logging setup ─────────────────────────────────────────────────────
    std::string log_path = make_run_csv_path(node->get_logger());
    gzFile csv_gz = gzopen(log_path.c_str(), "wb1");
    gzwrite(csv_gz, dyno::PDO_LOG_CSV_HEADER, static_cast<unsigned>(strlen(dyno::PDO_LOG_CSV_HEADER)));
    gzwrite(csv_gz, "\n", 1);
    chown_to_sudo_user(log_path);
    RCLCPP_INFO(node->get_logger(), "PDO log: %s", log_path.c_str());

    // Encoder resolution — read from topology config; defaults live in SlaveConfig::Scaling.
    const int main_out_enc_bits = [&] {
        for (const auto& sc : cfg.slaves)
            if (sc.name == drive_slave) return sc.scaling.output_encoder_res_bits;
        return 0;
    }();
    const int dut_out_enc_bits = [&] {
        for (const auto& sc : cfg.slaves)
            if (sc.name == dut_slave) return sc.scaling.output_encoder_res_bits;
        return 0;
    }();

    // Fault-reset window: drives are held disabled until this time point.
    const auto reset_end = std::chrono::steady_clock::now()
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(fault_reset_s));

    DualNovantaTestbench testbench(
        drive_slave, dut_slave, encoder_slave, torque_slave, io_slave,
        dut_present, elm3002,
        drive_soem_idx, dut_soem_idx,
        main_out_enc_bits, dut_out_enc_bits
    );
    testbench.extractAndSeedGains(*rt, g_cmd_state, g_cmd_mutex);
    {
        std::lock_guard<std::mutex> lk(g_cmd_mutex);
        RCLCPP_INFO(node->get_logger(),
            "[main_drive] vel_kp=%.4f vel_ki=%.4f torque_kp=%.4f",
            static_cast<double>(g_cmd_state.main_vel_kp),
            static_cast<double>(g_cmd_state.main_vel_ki),
            static_cast<double>(g_cmd_state.main_torque_kp));
        RCLCPP_INFO(node->get_logger(),
            "[dut]        vel_kp=%.4f vel_ki=%.4f torque_kp=%.4f",
            static_cast<double>(g_cmd_state.dut_vel_kp),
            static_cast<double>(g_cmd_state.dut_vel_ki),
            static_cast<double>(g_cmd_state.dut_torque_kp));
    }

    // Ring buffer: RT callback pushes one record per cycle; drain thread writes to CSV.
    dyno::PdoLogBuffer<200> log_buf;

    std::thread log_drain([&]() {
        while (!g_shutdown.load() || !log_buf.empty()) {
            while (auto rec = log_buf.pop()) {
                std::string row = DualNovantaTestbench::serializeToCsvRow(*rec) + '\n';
                gzwrite(csv_gz, row.c_str(), static_cast<unsigned>(row.size()));
            }
            if (g_rotate_log.exchange(false)) {
                gzflush(csv_gz, Z_SYNC_FLUSH);
                gzclose(csv_gz);
                std::string sname;
                { std::lock_guard<std::mutex> lk(g_script_name_mtx); sname = g_script_name; }
                log_path = make_run_csv_path(node->get_logger(), sname);
                csv_gz = gzopen(log_path.c_str(), "wb1");
                gzwrite(csv_gz, dyno::PDO_LOG_CSV_HEADER, static_cast<unsigned>(strlen(dyno::PDO_LOG_CSV_HEADER)));
                gzwrite(csv_gz, "\n", 1);
                chown_to_sudo_user(log_path);
                RCLCPP_INFO(node->get_logger(), "PDO log rotated: %s", log_path.c_str());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        gzclose(csv_gz);
    });

    // ── Register cycle callback — fires from RT thread at EtherCAT cycle rate ─
    loop.setCycleCallback(
        testbench.makeCallback(g_cmd_state, g_cmd_mutex, log_buf, reset_end));

    loop.start();
    g_loop_running = true;

    // Safe stop/start helpers — guard against double-stop or double-start.
    auto safe_stop  = [&]{ if ( g_loop_running) { loop.stop();  g_loop_running = false; } };
    auto safe_start = [&]{ if (!g_loop_running) { loop.start(); g_loop_running = true;  } };
    auto recover_bus_to_op = [&]() -> std::string {
        ec_slave[0].state = EC_STATE_SAFE_OP;
        ec_writestate(0);
        int safe_chk = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        if ((safe_chk & 0x0F) != EC_STATE_SAFE_OP) {
            ec_readstate();
            return "Not all slaves reached SAFE-OP";
        }

        // Prime process data before requesting OP — some slaves need a few
        // exchanges before they will transition out of SAFE-OP.
        for (int i = 0; i < 5; ++i) {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
        }

        ec_slave[0].state = EC_STATE_OPERATIONAL;
        ec_writestate(0);

        for (int attempt = 0; attempt < 50; ++attempt) {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_readstate();

            bool all_in_op = true;
            for (int i = 1; i <= ec_slavecount; ++i) {
                if ((ec_slave[i].state & 0x0F) != EC_STATE_OPERATIONAL) {
                    all_in_op = false;
                    break;
                }
            }
            if (all_in_op) {
                return "";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::ostringstream oss;
        oss << "Slaves still not OP:";
        for (int i = 1; i <= ec_slavecount; ++i) {
            if ((ec_slave[i].state & 0x0F) != EC_STATE_OPERATIONAL) {
                oss << " [" << i << "] " << ec_slave[i].name
                    << " state=0x" << std::hex << static_cast<int>(ec_slave[i].state);
            }
        }
        return oss.str();
    };


    // Pin the main thread (publish/command loop) to all CPUs except the ones
    // reserved for the EtherCAT RT loop, so they never compete on the same core.
    {
        const int ncpus = static_cast<int>(std::thread::hardware_concurrency());
        cpu_set_t main_cpuset;
        CPU_ZERO(&main_cpuset);
        for (int i = 0; i < ncpus; ++i) {
            if (rt_cfg.cpu_affinity.find(i) == rt_cfg.cpu_affinity.end())
                CPU_SET(i, &main_cpuset);
        }
        if (pthread_setaffinity_np(pthread_self(), sizeof(main_cpuset), &main_cpuset) != 0) {
            RCLCPP_WARN(node->get_logger(),
                "Failed to set main thread CPU affinity (errno=%d). "
                "Main loop may share CPU with EtherCAT thread.", errno);
        } else {
            RCLCPP_INFO(node->get_logger(),
                "Main thread pinned to all CPUs except RT core(s).");
        }
    }

    RCLCPP_INFO(node->get_logger(),
        "bridge_ros2 running | pub_hz=%.1f fault_reset=%.1fs", pub_hz, fault_reset_s);

    const auto   t0         = std::chrono::steady_clock::now();
    const double pub_period = 1.0 / std::max(pub_hz, 1.0);
    auto         next_pub   = t0;


    RCLCPP_INFO(node->get_logger(), "Entering main loop...");
    int  debug_iter    = 0;
    auto main_loop_prev = std::chrono::steady_clock::now();

    while (!g_shutdown.load()) {
        const auto   now        = std::chrono::steady_clock::now();
        main_loop_prev = now;
        const bool in_reset = now < reset_end;

        if (++debug_iter <= 3)
            RCLCPP_INFO(node->get_logger(), "Loop iter %d  in_reset=%d  now>=next_pub=%d",
                        debug_iter, (int)in_reset, (int)(now >= next_pub));

        // Snapshot command state.
        CommandState cmd;
        {
            std::lock_guard<std::mutex> lk(g_cmd_mutex);
            cmd = g_cmd_state;
        }

        // Handle one-shot save_log: signal the drain thread to rotate the CSV file.
        if (cmd.save_log) {
            g_rotate_log.store(true);
            std::lock_guard<std::mutex> lk(g_cmd_mutex);
            g_cmd_state.save_log = false;
        }

        // SDO / EtherCAT state operations — all serialised with PDO via safe_stop/safe_start.
        {
            SdoRequest req;
            {
                std::lock_guard<std::mutex> lk(g_sdo_mutex);
                if (g_sdo_req.pending) {
                    req = g_sdo_req;
                    g_sdo_req.pending = false;
                }
            }
            if (req.pending) {
                static constexpr int SDO_TIMEOUT_US = EC_TIMEOUTSAFE; // 20 ms

                if (req.is_pre_op) {
                    // ── Pre-OP toggle ────────────────────────────────────────
                    SdoResponse resp;
                    resp.op = (req.value == 0) ? "pre_op_all" : "pre_op_off";
                    if (req.value == 0) {
                        safe_stop();
                        ec_slave[0].state = EC_STATE_PRE_OP;
                        ec_writestate(0);
                        int chk = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
                        ec_readstate();  // refresh ec_slave[1..N].state for publishBusStatus
                        resp.success = ((chk & 0x0F) == EC_STATE_PRE_OP);
                        resp.error   = resp.success ? "" : "Not all slaves reached PRE-OP";
                        // loop stays stopped — PDO invalid in PRE-OP
                    } else {
                        const std::string op_err = recover_bus_to_op();
                        resp.success = op_err.empty();
                        resp.error   = op_err;
                        safe_start();
                    }
                    node->publishSdoResponse(resp);

                } else if (req.is_store_all) {
                    // ── Store All: pre-op → write 0x26DB → return to OP ─────
                    SdoResponse resp;
                    resp.op    = "store_all";
                    resp.index = 0x26DB;
                    safe_stop();
                    ec_slave[0].state = EC_STATE_PRE_OP;
                    ec_writestate(0);
                    int pre_chk = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
                    ec_readstate();  // refresh ec_slave[1..N].state
                    if ((pre_chk & 0x0F) != EC_STATE_PRE_OP) {
                        resp.success = false;
                        resp.error   = "Failed to reach PRE-OP";
                    } else if (req.slave_idx < 1) {
                        resp.success = false;
                        resp.error   = "Slave not present (index="
                                     + std::to_string(req.slave_idx) + ")";
                    } else {
                        uint32_t magic = 0x65766173u; // "evas" — DS301 save password
                        uint8_t  buf[4];
                        std::memcpy(buf, &magic, 4);
                        int rc = ec_SDOwrite(static_cast<uint16_t>(req.slave_idx),
                                             0x26DB, 0x00, FALSE, 4, buf, SDO_TIMEOUT_US);
                        resp.success = (rc > 0);
                        resp.value   = magic;
                        if (!resp.success)
                            resp.error = "ec_SDOwrite 0x26DB failed (rc="
                                         + std::to_string(rc) + ")";
                    }
                    // Return to OP regardless of SDO result, but report if recovery fails.
                    const std::string op_err = recover_bus_to_op();
                    if (!op_err.empty()) {
                        if (resp.error.empty()) {
                            resp.error = op_err;
                        } else {
                            resp.error += " | " + op_err;
                        }
                        resp.success = false;
                    }
                    safe_start();
                    node->publishSdoResponse(resp);

                } else {
                    // ── Regular SDO read / write ─────────────────────────────
                    SdoResponse resp;
                    resp.op       = req.is_write ? "write" : "read";
                    resp.index    = req.index;
                    resp.subindex = req.subindex;
                    resp.size     = req.size;
                    if (req.slave_idx < 1) {
                        resp.success = false;
                        resp.error   = "Slave not present (index="
                                       + std::to_string(req.slave_idx) + ")";
                    } else if ((ec_slave[req.slave_idx].state & 0x0F) < EC_STATE_PRE_OP) {
                        resp.success = false;
                        resp.error   = "Slave not mailbox-ready (state=0x"
                                       + std::to_string(ec_slave[req.slave_idx].state & 0x0F) + ")";
                    } else {
                        safe_stop();
                        uint8_t buf[8] = {};
                        if (req.is_write) {
                            std::memcpy(buf, &req.value, static_cast<size_t>(req.size));
                            int rc = ec_SDOwrite(static_cast<uint16_t>(req.slave_idx),
                                                 req.index, req.subindex,
                                                 FALSE, req.size, buf, SDO_TIMEOUT_US);
                            resp.success = (rc > 0);
                            resp.value   = req.value;
                            if (!resp.success)
                                resp.error = "ec_SDOwrite failed (rc=" + std::to_string(rc) + ")";
                        } else {
                            int sz = req.size;
                            int rc = ec_SDOread(static_cast<uint16_t>(req.slave_idx),
                                                req.index, req.subindex,
                                                FALSE, &sz, buf, SDO_TIMEOUT_US);
                            resp.success = (rc > 0);
                            resp.size    = sz;
                            if (resp.success) {
                                int64_t v = 0;
                                std::memcpy(&v, buf, static_cast<size_t>(sz));
                                resp.value = v;
                            } else {
                                resp.error = "ec_SDOread failed (rc=" + std::to_string(rc) + ")";
                            }
                        }
                        safe_start();
                    }
                    node->publishSdoResponse(resp);
                }
            }
        }

        // Apply torque sensor ADC scale to the adapter.
        // Note: setCh1/2TorqueScale() writes ch1_torque_scale_ which is also read
        // by the RT cycle callback's scaledTorqueCh1/2() — on x86_64, aligned float
        // read/write is naturally atomic, so the worst case is one stale cycle during
        // a user-initiated scale change. Acceptable for an infrequent measurement setting.
        try {
            elm3002->setCh1TorqueScale(cmd.ch1_torque_scale);
            elm3002->setCh2TorqueScale(cmd.ch2_torque_scale);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                "Ignoring invalid torque scale value: %s", e.what());
        }

        // Command building and per-cycle logging are done in the RT cycle callback.

        // Sample current EtherCAT status and stats for telemetry publishing.
        const SystemStatus cur_status = loop.getStatus();
        const LoopStats    cur_stats  = loop.stats();

        // Publish telemetry at pub_hz rate.
        if (now >= next_pub) { try {
            DriveGains cmd_main_gains;
            cmd_main_gains.torque_kp              = cmd.main_torque_kp;
            cmd_main_gains.torque_loop_max_output = cmd.main_torque_max_out;
            cmd_main_gains.torque_loop_min_output = cmd.main_torque_min_out;
            cmd_main_gains.velocity_loop_kp       = cmd.main_vel_kp;
            cmd_main_gains.velocity_loop_ki       = cmd.main_vel_ki;
            cmd_main_gains.velocity_loop_kd       = cmd.main_vel_kd;
            cmd_main_gains.position_loop_kp       = cmd.main_pos_kp;
            cmd_main_gains.position_loop_ki       = cmd.main_pos_ki;
            cmd_main_gains.position_loop_kd       = cmd.main_pos_kd;
            cmd_main_gains.max_current_a          = cmd.main_max_current_a;

            DriveGains cmd_dut_gains;
            cmd_dut_gains.torque_kp               = cmd.dut_torque_kp;
            cmd_dut_gains.torque_loop_max_output  = cmd.dut_torque_max_out;
            cmd_dut_gains.torque_loop_min_output  = cmd.dut_torque_min_out;
            cmd_dut_gains.velocity_loop_kp        = cmd.dut_vel_kp;
            cmd_dut_gains.velocity_loop_ki        = cmd.dut_vel_ki;
            cmd_dut_gains.velocity_loop_kd        = cmd.dut_vel_kd;
            cmd_dut_gains.position_loop_kp        = cmd.dut_pos_kp;
            cmd_dut_gains.position_loop_ki        = cmd.dut_pos_ki;
            cmd_dut_gains.position_loop_kd        = cmd.dut_pos_kd;
            cmd_dut_gains.max_current_a           = cmd.dut_max_current_a;

            const std::string main_json = DualNovantaTestbench::makeDriveJson(
                drive_slave, drive_soem_idx, cur_status, main_out_enc_bits, cmd_main_gains);
            const std::string dut_json = DualNovantaTestbench::makeDriveJson(
                dut_slave, dut_soem_idx, cur_status, dut_out_enc_bits, cmd_dut_gains);

            uint32_t enc = 0;
            auto enc_it = cur_status.by_slave.find(encoder_slave);
            if (enc_it != cur_status.by_slave.end() && enc_it->second.has_value())
                enc = std::any_cast<const beckhoff::el5032::Data&>(
                    enc_it->second).encoder_count_25bit;

            double ch1_t = 0.0, ch2_t = 0.0;
            auto torque_it = cur_status.by_slave.find(torque_slave);
            if (torque_it != cur_status.by_slave.end() && torque_it->second.has_value()) {
                const auto& d = std::any_cast<const beckhoff::elm3002::Data&>(torque_it->second);
                // Apply one-shot zero before reading, then clear flags in shared state.
                if (cmd.zero_torque_ch1) {
                    elm3002->zeroTorqueCh1(d);
                    std::lock_guard<std::mutex> lk(g_cmd_mutex);
                    g_cmd_state.zero_torque_ch1 = false;
                }
                if (cmd.zero_torque_ch2) {
                    elm3002->zeroTorqueCh2(d);
                    std::lock_guard<std::mutex> lk(g_cmd_mutex);
                    g_cmd_state.zero_torque_ch2 = false;
                }
                ch1_t = static_cast<double>(elm3002->scaledTorqueCh1(d));
                ch2_t = static_cast<double>(elm3002->scaledTorqueCh2(d));
            }

            node->publishTelemetry(
                cur_stats.cycle_count,
                cur_stats.last_wkc,
                static_cast<double>(cur_stats.last_cycle_time_ns) / 1000.0,
                main_json, dut_json,
                enc, ch1_t, ch2_t
            );
            node->publishRtCommand(cmd, testbench.lastMainFgOut(), testbench.lastDutFgOut());
            node->publishBusStatus();

            if (debug_print)
                testbench.printDebug(cur_status, cur_stats, cmd, enc, ch1_t, ch2_t);

            next_pub += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(pub_period));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Publish exception: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(node->get_logger(), "Publish unknown exception");
        } }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Shutdown: do not send any DS402 disable commands before stopping.
    //
    // Any path through READY_TO_SWITCH_ON causes the Capitan drive to reset its
    // velocity gain registers (0x250A/B) to 0.  The only safe approach is to
    // stop the EtherCAT loop directly — the drive's PDO watchdog fires and
    // the AL layer moves to SAFE-OP+ERR while the DS402 state is preserved.
    // On the next initialize(), the fault-reset phase clears any residual state.
    // This matches drive_simple_speed_test1 behaviour, which does not cause faults.
    {
        const SystemStatus s  = loop.getStatus();
        auto log_state = [&](const std::string& name) {
            auto it = s.by_slave.find(name);
            if (it == s.by_slave.end() || !it->second.has_value()) return;
            const auto& ds = std::any_cast<const DriveStatus&>(it->second);
            RCLCPP_INFO(node->get_logger(), "[shutdown] %s was in %s — stopping loop",
                name.c_str(), cia402Name(ds.cia402_state));
        };
        log_state(drive_slave);
        if (dut_present) log_state(dut_slave);
    }

    safe_stop();
    master.close();

    // Drain remaining log records and close the CSV file.
    log_drain.join();
    RCLCPP_INFO(node->get_logger(), "PDO log saved: %s", log_path.c_str());

    executor->cancel();
    ros_thread.join();
    rclcpp::shutdown();
    return 0;
}
