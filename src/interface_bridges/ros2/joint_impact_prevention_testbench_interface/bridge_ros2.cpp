// Joint Impact Prevention Testbench — ROS2 Bridge
//
// Single C++ process: runs the EtherCAT loop for a single Novanta/Volcano
// main_drive and publishes telemetry directly to ROS2 topics using rclcpp.
// Subscribes to a command topic to receive drive commands from other nodes.
//
// Build with colcon (after building the parent CMake project first):
//   cmake -S . -B build && cmake --build build
//   source /opt/ros/humble/setup.bash
//   colcon build --packages-select jipt_ros2_bridge \
//                --base-paths src/interface_bridges/ros2
//
// Run:
//   source install/setup.bash
//   sudo ros2 run jipt_ros2_bridge bridge_ros2 [--ros-args -p topology:=...]
//
// Topics published (std_msgs/String, JSON):
//   /jipt/main_drive/status
//   /jipt/loop/stats
//   /jipt/sdo_response
//   /jipt/bus_status
//
// Topics subscribed:
//   /jipt/command   (std_msgs/String, JSON)
//     Fields: main_speed, main_position, main_torque, main_current,
//             main_enable, fault_reset, main_mode,
//             main_torque_kp, main_torque_max, main_torque_min,
//             main_vel_kp, main_vel_ki, main_vel_kd,
//             main_pos_kp, main_pos_ki, main_pos_kd
//   /jipt/sdo_request (std_msgs/String, JSON)

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/loop.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/default_adapter_factory.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"
#include "ethercat_core/devices/motor_drives/drive_bases/ds402/data_types.hpp"

#include "joint_impact_prevention_testbench/joint_impact_prevention_testbench.hpp"

extern "C" {
#include "ethercat.h"
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <nlohmann/json.hpp>

#include <zlib.h>

#include <algorithm>
#include <any>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <pwd.h>
#include <sched.h>
#include <set>
#include <signal.h>
#include <sstream>
#include <string>
#include <thread>

using namespace ethercat_core;
using namespace ethercat_core::novanta::volcano;
using ModeOfOperation = ethercat_core::ds402::ModeOfOperation;
using json            = nlohmann::json;

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{false};
static std::atomic<bool> g_rotate_log{false};
static std::string       g_script_name;
static std::mutex        g_script_name_mtx;
static void onSignal(int) { g_shutdown.store(true); }

// ── Shared command state (written by ROS2 subscriber, read by RT callback) ────

static std::mutex   g_cmd_mutex;
static CommandState g_cmd_state;
static bool         g_loop_running = false;

// ── SDO request / response ────────────────────────────────────────────────────

struct SdoRequest {
    bool     pending      = false;
    bool     is_write     = false;
    bool     is_pre_op    = false;
    bool     is_store_all = false;
    int      slave_idx    = 0;
    uint16_t index        = 0;
    uint8_t  subindex     = 0;
    int      size         = 4;
    int64_t  value        = 0;
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

static std::mutex  g_sdo_mutex;
static SdoRequest  g_sdo_req;

// ── ROS2 node ─────────────────────────────────────────────────────────────────

class JiptBridgeNode : public rclcpp::Node {
public:
    JiptBridgeNode() : Node("jipt_ros2_bridge") {
        pub_main_  = create_publisher<std_msgs::msg::String>("/jipt/main_drive/status", 10);
        pub_stats_ = create_publisher<std_msgs::msg::String>("/jipt/loop/stats",         10);
        pub_sdo_   = create_publisher<std_msgs::msg::String>("/jipt/sdo_response",       10);
        pub_bus_   = create_publisher<std_msgs::msg::String>("/jipt/bus_status",          10);

        sub_cmd_ = create_subscription<std_msgs::msg::String>(
            "/jipt/command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    const json j = json::parse(msg->data);
                    std::lock_guard<std::mutex> lk(g_cmd_mutex);
                    g_cmd_state.main_speed    = j.value("main_velocity", j.value("main_speed", 0.0f));
                    g_cmd_state.main_position = j.value("main_position", 0.0f);
                    g_cmd_state.main_torque   = j.value("main_torque",   0.0f);
                    g_cmd_state.main_current  = j.value("main_iq_command",
                                                        j.value("main_current", 0.0f));
                    g_cmd_state.main_enable   = j.value("main_enable",   false);
                    g_cmd_state.fault_reset   = j.value("fault_reset",   false);
                    g_cmd_state.main_mode     = static_cast<int8_t>(j.value(
                        "main_mode", static_cast<int>(ModeOfOperation::CYCLIC_SYNC_VELOCITY)));
                    // Gains: omitted fields keep current value
                    g_cmd_state.main_torque_kp      = j.value("main_torque_kp",  g_cmd_state.main_torque_kp);
                    g_cmd_state.main_torque_max_out = j.value("main_torque_max", g_cmd_state.main_torque_max_out);
                    g_cmd_state.main_torque_min_out = j.value("main_torque_min", g_cmd_state.main_torque_min_out);
                    g_cmd_state.main_vel_kp         = j.value("main_vel_kp",     g_cmd_state.main_vel_kp);
                    g_cmd_state.main_vel_ki         = j.value("main_vel_ki",     g_cmd_state.main_vel_ki);
                    g_cmd_state.main_vel_kd         = j.value("main_vel_kd",     g_cmd_state.main_vel_kd);
                    g_cmd_state.main_pos_kp         = j.value("main_pos_kp",     g_cmd_state.main_pos_kp);
                    g_cmd_state.main_pos_ki         = j.value("main_pos_ki",     g_cmd_state.main_pos_ki);
                    g_cmd_state.main_pos_kd         = j.value("main_pos_kd",     g_cmd_state.main_pos_kd);
                    g_cmd_state.save_log           |= j.value("save_log", false);
                    g_cmd_state.inertia             = j.value("inertia",             g_cmd_state.inertia);
                    g_cmd_state.hardstop_pos_upper  = j.value("hardstop_pos_upper",  g_cmd_state.hardstop_pos_upper);
                    g_cmd_state.hardstop_pos_lower  = j.value("hardstop_pos_lower",  g_cmd_state.hardstop_pos_lower);
                    g_cmd_state.margin              = j.value("margin",              g_cmd_state.margin);
                    if (j.contains("script_name")) {
                        std::lock_guard<std::mutex> snlk(g_script_name_mtx);
                        g_script_name = j.value("script_name", "");
                    }
                } catch (...) {
                    RCLCPP_WARN(get_logger(), "Failed to parse /jipt/command JSON");
                }
            }
        );

        sub_sdo_ = create_subscription<std_msgs::msg::String>(
            "/jipt/sdo_request", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    const json j = json::parse(msg->data);
                    const std::string op_str = j.value("op", "read");
                    SdoRequest req;
                    req.pending      = true;
                    req.is_write     = (op_str == "write");
                    req.is_pre_op    = (op_str == "pre_op_all" || op_str == "pre_op_off");
                    req.is_store_all = (op_str == "store_all");
                    req.slave_idx    = drive_soem_idx_;
                    req.index        = static_cast<uint16_t>(
                                           std::stoul(j.value("index", "0"), nullptr, 16));
                    req.subindex     = static_cast<uint8_t>(
                                           std::stoul(j.value("subindex", "0"), nullptr, 16));
                    req.size         = j.value("size", 4);
                    req.value        = req.is_pre_op
                                       ? (op_str == "pre_op_off" ? int64_t{1} : int64_t{0})
                                       : j.value("value", int64_t{0});
                    std::lock_guard<std::mutex> lk(g_sdo_mutex);
                    g_sdo_req = req;
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "Bad /jipt/sdo_request: %s", e.what());
                }
            }
        );

        RCLCPP_INFO(get_logger(), "JiptBridgeNode ready.");
    }

    void setDriveIndex(int drive_idx) { drive_soem_idx_ = drive_idx; }

    void publishSdoResponse(const SdoResponse& resp) {
        std::ostringstream idx_ss, val_ss;
        idx_ss << "0x" << std::hex << std::uppercase
               << std::setw(4) << std::setfill('0') << resp.index;
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
        const std::string& main_json)
    {
        {
            std_msgs::msg::String msg;
            msg.data = main_json;
            pub_main_->publish(msg);
        }
        {
            std_msgs::msg::String msg;
            msg.data = json{
                {"cycle",    cycle},
                {"wkc",      wkc},
                {"cycle_us", cycle_us},
            }.dump();
            pub_stats_->publish(msg);
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_main_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_stats_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_sdo_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_bus_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sdo_;
    int drive_soem_idx_ = 1;
};

// ── Logging helpers ───────────────────────────────────────────────────────────

static void chown_to_sudo_user(const std::string& path)
{
    const char* sudo_user = std::getenv("DYNO_ORIGINAL_USER");
    if (!sudo_user || !*sudo_user) sudo_user = std::getenv("SUDO_USER");
    if (!sudo_user || !*sudo_user) return;
    const struct passwd* pw = getpwnam(sudo_user);
    if (!pw) return;
    chown(path.c_str(), pw->pw_uid, pw->pw_gid);
}

static std::string make_run_csv_path(rclcpp::Logger logger,
                                     const std::string& script_name = "")
{
    std::time_t t   = std::time(nullptr);
    std::tm*    tm_ = std::localtime(&t);
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
    if (ec)
        RCLCPP_WARN(logger, "Could not create log dir '%s': %s",
                    run_dir.c_str(), ec.message().c_str());
    chown_to_sudo_user(std::string("test_data_log/") + date_buf);
    chown_to_sudo_user(run_dir);
    return run_dir + "/jipt_pdo.csv.gz";
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node     = std::make_shared<JiptBridgeNode>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    std::thread ros_thread([&executor]() { executor->spin(); });

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

    const std::string topology         = get_str("topology",      DEFAULT_TOPOLOGY);
    const std::string drive_slave      = get_str("drive_slave",   DEFAULT_DRIVE_SLAVE);
    const double      pub_hz           = get_dbl("pub_hz",        DEFAULT_PUB_HZ);
    const double      fault_reset_s    = get_dbl("fault_reset_s", DEFAULT_FAULT_RESET);
    const int         rt_priority      = get_int("rt_priority",   95);
    const std::string cpu_affinity_str = get_str("cpu_affinity",  "2");
    const bool        debug_print      = get_int("debug",         0) != 0;

    // EtherCAT init
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

    if (rt->adapters.find(drive_slave) == rt->adapters.end()) {
        RCLCPP_FATAL(node->get_logger(),
            "Required slave '%s' not found in topology.", drive_slave.c_str());
        master.close();
        executor->cancel();
        ros_thread.join();
        return 1;
    }

    const int drive_soem_idx = rt->slave_index.at(drive_slave);
    RCLCPP_INFO(node->get_logger(),
        "[init] SOEM index — %s=%d", drive_slave.c_str(), drive_soem_idx);
    node->setDriveIndex(drive_soem_idx);

    const int main_out_enc_bits = [&] {
        for (const auto& sc : cfg.slaves)
            if (sc.name == drive_slave) return sc.scaling.output_encoder_res_bits;
        return 20;
    }();

    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority = std::clamp(rt_priority, 0, 99);
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

    struct sigaction sa{};
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    const auto reset_end = std::chrono::steady_clock::now()
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(fault_reset_s));

    JointImpactPreventionTestbench testbench(drive_slave, drive_soem_idx, main_out_enc_bits);
    testbench.extractAndSeedGains(*rt, g_cmd_state, g_cmd_mutex);
    {
        std::lock_guard<std::mutex> lk(g_cmd_mutex);
        RCLCPP_INFO(node->get_logger(),
            "[%s] vel_kp=%.4f vel_ki=%.4f torque_kp=%.4f",
            drive_slave.c_str(),
            static_cast<double>(g_cmd_state.main_vel_kp),
            static_cast<double>(g_cmd_state.main_vel_ki),
            static_cast<double>(g_cmd_state.main_torque_kp));
    }

    dyno::PdoLogBuffer<200> log_buf;

    // Open initial CSV log file.
    std::string log_path = make_run_csv_path(node->get_logger());
    gzFile csv_gz = gzopen(log_path.c_str(), "wb1");
    gzwrite(csv_gz, dyno::PDO_LOG_CSV_HEADER,
            static_cast<unsigned>(strlen(dyno::PDO_LOG_CSV_HEADER)));
    gzwrite(csv_gz, "\n", 1);
    chown_to_sudo_user(log_path);
    RCLCPP_INFO(node->get_logger(), "PDO log: %s", log_path.c_str());

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);
    loop.setCycleCallback(
        testbench.makeCallback(g_cmd_state, g_cmd_mutex, log_buf, reset_end));
    loop.start();
    g_loop_running = true;

    auto safe_stop  = [&]{ if ( g_loop_running) { loop.stop();  g_loop_running = false; } };
    auto safe_start = [&]{ if (!g_loop_running) { loop.start(); g_loop_running = true;  } };

    // Drain thread: pops records from the ring buffer and writes to gzipped CSV.
    std::thread log_drain([&]() {
        while (!g_shutdown.load() || !log_buf.empty()) {
            while (auto rec = log_buf.pop()) {
                std::string row = JointImpactPreventionTestbench::serializeToCsvRow(*rec) + '\n';
                gzwrite(csv_gz, row.c_str(), static_cast<unsigned>(row.size()));
            }
            if (g_rotate_log.exchange(false)) {
                gzflush(csv_gz, Z_SYNC_FLUSH);
                gzclose(csv_gz);
                std::string sname;
                { std::lock_guard<std::mutex> lk(g_script_name_mtx); sname = g_script_name; }
                log_path = make_run_csv_path(node->get_logger(), sname);
                csv_gz = gzopen(log_path.c_str(), "wb1");
                gzwrite(csv_gz, dyno::PDO_LOG_CSV_HEADER,
                        static_cast<unsigned>(strlen(dyno::PDO_LOG_CSV_HEADER)));
                gzwrite(csv_gz, "\n", 1);
                chown_to_sudo_user(log_path);
                RCLCPP_INFO(node->get_logger(), "PDO log rotated: %s", log_path.c_str());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        gzclose(csv_gz);
    });

    auto recover_bus_to_op = [&]() -> std::string {
        ec_slave[0].state = EC_STATE_SAFE_OP;
        ec_writestate(0);
        int safe_chk = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        if ((safe_chk & 0x0F) != EC_STATE_SAFE_OP) {
            ec_readstate();
            return "Not all slaves reached SAFE-OP";
        }
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
                    all_in_op = false; break;
                }
            }
            if (all_in_op) return "";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::ostringstream oss;
        oss << "Slaves still not OP:";
        for (int i = 1; i <= ec_slavecount; ++i) {
            if ((ec_slave[i].state & 0x0F) != EC_STATE_OPERATIONAL)
                oss << " [" << i << "] " << ec_slave[i].name
                    << " state=0x" << std::hex << static_cast<int>(ec_slave[i].state);
        }
        return oss.str();
    };

    // Pin main thread to all CPUs except the RT core(s).
    {
        const int ncpus = static_cast<int>(std::thread::hardware_concurrency());
        cpu_set_t main_cpuset;
        CPU_ZERO(&main_cpuset);
        for (int i = 0; i < ncpus; ++i) {
            if (rt_cfg.cpu_affinity.find(i) == rt_cfg.cpu_affinity.end())
                CPU_SET(i, &main_cpuset);
        }
        if (pthread_setaffinity_np(pthread_self(), sizeof(main_cpuset), &main_cpuset) != 0)
            RCLCPP_WARN(node->get_logger(),
                "Failed to set main thread CPU affinity (errno=%d).", errno);
        else
            RCLCPP_INFO(node->get_logger(),
                "Main thread pinned to all CPUs except RT core(s).");
    }

    RCLCPP_INFO(node->get_logger(),
        "bridge_ros2 running | pub_hz=%.1f fault_reset=%.1fs", pub_hz, fault_reset_s);

    const double pub_period = 1.0 / std::max(pub_hz, 1.0);
    auto         next_pub   = std::chrono::steady_clock::now();

    while (!g_shutdown.load()) {
        const auto now = std::chrono::steady_clock::now();

        CommandState cmd;
        { std::lock_guard<std::mutex> lk(g_cmd_mutex); cmd = g_cmd_state; }

        // Handle one-shot save_log: signal the drain thread to rotate the CSV file.
        if (cmd.save_log) {
            g_rotate_log.store(true);
            std::lock_guard<std::mutex> lk(g_cmd_mutex);
            g_cmd_state.save_log = false;
        }

        // SDO / EtherCAT state operations
        {
            SdoRequest req;
            {
                std::lock_guard<std::mutex> lk(g_sdo_mutex);
                if (g_sdo_req.pending) { req = g_sdo_req; g_sdo_req.pending = false; }
            }
            if (req.pending) {
                static constexpr int SDO_TIMEOUT_US = EC_TIMEOUTSAFE;

                if (req.is_pre_op) {
                    SdoResponse resp;
                    resp.op = (req.value == 0) ? "pre_op_all" : "pre_op_off";
                    if (req.value == 0) {
                        safe_stop();
                        ec_slave[0].state = EC_STATE_PRE_OP;
                        ec_writestate(0);
                        int chk = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
                        ec_readstate();
                        resp.success = ((chk & 0x0F) == EC_STATE_PRE_OP);
                        resp.error   = resp.success ? "" : "Not all slaves reached PRE-OP";
                    } else {
                        const std::string op_err = recover_bus_to_op();
                        resp.success = op_err.empty();
                        resp.error   = op_err;
                        safe_start();
                    }
                    node->publishSdoResponse(resp);

                } else if (req.is_store_all) {
                    SdoResponse resp;
                    resp.op       = "store_all";
                    resp.index    = 0x1010;
                    resp.subindex = 0x01;
                    safe_stop();
                    ec_slave[0].state = EC_STATE_PRE_OP;
                    ec_writestate(0);
                    int pre_chk = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
                    ec_readstate();
                    if ((pre_chk & 0x0F) != EC_STATE_PRE_OP) {
                        resp.success = false;
                        resp.error   = "Failed to reach PRE-OP";
                    } else if (req.slave_idx < 1) {
                        resp.success = false;
                        resp.error   = "Slave not present (index=" + std::to_string(req.slave_idx) + ")";
                    } else {
                        uint32_t magic = 0x65766173u;
                        uint8_t  buf[4];
                        std::memcpy(buf, &magic, 4);
                        int rc = ec_SDOwrite(static_cast<uint16_t>(req.slave_idx),
                                             0x1010, 0x01, FALSE, 4, buf, SDO_TIMEOUT_US);
                        resp.success = (rc > 0);
                        resp.value   = magic;
                        if (!resp.success)
                            resp.error = "ec_SDOwrite 0x1010:01 failed (rc=" + std::to_string(rc) + ")";
                    }
                    const std::string op_err = recover_bus_to_op();
                    if (!op_err.empty()) {
                        resp.error   = resp.error.empty() ? op_err : resp.error + " | " + op_err;
                        resp.success = false;
                    }
                    safe_start();
                    node->publishSdoResponse(resp);

                } else {
                    SdoResponse resp;
                    resp.op       = req.is_write ? "write" : "read";
                    resp.index    = req.index;
                    resp.subindex = req.subindex;
                    resp.size     = req.size;
                    if (req.slave_idx < 1) {
                        resp.success = false;
                        resp.error   = "Slave not present (index=" + std::to_string(req.slave_idx) + ")";
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

        // Publish telemetry at pub_hz rate.
        if (now >= next_pub) { try {
            const SystemStatus cur_status = loop.getStatus();
            const LoopStats    cur_stats  = loop.stats();

            DriveGains cmd_gains;
            cmd_gains.torque_kp              = cmd.main_torque_kp;
            cmd_gains.torque_loop_max_output = cmd.main_torque_max_out;
            cmd_gains.torque_loop_min_output = cmd.main_torque_min_out;
            cmd_gains.velocity_loop_kp       = cmd.main_vel_kp;
            cmd_gains.velocity_loop_ki       = cmd.main_vel_ki;
            cmd_gains.velocity_loop_kd       = cmd.main_vel_kd;
            cmd_gains.position_loop_kp       = cmd.main_pos_kp;
            cmd_gains.position_loop_ki       = cmd.main_pos_ki;
            cmd_gains.position_loop_kd       = cmd.main_pos_kd;
            cmd_gains.max_current_a          = cmd.main_max_current_a;

            std::string main_json = JointImpactPreventionTestbench::makeDriveJson(
                drive_slave, drive_soem_idx, cur_status, main_out_enc_bits, cmd_gains);

            // Inject RT algorithm outputs and SDO abs limit so the GUI can
            // display the actual clamped torque and set an accurate slider range.
            try {
                auto jj = json::parse(main_json);
                jj["rt_torque_max"]  = static_cast<double>(testbench.rt_torque_max_out_.load(std::memory_order_relaxed));
                jj["rt_torque_min"]  = static_cast<double>(testbench.rt_torque_min_out_.load(std::memory_order_relaxed));
                jj["torque_abs_max"] = static_cast<double>(testbench.sdo_torque_abs_max_);
                main_json = jj.dump();
            } catch (...) {}

            node->publishTelemetry(
                cur_stats.cycle_count,
                cur_stats.last_wkc,
                static_cast<double>(cur_stats.last_cycle_time_ns) / 1000.0,
                main_json);
            node->publishBusStatus();

            if (debug_print)
                testbench.printDebug(cur_status, cur_stats, cmd);

            next_pub += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(pub_period));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Publish exception: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(node->get_logger(), "Publish unknown exception");
        } }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Shutdown: stop the loop without sending DS402 disable commands.
    {
        const SystemStatus s = loop.getStatus();
        auto it = s.by_slave.find(drive_slave);
        if (it != s.by_slave.end() && it->second.has_value()) {
            const auto& ds = std::any_cast<const DriveStatus&>(it->second);
            RCLCPP_INFO(node->get_logger(), "[shutdown] %s was in %s — stopping loop",
                drive_slave.c_str(), cia402Name(ds.cia402_state));
        }
    }

    safe_stop();
    master.close();

    log_drain.join();
    RCLCPP_INFO(node->get_logger(), "PDO log saved: %s", log_path.c_str());

    executor->cancel();
    ros_thread.join();
    rclcpp::shutdown();
    return 0;
}
