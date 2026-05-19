// Dyno2 UDP Bridge
//
// Runs the EtherCAT loop indefinitely, publishing telemetry as newline-
// delimited JSON over UDP and accepting command packets on a second port.
//
// Telemetry  → localhost:7600  (JSON, sent at --print-hz rate)
// Commands   ← localhost:7601  (JSON, non-blocking receive each main loop tick)
//
// Usage:
//   sudo ./bridge_udp [options]
//
// Options:
//   --topology      <path>   Topology JSON (default: config/topology.dyno2.template6.json)
//   --drive-slave   <name>   Main drive slave name   (default: main_drive)
//   --dut-slave     <name>   DUT drive slave name    (default: dut)
//   --encoder-slave <name>   EL5032 slave name       (default: encoder_interface)
//   --torque-slave  <name>   ELM3002 slave name      (default: analog_input_interface)
//   --io-slave      <name>   EL2004 slave name       (default: digital_IO)
//   --print-hz      <hz>     Telemetry publish rate  (default: 20)
//   --fault-reset   <s>      Fault-reset phase       (default: 0.5)
//   --rt-priority   <1-99>   SCHED_FIFO priority     (default: 0)
//   --cpu-affinity  <cpu>    Comma-separated CPUs    (default: none)
//   --telem-port    <port>   Telemetry UDP port      (default: 7600)
//   --cmd-port      <port>   Command UDP port        (default: 7601)

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/loop.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/default_adapter_factory.hpp"
#include "ethercat_core/devices/beckhoff/el2004/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el2004/data_types.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/adapter.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/data_types.hpp"
#include "ethercat_core/devices/beckhoff/el5032/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el5032/data_types.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/adapter.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"
#include "ethercat_core/devices/motor_drives/drive_bases/ds402/data_types.hpp"
#include "../common/ipc_types.hpp"

extern "C" {
#include "ethercat.h"
}

// UDP / POSIX
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include <getopt.h>
#include <algorithm>
#include <any>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

using namespace ethercat_core;
using namespace ethercat_core::novanta::volcano;
using Cia402State     = ethercat_core::ds402::Cia402State;
using ModeOfOperation = ethercat_core::ds402::ModeOfOperation;
using json            = nlohmann::json;

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{false};
static void onSignal(int) { g_shutdown.store(true); }

// ── Defaults ──────────────────────────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY      = "config/ethercat_device_config/topology.dyno2.template6.json";
static constexpr const char* DEFAULT_DRIVE_SLAVE   = "main_drive";
static constexpr const char* DEFAULT_DUT_SLAVE     = "dut";
static constexpr const char* DEFAULT_ENCODER_SLAVE = "encoder_interface";
static constexpr const char* DEFAULT_TORQUE_SLAVE  = "analog_input_interface";
static constexpr const char* DEFAULT_IO_SLAVE      = "digital_IO";
static constexpr double      DEFAULT_PRINT_HZ      = 20.0;
static constexpr double      DEFAULT_FAULT_RESET   = 0.5;

struct Args {
    std::string   topology        = DEFAULT_TOPOLOGY;
    std::string   drive_slave     = DEFAULT_DRIVE_SLAVE;
    std::string   dut_slave       = DEFAULT_DUT_SLAVE;
    std::string   encoder_slave   = DEFAULT_ENCODER_SLAVE;
    std::string   torque_slave    = DEFAULT_TORQUE_SLAVE;
    std::string   io_slave        = DEFAULT_IO_SLAVE;
    double        print_hz        = DEFAULT_PRINT_HZ;
    double        fault_reset_s   = DEFAULT_FAULT_RESET;
    int           rt_priority     = 0;
    std::set<int> cpu_affinity;
    uint16_t      telem_port      = dyno::ipc::TELEMETRY_PORT;
    uint16_t      cmd_port        = dyno::ipc::COMMAND_PORT;
};

static std::set<int> parseCpuAffinity(const char* str) {
    std::set<int> cpus;
    std::istringstream ss(str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (token.empty()) continue;
        cpus.insert(std::stoi(token));
    }
    return cpus;
}

static Args parseArgs(int argc, char** argv) {
    Args a;
    static struct option long_opts[] = {
        {"topology",        required_argument, nullptr, 't'},
        {"drive-slave",     required_argument, nullptr, 'D'},
        {"dut-slave",       required_argument, nullptr, 'U'},
        {"encoder-slave",   required_argument, nullptr, 'E'},
        {"torque-slave",    required_argument, nullptr, 'T'},
        {"io-slave",        required_argument, nullptr, 'I'},
        {"print-hz",        required_argument, nullptr, 'p'},
        {"fault-reset",     required_argument, nullptr, 'f'},
        {"rt-priority",     required_argument, nullptr, 'r'},
        {"cpu-affinity",    required_argument, nullptr, 'c'},
        {"telem-port",      required_argument, nullptr, 'P'},
        {"cmd-port",        required_argument, nullptr, 'C'},
        {"help",            no_argument,       nullptr, 'h'},
        {nullptr,           0,                 nullptr,  0 },
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "t:D:U:E:T:I:p:f:r:c:P:C:h", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 't': a.topology      = optarg;                       break;
        case 'D': a.drive_slave   = optarg;                       break;
        case 'U': a.dut_slave     = optarg;                       break;
        case 'E': a.encoder_slave = optarg;                       break;
        case 'T': a.torque_slave  = optarg;                       break;
        case 'I': a.io_slave      = optarg;                       break;
        case 'p': a.print_hz      = std::stod(optarg);            break;
        case 'f': a.fault_reset_s = std::stod(optarg);            break;
        case 'r': a.rt_priority   = std::stoi(optarg);            break;
        case 'c': a.cpu_affinity  = parseCpuAffinity(optarg);     break;
        case 'P': a.telem_port    = static_cast<uint16_t>(std::stoi(optarg)); break;
        case 'C': a.cmd_port      = static_cast<uint16_t>(std::stoi(optarg)); break;
        case 'h': std::printf(
            "Usage: bridge_udp [options]\n"
            "  --topology/--drive-slave/--dut-slave/--encoder-slave\n"
            "  --torque-slave/--io-slave/--print-hz/--fault-reset\n"
            "  --rt-priority/--cpu-affinity/--telem-port/--cmd-port\n");
            std::exit(0);
        default: std::exit(2);
        }
    }
    return a;
}

// ── DS402 helpers ─────────────────────────────────────────────────────────────

static const char* cia402Name(Cia402State s) {
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

// ── UDP helpers ───────────────────────────────────────────────────────────────

static int makeTelemSocket(uint16_t port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) throw std::runtime_error("Failed to create telemetry socket");
    // Connect to localhost so send() works without specifying destination.
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("Failed to connect telemetry socket");
    return fd;
}

static int makeCmdSocket(uint16_t port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) throw std::runtime_error("Failed to create command socket");
    // Non-blocking so receive doesn't stall the main loop.
    fcntl(fd, F_SETFL, O_NONBLOCK);
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("Failed to bind command socket");
    return fd;
}

// Serialize Telemetry → compact JSON string (no trailing newline here).
static std::string serializeTelemetry(const dyno::ipc::Telemetry& t) {
    auto drive_to_json = [](const dyno::ipc::DriveTelementry& d) {
        return json{
            {"al",       d.al_state},
            {"state",    d.cia402_state},
            {"cmd_vel",  d.cmd_velocity},
            {"fb_vel",   d.fb_velocity},
            {"mode",     d.mode_display},
            {"sw",       d.status_word},
            {"err",      d.error_code},
        };
    };
    json j{
        {"cycle",   t.cycle},
        {"wkc",     t.wkc},
        {"t_us",    t.cycle_us},
        {"main",    drive_to_json(t.main_drive)},
        {"dut",     drive_to_json(t.dut)},
        {"enc",     t.encoder_count},
        {"ch1_v",   t.ch1_voltage},
        {"ch1_t",   t.ch1_torque},
        {"ch2_v",   t.ch2_voltage},
        {"ch2_t",   t.ch2_torque},
        {"out1",    t.out1},
    };
    return j.dump();
}

// Deserialize JSON → Command. Returns false if parse fails.
static bool parseCommand(const char* buf, int len, dyno::ipc::Command& out) {
    try {
        const json j = json::parse(buf, buf + len);
        out.main_speed   = j.value("main_speed",   0);
        out.dut_speed    = j.value("dut_speed",    0);
        out.main_enable  = j.value("main_enable",  false);
        out.dut_enable   = j.value("dut_enable",   false);
        out.fault_reset  = j.value("fault_reset",  false);
        out.hold_output1 = j.value("hold_output1", false);
        return true;
    } catch (...) {
        return false;
    }
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);

    MasterConfig cfg;
    try {
        cfg = loadTopology(args.topology);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load topology: %s\n", e.what());
        return 1;
    }

    EthercatMaster master(cfg, ethercat_core::makeDefaultAdapterFactory());
    MasterRuntime* rt = nullptr;
    try {
        rt = &master.initialize();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Master init failed: %s\n", e.what());
        return 1;
    }

    // Validate slaves.
    for (const auto& name : {args.drive_slave, args.dut_slave,
                              args.encoder_slave, args.torque_slave, args.io_slave}) {
        if (rt->adapters.find(name) == rt->adapters.end()) {
            std::fprintf(stderr, "Slave '%s' not found in topology.\n", name.c_str());
            master.close();
            return 1;
        }
    }

    auto* elm3002 = dynamic_cast<beckhoff::elm3002::Elm3002Adapter*>(
        rt->adapters.at(args.torque_slave).get());
    if (!elm3002) {
        std::fprintf(stderr, "Slave '%s' is not an ELM3002.\n", args.torque_slave.c_str());
        master.close();
        return 1;
    }

    const int drive_soem_idx = rt->slave_index.at(args.drive_slave);
    const int dut_soem_idx   = rt->slave_index.at(args.dut_slave);

    // UDP sockets.
    int telem_fd = -1, cmd_fd = -1;
    try {
        telem_fd = makeTelemSocket(args.telem_port);
        cmd_fd   = makeCmdSocket(args.cmd_port);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "UDP socket error: %s\n", e.what());
        master.close();
        return 1;
    }

    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority  = std::clamp(args.rt_priority, 0, 99);
    rt_cfg.cpu_affinity = args.cpu_affinity;

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);
    loop.start();

    std::printf(
        "bridge_udp running | telem=localhost:%d  cmd=localhost:%d\n"
        "Ctrl+C to stop.\n",
        args.telem_port, args.cmd_port
    );

    const auto   t0           = std::chrono::steady_clock::now();
    const auto   reset_end    = t0 + std::chrono::duration<double>(args.fault_reset_s);
    const double print_period = 1.0 / std::max(args.print_hz, 1.0);
    auto         next_print   = t0;

    // Initial command state — updated when UDP commands arrive.
    dyno::ipc::Command ipc_cmd;

    char cmd_buf[dyno::ipc::MAX_PACKET];

    while (!g_shutdown.load()) {
        const auto now      = std::chrono::steady_clock::now();
        const bool in_reset = now < reset_end;

        // ── Receive incoming command (non-blocking) ───────────────────────────
        const ssize_t n = recv(cmd_fd, cmd_buf, sizeof(cmd_buf) - 1, 0);
        if (n > 0) {
            cmd_buf[n] = '\0';
            dyno::ipc::Command parsed;
            if (parseCommand(cmd_buf, static_cast<int>(n), parsed))
                ipc_cmd = parsed;
        }

        // ── Build EtherCAT commands ───────────────────────────────────────────
        const bool main_enable = !in_reset && ipc_cmd.main_enable;
        const bool dut_enable  = !in_reset && ipc_cmd.dut_enable;

        Command main_cmd;
        main_cmd.mode_of_operation     = ModeOfOperation::CYCLIC_SYNC_VELOCITY;
        main_cmd.target_velocity_mrevs = static_cast<float>(ipc_cmd.main_speed);
        main_cmd.enable_drive          = main_enable;
        main_cmd.clear_fault           = in_reset || ipc_cmd.fault_reset;

        Command dut_cmd;
        dut_cmd.mode_of_operation     = ModeOfOperation::CYCLIC_SYNC_VELOCITY;
        dut_cmd.target_velocity_mrevs = static_cast<float>(ipc_cmd.dut_speed);
        dut_cmd.enable_drive          = dut_enable;
        dut_cmd.clear_fault           = in_reset || ipc_cmd.fault_reset;

        beckhoff::el2004::Command io_cmd;
        io_cmd.output_1 = ipc_cmd.hold_output1;

        SystemCommand sys_cmd;
        sys_cmd.by_slave[args.drive_slave] = main_cmd;
        sys_cmd.by_slave[args.dut_slave]   = dut_cmd;
        sys_cmd.by_slave[args.io_slave]    = io_cmd;
        loop.setCommand(sys_cmd);

        // ── Publish telemetry at print_hz rate ────────────────────────────────
        if (now >= next_print) {
            const SystemStatus status = loop.getStatus();
            const LoopStats    stats  = loop.stats();

            dyno::ipc::Telemetry telem;
            telem.cycle    = stats.cycle_count;
            telem.wkc      = stats.last_wkc;
            telem.cycle_us = static_cast<double>(stats.last_cycle_time_ns) / 1000.0;

            auto fill_drive = [&](const std::string& slave_name,
                                  int soem_idx,
                                  int32_t cmd_vel,
                                  dyno::ipc::DriveTelementry& out) {
                out.al_state = alStateName(static_cast<int>(ec_slave[soem_idx].state));
                auto it = status.by_slave.find(slave_name);
                if (it != status.by_slave.end() && it->second.has_value()) {
                    const auto& ds = std::any_cast<const DriveStatus&>(it->second);
                    out.cia402_state  = cia402Name(ds.cia402_state);
                    out.cmd_velocity  = cmd_vel;
                    out.fb_velocity   = ds.measured_input_side_velocity_raw;
                    out.mode_display  = static_cast<int>(ds.mode_of_operation_display);
                    out.status_word   = ds.status_word;
                    out.error_code    = ds.error_code;
                } else {
                    out.cia402_state = "unavailable";
                }
            };

            fill_drive(args.drive_slave, drive_soem_idx, ipc_cmd.main_speed, telem.main_drive);
            fill_drive(args.dut_slave,   dut_soem_idx,   ipc_cmd.dut_speed,  telem.dut);

            auto enc_it = status.by_slave.find(args.encoder_slave);
            if (enc_it != status.by_slave.end() && enc_it->second.has_value())
                telem.encoder_count = std::any_cast<const beckhoff::el5032::Data&>(
                    enc_it->second).encoder_count_25bit;

            auto torque_it = status.by_slave.find(args.torque_slave);
            if (torque_it != status.by_slave.end() && torque_it->second.has_value()) {
                const auto& d = std::any_cast<const beckhoff::elm3002::Data&>(torque_it->second);
                telem.ch1_voltage = static_cast<double>(
                    beckhoff::elm3002::Elm3002Adapter::scaleAdcToVoltage(d.pai_samples_1));
                telem.ch2_voltage = static_cast<double>(
                    beckhoff::elm3002::Elm3002Adapter::scaleAdcToVoltage(d.pai_samples_2));
                telem.ch1_torque  = static_cast<double>(elm3002->scaledTorqueCh1(d));
                telem.ch2_torque  = static_cast<double>(elm3002->scaledTorqueCh2(d));
            }

            telem.out1 = ipc_cmd.hold_output1;

            const std::string packet = serializeTelemetry(telem) + "\n";
            send(telem_fd, packet.c_str(), packet.size(), 0);

            next_print += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(print_period));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // ── Graceful drive disable ────────────────────────────────────────────────
    std::printf("Disabling drives (graceful shutdown)...\n");
    {
        Command disable;
        disable.enable_drive = false;
        disable.clear_fault  = false;

        SystemCommand sys;
        sys.by_slave[args.drive_slave] = disable;
        sys.by_slave[args.dut_slave]   = disable;
        sys.by_slave[args.io_slave]    = beckhoff::el2004::Command{};
        loop.setCommand(sys);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    loop.stop();
    master.close();
    close(telem_fd);
    close(cmd_fd);
    std::printf("Bridge stopped.\n");
    return 0;
}
