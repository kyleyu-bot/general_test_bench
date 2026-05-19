// Integrated dyno test: drive speed command + EL2004 digital output +
// EL5032 encoder readback + ELM3002 torque readback.
//
// All four devices share a single EthercatLoop.  One combined status line
// is printed per print period.
//
// Usage:
//   sudo ./dyno2_integrated_test1 [options]
//
// Options:
//   --topology      <path>   Topology JSON (default: config/topology.dyno2.template6.json)
//   --drive-slave   <name>   Volcano/DS402 drive slave name (default: main_drive)
//   --encoder-slave <name>   EL5032 slave name (default: encoder_interface)
//   --torque-slave  <name>   ELM3002 slave name (default: analog_input_interface)
//   --io-slave      <name>   EL2004 slave name (default: digital_IO)
//   --speed         <int>    Target velocity sent to 0x60FF (default: 0)
//   --mode          <int>    Mode of operation 0x6060 (default: 9 = CSV)
//   --duration      <s>      Total test duration (default: 60)
//   --fault-reset   <s>      Fault-reset phase at start (default: 0.5)
//   --print-hz      <hz>     Status print rate (default: 5)
//   --hold-output1           Hold EL2004 output 1 high for the entire test
//   --write-startup-sdos     Write loaded startup SDO gains back via SDO
//   --force-sdo-mode         Force-write 0x6060 once via SDO before loop
//   --debug-gain-sdos        Print raw SDO values for 0x250A/0x250B at startup
//   --rt-priority   <1-99>   Loop thread SCHED_FIFO priority (0 = default)
//   --cpu-affinity  <cpu>    Comma-separated CPU indices (e.g. 2 or 2,3)

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

extern "C" {
#include "ethercat.h"
}

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

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{false};
static void onSignal(int) { g_shutdown.store(true); }

// ── Argument defaults ─────────────────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY      = "config/ethercat_device_config/topology.dyno2.template6.json";
static constexpr const char* DEFAULT_DRIVE_SLAVE   = "main_drive";
static constexpr const char* DEFAULT_ENCODER_SLAVE = "encoder_interface";
static constexpr const char* DEFAULT_TORQUE_SLAVE  = "analog_input_interface";
static constexpr const char* DEFAULT_IO_SLAVE      = "digital_IO";
static constexpr int         DEFAULT_SPEED         = 0;
static constexpr int         DEFAULT_MODE          = 9;   // CSV
static constexpr double      DEFAULT_DURATION      = 60.0;
static constexpr double      DEFAULT_FAULT_RESET   = 0.5;
static constexpr double      DEFAULT_PRINT_HZ      = 5.0;

struct Args {
    std::string   topology        = DEFAULT_TOPOLOGY;
    std::string   drive_slave     = DEFAULT_DRIVE_SLAVE;
    std::string   encoder_slave   = DEFAULT_ENCODER_SLAVE;
    std::string   torque_slave    = DEFAULT_TORQUE_SLAVE;
    std::string   io_slave        = DEFAULT_IO_SLAVE;
    int32_t       speed           = DEFAULT_SPEED;
    int           mode            = DEFAULT_MODE;
    double        duration_s      = DEFAULT_DURATION;
    double        fault_reset_s   = DEFAULT_FAULT_RESET;
    double        print_hz        = DEFAULT_PRINT_HZ;
    bool          hold_output1    = false;
    bool          write_startup   = false;
    bool          force_sdo_mode  = false;
    bool          debug_gain_sdos = false;
    int           rt_priority     = 0;
    std::set<int> cpu_affinity;
};

static void printUsage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  --topology      <path>   Topology JSON        (default: %s)\n"
        "  --drive-slave   <name>   Drive slave name     (default: %s)\n"
        "  --encoder-slave <name>   EL5032 slave name    (default: %s)\n"
        "  --torque-slave  <name>   ELM3002 slave name   (default: %s)\n"
        "  --io-slave      <name>   EL2004 slave name    (default: %s)\n"
        "  --speed         <int>    Velocity cmd 0x60FF  (default: %d)\n"
        "  --mode          <int>    Mode 0x6060          (default: %d = CSV)\n"
        "  --duration      <s>      Test duration        (default: %.1f s)\n"
        "  --fault-reset   <s>      Fault-reset phase    (default: %.1f s)\n"
        "  --print-hz      <hz>     Print rate           (default: %.1f Hz)\n"
        "  --hold-output1           Hold EL2004 output 1 high during test\n"
        "  --write-startup-sdos     Write startup gains back via SDO\n"
        "  --force-sdo-mode         Force-write 0x6060 via SDO before loop\n"
        "  --debug-gain-sdos        Print raw 0x250A/0x250B SDO values\n"
        "  --rt-priority   <1-99>   SCHED_FIFO priority  (0 = default)\n"
        "  --cpu-affinity  <cpu>    CPU index(es), comma-separated\n",
        prog,
        DEFAULT_TOPOLOGY,
        DEFAULT_DRIVE_SLAVE, DEFAULT_ENCODER_SLAVE,
        DEFAULT_TORQUE_SLAVE, DEFAULT_IO_SLAVE,
        DEFAULT_SPEED, DEFAULT_MODE,
        DEFAULT_DURATION, DEFAULT_FAULT_RESET, DEFAULT_PRINT_HZ
    );
}

static std::set<int> parseCpuAffinity(const char* str) {
    std::set<int> cpus;
    std::istringstream ss(str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (token.empty()) continue;
        int cpu = std::stoi(token);
        if (cpu < 0) throw std::invalid_argument("CPU index must be >= 0");
        cpus.insert(cpu);
    }
    if (cpus.empty()) throw std::invalid_argument("cpu-affinity needs at least one CPU");
    return cpus;
}

static Args parseArgs(int argc, char** argv) {
    Args a;
    static struct option long_opts[] = {
        {"topology",           required_argument, nullptr, 't'},
        {"drive-slave",        required_argument, nullptr, 'D'},
        {"encoder-slave",      required_argument, nullptr, 'E'},
        {"torque-slave",       required_argument, nullptr, 'T'},
        {"io-slave",           required_argument, nullptr, 'I'},
        {"speed",              required_argument, nullptr, 'S'},
        {"mode",               required_argument, nullptr, 'm'},
        {"duration",           required_argument, nullptr, 'd'},
        {"fault-reset",        required_argument, nullptr, 'f'},
        {"print-hz",           required_argument, nullptr, 'p'},
        {"hold-output1",       no_argument,       nullptr, 'o'},
        {"write-startup-sdos", no_argument,       nullptr, 'w'},
        {"force-sdo-mode",     no_argument,       nullptr, 'F'},
        {"debug-gain-sdos",    no_argument,       nullptr, 'g'},
        {"rt-priority",        required_argument, nullptr, 'r'},
        {"cpu-affinity",       required_argument, nullptr, 'c'},
        {"help",               no_argument,       nullptr, 'h'},
        {nullptr,              0,                 nullptr,  0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "t:D:E:T:I:S:m:d:f:p:owFgr:c:h", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 't': a.topology        = optarg;             break;
        case 'D': a.drive_slave     = optarg;             break;
        case 'E': a.encoder_slave   = optarg;             break;
        case 'T': a.torque_slave    = optarg;             break;
        case 'I': a.io_slave        = optarg;             break;
        case 'S': a.speed           = std::stoi(optarg);  break;
        case 'm': a.mode            = std::stoi(optarg);  break;
        case 'd': a.duration_s      = std::stod(optarg);  break;
        case 'f': a.fault_reset_s   = std::stod(optarg);  break;
        case 'p': a.print_hz        = std::stod(optarg);  break;
        case 'o': a.hold_output1    = true;               break;
        case 'w': a.write_startup   = true;               break;
        case 'F': a.force_sdo_mode  = true;               break;
        case 'g': a.debug_gain_sdos = true;               break;
        case 'r': a.rt_priority     = std::stoi(optarg);  break;
        case 'c': a.cpu_affinity    = parseCpuAffinity(optarg); break;
        case 'h': printUsage(argv[0]); std::exit(0);
        default:  printUsage(argv[0]); std::exit(2);
        }
    }
    return a;
}

// ── SDO helpers ───────────────────────────────────────────────────────────────

static void writeSdoF32(int soem_idx, uint16_t index, uint8_t subindex, float value) {
    for (int attempt = 0; attempt < 5; ++attempt) {
        int ret = ec_SDOwrite(static_cast<uint16>(soem_idx), index, subindex,
                              FALSE, static_cast<int>(sizeof(value)), &value, EC_TIMEOUTRET);
        if (ret > 0) return;
        if (attempt < 4) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::fprintf(stderr, "WARNING: SDO write failed at 0x%04X:%02X\n", index, subindex);
}

static void writeSdoI8(int soem_idx, uint16_t index, uint8_t subindex, int8_t value) {
    for (int attempt = 0; attempt < 5; ++attempt) {
        int ret = ec_SDOwrite(static_cast<uint16>(soem_idx), index, subindex,
                              FALSE, 1, &value, EC_TIMEOUTRET);
        if (ret > 0) return;
        if (attempt < 4) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::fprintf(stderr, "WARNING: SDO write failed at 0x%04X:%02X\n", index, subindex);
}

static void debugGainRegisters(int soem_idx) {
    struct { const char* name; uint16_t index; } regs[] = {
        {"velocity_loop_kp", 0x250A},
        {"velocity_loop_ki", 0x250B},
    };
    for (auto& r : regs) {
        uint8_t buf[8] = {};
        int sz = sizeof(buf);
        int ret = ec_SDOread(static_cast<uint16>(soem_idx), r.index, 0x00, FALSE, &sz, buf, EC_TIMEOUTRET);
        if (ret <= 0) { std::printf("%s 0x%04X:00 sdo_read failed\n", r.name, r.index); continue; }
        float f32 = 0.0f; std::memcpy(&f32, buf, 4);
        uint32_t u32 = 0; std::memcpy(&u32, buf, 4);
        int32_t  s32 = 0; std::memcpy(&s32, buf, 4);
        std::printf("%s 0x%04X:00  f32=%.6f  u32=%u  s32=%d\n",
                    r.name, r.index, static_cast<double>(f32), u32, s32);
    }
}

static void applyStartupParams(const std::unordered_map<std::string, float>& params, int soem_idx) {
    static const struct { const char* key; uint16_t index; } kMap[] = {
        {"torque_loop_max_output", 0x2527},
        {"torque_loop_min_output", 0x2528},
        {"velocity_loop_kp",       0x250A},
        {"velocity_loop_ki",       0x250B},
        {"velocity_loop_kd",       0x250C},
        {"position_loop_kp",       0x2511},
        {"position_loop_ki",       0x2512},
        {"position_loop_kd",       0x2513},
        {"motor_kt",               0x243B},
    };
    for (auto& m : kMap) {
        auto it = params.find(m.key);
        if (it == params.end()) continue;
        writeSdoF32(soem_idx, m.index, 0x00, it->second);
    }
}

// ── DS402 state name ──────────────────────────────────────────────────────────

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

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);

    const auto cmd_mode = static_cast<ModeOfOperation>(args.mode);

    MasterConfig cfg;
    try {
        cfg = loadTopology(args.topology);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load topology '%s': %s\n", args.topology.c_str(), e.what());
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

    // Validate all required slaves are present.
    for (const auto& name : {args.drive_slave, args.encoder_slave, args.torque_slave, args.io_slave}) {
        if (rt->adapters.find(name) == rt->adapters.end()) {
            std::fprintf(stderr, "Slave '%s' not found in topology. Available:", name.c_str());
            for (auto& [k, _] : rt->adapters) std::fprintf(stderr, " %s", k.c_str());
            std::fprintf(stderr, "\n");
            master.close();
            return 1;
        }
    }

    auto* elm3002 = dynamic_cast<beckhoff::elm3002::Elm3002Adapter*>(
        rt->adapters.at(args.torque_slave).get());
    if (!elm3002) {
        std::fprintf(stderr, "Slave '%s' is not an ELM3002 adapter.\n", args.torque_slave.c_str());
        master.close();
        return 1;
    }

    if (!dynamic_cast<beckhoff::el5032::El5032Adapter*>(rt->adapters.at(args.encoder_slave).get())) {
        std::fprintf(stderr, "Slave '%s' is not an EL5032 adapter.\n", args.encoder_slave.c_str());
        master.close();
        return 1;
    }

    const int drive_soem_idx = rt->slave_index.at(args.drive_slave);

    // Extract startup params.
    float torque_kp = 0.0f, vel_qr = 0.0f, vel_is = 0.0f;
    float vel_kp = 0.0f, vel_ki = 0.0f, vel_kd = 0.0f;
    float pos_kp = 0.0f, pos_ki = 0.0f, pos_kd = 0.0f;

    auto it_params = rt->startup_params.find(args.drive_slave);
    if (it_params != rt->startup_params.end()) {
        const auto& p = it_params->second;
        auto get = [&](const char* k) -> float {
            auto it = p.find(k); return it != p.end() ? it->second : 0.0f;
        };
        const float kt = get("motor_kt");
        torque_kp = (std::abs(kt) > 1e-9f) ? (1.0f / kt) : 0.0f;
        vel_qr = get("torque_loop_max_output");
        vel_is = get("torque_loop_min_output");
        vel_kp = get("velocity_loop_kp");
        vel_ki = get("velocity_loop_ki");
        vel_kd = get("velocity_loop_kd");
        pos_kp = get("position_loop_kp");
        pos_ki = get("position_loop_ki");
        pos_kd = get("position_loop_kd");
    }

    if (args.debug_gain_sdos) debugGainRegisters(drive_soem_idx);

    if (args.write_startup && it_params != rt->startup_params.end()) {
        applyStartupParams(it_params->second, drive_soem_idx);
        std::printf("Wrote startup gains back via SDO.\n");
    }

    if (args.force_sdo_mode) {
        writeSdoI8(drive_soem_idx, 0x6060, 0x00, static_cast<int8_t>(args.mode));
        std::printf("Forced SDO mode write: 0x6060=%d\n", args.mode);
    }

    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority  = std::clamp(args.rt_priority, 0, 99);
    rt_cfg.cpu_affinity = args.cpu_affinity;

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);
    loop.start();

    std::printf(
        "Starting integrated dyno test | "
        "speed_cmd=%d duration=%.1fs fault_reset=%.1fs | "
        "rt_priority=%d hold_output1=%s\n",
        args.speed, args.duration_s, args.fault_reset_s,
        std::clamp(args.rt_priority, 0, 99),
        args.hold_output1 ? "yes" : "no"
    );

    const auto   t0           = std::chrono::steady_clock::now();
    const auto   deadline     = t0 + std::chrono::duration<double>(std::max(0.0, args.duration_s));
    const auto   reset_end    = t0 + std::chrono::duration<double>(std::max(0.0, args.fault_reset_s));
    const double print_period = 1.0 / std::max(args.print_hz, 0.1);
    auto         next_print   = t0;

    while (!g_shutdown.load() && std::chrono::steady_clock::now() < deadline) {
        const auto now      = std::chrono::steady_clock::now();
        const bool in_reset = now < reset_end;

        // ── Drive command ────────────────────────────────────────────────────
        Command drive_cmd;
        drive_cmd.mode_of_operation      = cmd_mode;
        drive_cmd.target_velocity_mrevs  = static_cast<float>(args.speed);
        drive_cmd.torque_kp              = torque_kp;
        drive_cmd.torque_loop_max_output = vel_qr;
        drive_cmd.torque_loop_min_output = vel_is;
        drive_cmd.velocity_loop_kp       = vel_kp;
        drive_cmd.velocity_loop_ki       = vel_ki;
        drive_cmd.velocity_loop_kd       = vel_kd;
        drive_cmd.position_loop_kp       = pos_kp;
        drive_cmd.position_loop_ki       = pos_ki;
        drive_cmd.position_loop_kd       = pos_kd;
        drive_cmd.enable_drive           = !in_reset;
        drive_cmd.clear_fault            = in_reset;

        // ── EL2004 command ───────────────────────────────────────────────────
        beckhoff::el2004::Command io_cmd;
        io_cmd.output_1 = args.hold_output1;

        SystemCommand sys_cmd;
        sys_cmd.by_slave[args.drive_slave] = drive_cmd;
        sys_cmd.by_slave[args.io_slave]    = io_cmd;
        loop.setCommand(sys_cmd);

        // ── Print ────────────────────────────────────────────────────────────
        if (now >= next_print) {
            const SystemStatus status = loop.getStatus();
            const LoopStats    stats  = loop.stats();

            // Drive.
            const std::string drive_al = alStateName(
                static_cast<int>(ec_slave[drive_soem_idx].state));
            auto drive_it = status.by_slave.find(args.drive_slave);
            char drive_buf[128];
            if (drive_it == status.by_slave.end() || !drive_it->second.has_value()) {
                std::snprintf(drive_buf, sizeof(drive_buf),
                    "al=%s state=unavailable cmd=%d fb=unavailable",
                    drive_al.c_str(), args.speed);
            } else {
                const auto& ds = std::any_cast<const DriveStatus&>(drive_it->second);
                std::snprintf(drive_buf, sizeof(drive_buf),
                    "al=%s state=%s cmd=%d fb=%d mode=%d sw=0x%04X err=0x%04X",
                    drive_al.c_str(),
                    cia402Name(ds.cia402_state),
                    args.speed,
                    ds.measured_input_side_velocity_raw,
                    static_cast<int>(ds.mode_of_operation_display),
                    static_cast<unsigned>(ds.status_word),
                    static_cast<unsigned>(ds.error_code));
            }

            // Encoder.
            char enc_buf[64];
            auto enc_it = status.by_slave.find(args.encoder_slave);
            if (enc_it == status.by_slave.end() || !enc_it->second.has_value()) {
                std::snprintf(enc_buf, sizeof(enc_buf), "enc=unavailable");
            } else {
                const auto& d = std::any_cast<const beckhoff::el5032::Data&>(enc_it->second);
                std::snprintf(enc_buf, sizeof(enc_buf), "enc=%u", d.encoder_count_25bit);
            }

            // Torque (ELM3002).
            char torque_buf[128];
            auto torque_it = status.by_slave.find(args.torque_slave);
            if (torque_it == status.by_slave.end() || !torque_it->second.has_value()) {
                std::snprintf(torque_buf, sizeof(torque_buf),
                    "ch1_v=unavailable ch1_t=unavailable ch2_v=unavailable ch2_t=unavailable");
            } else {
                const auto& d = std::any_cast<const beckhoff::elm3002::Data&>(torque_it->second);
                const float ch1_v = beckhoff::elm3002::Elm3002Adapter::scaleAdcToVoltage(d.pai_samples_1);
                const float ch2_v = beckhoff::elm3002::Elm3002Adapter::scaleAdcToVoltage(d.pai_samples_2);
                const float ch1_t = elm3002->scaledTorqueCh1(d);
                const float ch2_t = elm3002->scaledTorqueCh2(d);
                std::snprintf(torque_buf, sizeof(torque_buf),
                    "ch1_v=%.4f ch1_t=%.4f ch2_v=%.4f ch2_t=%.4f",
                    static_cast<double>(ch1_v), static_cast<double>(ch1_t),
                    static_cast<double>(ch2_v), static_cast<double>(ch2_t));
            }

            std::printf(
                "cycle=%lu wkc=%d cycle_us=%.1f | "
                "%s | %s | %s | out1=%d\n",
                static_cast<unsigned long>(stats.cycle_count),
                stats.last_wkc,
                static_cast<double>(stats.last_cycle_time_ns) / 1000.0,
                drive_buf, enc_buf, torque_buf,
                args.hold_output1 ? 1 : 0
            );

            next_print += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(print_period));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // ── Graceful shutdown ─────────────────────────────────────────────────────
    // Send enable_drive=false, then poll until the drive leaves OPERATION_ENABLED
    // before calling loop.stop().  Without this the watchdog fires the instant
    // cyclic PDO stops, landing the drive in FAULT.
    // Stop directly — any DS402 walkback through READY_TO_SWITCH_ON resets
    // the Capitan drive's velocity gain registers.  PDO watchdog handles AL exit.
    {
        const SystemStatus s = loop.getStatus();
        auto it = s.by_slave.find(args.drive_slave);
        if (it != s.by_slave.end() && it->second.has_value()) {
            const auto& ds = std::any_cast<const DriveStatus&>(it->second);
            std::printf("[shutdown] %s was in %s — stopping loop\n",
                args.drive_slave.c_str(), cia402Name(ds.cia402_state));
        }
    }
    std::printf("Stopping loop.\n");

    loop.stop();
    master.close();
    return 0;
}
