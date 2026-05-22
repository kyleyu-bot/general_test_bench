// Simple DS402 velocity command/readback test.
//
// Usage:
//   sudo ./drive_simple_speed_test1 [options]
//
// Options:
//   --topology <path>     Topology JSON file (default: config/topology.debug.json)
//   --slave    <name>     Configured slave name (default: main_drive)
//   --speed    <int>      Target velocity as int32 for 0x60FF (default: 1000)
//   --mode     <int>      Mode of operation for 0x6060 (default: 9 = CSV)
//   --duration <s>        Total test duration in seconds (default: 60)
//   --fault-reset <s>     Fault-reset phase at start in seconds (default: 0.5)
//   --print-hz <hz>       Status print rate (default: 5)
//   --write-startup-sdos  Write loaded startup SDO values back to drive before loop
//   --force-sdo-mode      Force-write 0x6060 once via SDO before loop start
//   --debug-gain-sdos     Print raw + decoded SDO values for 0x250A/0x250B at startup
//   --rt-priority <1-99>  Loop thread SCHED_FIFO priority (0 = default scheduler)
//   --cpu-affinity <cpu>  CPU index to pin the loop thread to (e.g. 2)

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/loop.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/default_adapter_factory.hpp"
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
#include <cstdio>
#include <cstring>
#include <csignal>
#include <stdexcept>
#include <string>
#include <thread>

static std::atomic<bool> g_shutdown{false};
static void onSignal(int) { g_shutdown.store(true); }

using namespace ethercat_core;
using namespace ethercat_core::novanta::volcano;
using Cia402State     = ethercat_core::ds402::Cia402State;
using ModeOfOperation = ethercat_core::ds402::ModeOfOperation;

// ── Argument defaults ─────────────────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY   = "config/ethercat_device_config/topology.dyno2.template6.json";
static constexpr const char* DEFAULT_SLAVE      = "main_drive";
static constexpr int         DEFAULT_SPEED       = 1000;
static constexpr int         DEFAULT_MODE        = 9;   // CYCLIC_SYNC_VELOCITY
static constexpr double      DEFAULT_DURATION_S  = 60.0;
static constexpr double      DEFAULT_FAULT_RESET = 0.5;
static constexpr double      DEFAULT_PRINT_HZ    = 5.0;

struct Args {
    std::string topology         = DEFAULT_TOPOLOGY;
    std::string slave            = DEFAULT_SLAVE;
    int32_t     speed            = DEFAULT_SPEED;
    int         mode             = DEFAULT_MODE;
    double      duration_s       = DEFAULT_DURATION_S;
    double      fault_reset_s    = DEFAULT_FAULT_RESET;
    double      print_hz         = DEFAULT_PRINT_HZ;
    bool        write_startup    = false;
    bool        force_sdo_mode   = false;
    bool        debug_gain_sdos  = false;
    int         rt_priority      = 0;
    int         cpu_affinity     = -1;   // -1 = no affinity
};

static void printUsage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  --topology <path>     Topology JSON  (default: %s)\n"
        "  --slave    <name>     Slave name     (default: %s)\n"
        "  --speed    <int>      Velocity cmd   (default: %d)\n"
        "  --mode     <int>      Mode 0x6060    (default: %d = CSV)\n"
        "  --duration <s>        Test duration  (default: %.1f s)\n"
        "  --fault-reset <s>     Fault-reset phase (default: %.1f s)\n"
        "  --print-hz <hz>       Print rate     (default: %.1f Hz)\n"
        "  --write-startup-sdos  Write startup gains back via SDO\n"
        "  --force-sdo-mode      Force-write 0x6060 via SDO before loop\n"
        "  --debug-gain-sdos     Print raw SDO values for 0x250A/0x250B\n"
        "  --rt-priority <1-99>  SCHED_FIFO priority (0 = default)\n"
        "  --cpu-affinity <cpu>  CPU index to pin loop thread to\n",
        prog,
        DEFAULT_TOPOLOGY, DEFAULT_SLAVE, DEFAULT_SPEED, DEFAULT_MODE,
        DEFAULT_DURATION_S, DEFAULT_FAULT_RESET, DEFAULT_PRINT_HZ
    );
}

static Args parseArgs(int argc, char** argv) {
    Args a;
    static struct option long_opts[] = {
        {"topology",           required_argument, nullptr, 't'},
        {"slave",              required_argument, nullptr, 's'},
        {"speed",              required_argument, nullptr, 'S'},
        {"mode",               required_argument, nullptr, 'm'},
        {"duration",           required_argument, nullptr, 'd'},
        {"fault-reset",        required_argument, nullptr, 'f'},
        {"print-hz",           required_argument, nullptr, 'p'},
        {"write-startup-sdos", no_argument,       nullptr, 'w'},
        {"force-sdo-mode",     no_argument,       nullptr, 'F'},
        {"debug-gain-sdos",    no_argument,       nullptr, 'g'},
        {"rt-priority",        required_argument, nullptr, 'r'},
        {"cpu-affinity",       required_argument, nullptr, 'c'},
        {"help",               no_argument,       nullptr, 'h'},
        {nullptr,              0,                 nullptr,  0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "t:s:S:m:d:f:p:wFgr:c:h", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 't': a.topology       = optarg;            break;
        case 's': a.slave          = optarg;            break;
        case 'S': a.speed          = std::stoi(optarg); break;
        case 'm': a.mode           = std::stoi(optarg); break;
        case 'd': a.duration_s     = std::stod(optarg); break;
        case 'f': a.fault_reset_s  = std::stod(optarg); break;
        case 'p': a.print_hz       = std::stod(optarg); break;
        case 'w': a.write_startup  = true;              break;
        case 'F': a.force_sdo_mode = true;              break;
        case 'g': a.debug_gain_sdos = true;             break;
        case 'r': a.rt_priority    = std::stoi(optarg); break;
        case 'c': a.cpu_affinity   = std::stoi(optarg); break;
        case 'h': printUsage(argv[0]); std::exit(0);
        default:  printUsage(argv[0]); std::exit(2);
        }
    }
    return a;
}

// ── SDO helpers (direct SOEM calls) ──────────────────────────────────────────

static void writeSdoF32(int soem_idx, uint16_t index, uint8_t subindex, float value) {
    for (int attempt = 0; attempt < 5; ++attempt) {
        int ret = ec_SDOwrite(
            static_cast<uint16>(soem_idx), index, subindex,
            FALSE, static_cast<int>(sizeof(value)), &value, EC_TIMEOUTRET
        );
        if (ret > 0) return;
        if (attempt < 4) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::fprintf(stderr, "WARNING: SDO write failed at 0x%04X:%02X\n", index, subindex);
}

static void writeSdoI8(int soem_idx, uint16_t index, uint8_t subindex, int8_t value) {
    for (int attempt = 0; attempt < 5; ++attempt) {
        int ret = ec_SDOwrite(
            static_cast<uint16>(soem_idx), index, subindex,
            FALSE, 1, &value, EC_TIMEOUTRET
        );
        if (ret > 0) return;
        if (attempt < 4) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::fprintf(stderr, "WARNING: SDO write failed at 0x%04X:%02X\n", index, subindex);
}

// ── Debug gain SDO dump ───────────────────────────────────────────────────────

static void debugGainRegisters(int soem_idx) {
    struct { const char* name; uint16_t index; } regs[] = {
        {"velocity_loop_kp", 0x250A},
        {"velocity_loop_ki", 0x250B},
    };
    for (auto& r : regs) {
        uint8_t buf[8] = {};
        int sz = sizeof(buf);
        int ret = ec_SDOread(static_cast<uint16>(soem_idx), r.index, 0x00, FALSE, &sz, buf, EC_TIMEOUTRET);
        if (ret <= 0) {
            std::printf("%s 0x%04X:00 sdo_read failed\n", r.name, r.index);
            continue;
        }
        float    f32 = 0.0f; std::memcpy(&f32, buf, 4);
        uint32_t u32 = 0;    std::memcpy(&u32, buf, 4);
        int32_t  s32 = 0;    std::memcpy(&s32, buf, 4);
        std::printf("%s 0x%04X:00  f32=%.6f  u32=%u  s32=%d\n",
                    r.name, r.index, static_cast<double>(f32), u32, s32);
    }
}

// ── Write startup gains back via SDO ─────────────────────────────────────────

static void applyStartupParams(
    const std::unordered_map<std::string, float>& params,
    int soem_idx)
{
    // Index map matching NovantaEverestAdapter::startupReadSpecs().
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

// ── AL state name ─────────────────────────────────────────────────────────────

static const char* alStateName(uint8_t code) {
    const uint8_t base  = code & 0x0Fu;
    const bool    error = (code & 0x10u) != 0;
    const char*   name;
    switch (base) {
    case 0x01: name = "INIT";             break;
    case 0x02: name = "PRE_OP";          break;
    case 0x03: name = "BOOTSTRAP";       break;
    case 0x04: name = "SAFE_OP";         break;
    case 0x08: name = "OP";              break;
    case 0x00: name = "UNKNOWN";         break;
    default:   name = "UNKNOWN";         break;
    }
    return error ? "ERROR" : name;
}

// ── Cia402State name ──────────────────────────────────────────────────────────

static const char* cia402Name(Cia402State s) {
    switch (s) {
    case Cia402State::NOT_READY_TO_SWITCH_ON: return "NOT_READY_TO_SWITCH_ON";
    case Cia402State::SWITCH_ON_DISABLED:     return "SWITCH_ON_DISABLED";
    case Cia402State::READY_TO_SWITCH_ON:     return "READY_TO_SWITCH_ON";
    case Cia402State::SWITCHED_ON:            return "SWITCHED_ON";
    case Cia402State::OPERATION_ENABLED:      return "OPERATION_ENABLED";
    case Cia402State::QUICK_STOP_ACTIVE:      return "QUICK_STOP_ACTIVE";
    case Cia402State::FAULT_REACTION_ACTIVE:  return "FAULT_REACTION_ACTIVE";
    case Cia402State::FAULT:                  return "FAULT";
    }
    return "UNKNOWN";
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);

    // Validate mode value.
    const auto cmd_mode = static_cast<ModeOfOperation>(args.mode);
    switch (cmd_mode) {
    case ModeOfOperation::NO_MODE:
    case ModeOfOperation::PROFILE_POSITION:
    case ModeOfOperation::PROFILE_VELOCITY:
    case ModeOfOperation::PROFILE_TORQUE:
    case ModeOfOperation::CYCLIC_SYNC_POSITION:
    case ModeOfOperation::CYCLIC_SYNC_VELOCITY:
    case ModeOfOperation::CYCLIC_SYNC_TORQUE:
        break;
    default:
        std::fprintf(stderr, "Unsupported --mode value %d.\n", args.mode);
        return 2;
    }

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

    // Verify the requested slave exists.
    if (rt->adapters.find(args.slave) == rt->adapters.end()) {
        std::fprintf(stderr, "Unknown slave '%s'. Available:", args.slave.c_str());
        for (auto& [k, _] : rt->adapters) std::fprintf(stderr, " %s", k.c_str());
        std::fprintf(stderr, "\n");
        master.close();
        return 1;
    }

    const int soem_idx = rt->slave_index.at(args.slave);

    // Extract startup params (all as float).
    float torque_kp = 0.0f, vel_qr = 0.0f, vel_is = 0.0f;
    float vel_kp = 0.0f, vel_ki = 0.0f, vel_kd = 0.0f;
    float pos_kp = 0.0f, pos_ki = 0.0f, pos_kd = 0.0f;

    auto it_params = rt->startup_params.find(args.slave);
    if (it_params != rt->startup_params.end()) {
        const auto& p = it_params->second;
        auto get = [&](const char* k) -> float {
            auto it = p.find(k); return it != p.end() ? it->second : 0.0f;
        };
        const float kt = get("motor_kt");
        torque_kp = (std::abs(kt) > 1e-9f) ? (1.0f / kt) : 0.0f;
        vel_qr    = get("torque_loop_max_output");
        vel_is    = get("torque_loop_min_output");
        vel_kp    = get("velocity_loop_kp");
        vel_ki    = get("velocity_loop_ki");
        vel_kd    = get("velocity_loop_kd");
        pos_kp    = get("position_loop_kp");
        pos_ki    = get("position_loop_ki");
        pos_kd    = get("position_loop_kd");

        std::printf("Loaded startup gains:");
        for (auto& [k, v] : p) std::printf("  %s=%.6f", k.c_str(), static_cast<double>(v));
        std::printf("\n");
    }

    if (args.debug_gain_sdos) debugGainRegisters(soem_idx);

    if (args.write_startup && it_params != rt->startup_params.end()) {
        applyStartupParams(it_params->second, soem_idx);
        std::printf("Wrote startup gains back via SDO.\n");
    }

    if (args.force_sdo_mode) {
        writeSdoI8(soem_idx, 0x6060, 0x00, static_cast<int8_t>(args.mode));
        std::printf("Forced SDO mode write: 0x6060=%d\n", args.mode);
    }

    std::printf("Using '%s' at position %d\n", args.slave.c_str(), soem_idx - 1);

    // Build RT config.
    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority = std::clamp(args.rt_priority, 0, 99);
    if (args.cpu_affinity >= 0) rt_cfg.cpu_affinity.insert(args.cpu_affinity);

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);
    loop.start();

    const auto t0           = std::chrono::steady_clock::now();
    const auto deadline     = t0 + std::chrono::duration<double>(std::max(0.0, args.duration_s));
    const auto reset_end    = t0 + std::chrono::duration<double>(std::max(0.0, args.fault_reset_s));
    const double print_period = 1.0 / std::max(args.print_hz, 0.1);
    auto next_print         = t0;
    const int32_t speed_cmd = args.speed;

    while (!g_shutdown.load() && std::chrono::steady_clock::now() < deadline) {
        const auto now     = std::chrono::steady_clock::now();
        const bool in_reset = now < reset_end;

        Command cmd;
        cmd.mode_of_operation      = cmd_mode;
        cmd.target_velocity_mrevs  = static_cast<float>(speed_cmd);
        cmd.torque_kp              = torque_kp;
        cmd.torque_loop_max_output = vel_qr;
        cmd.torque_loop_min_output = vel_is;
        cmd.velocity_loop_kp       = vel_kp;
        cmd.velocity_loop_ki       = vel_ki;
        cmd.velocity_loop_kd       = vel_kd;
        cmd.position_loop_kp       = pos_kp;
        cmd.position_loop_ki       = pos_ki;
        cmd.position_loop_kd       = pos_kd;
        cmd.enable_drive           = !in_reset;
        cmd.clear_fault            = in_reset;

        SystemCommand sys_cmd;
        sys_cmd.by_slave[args.slave] = cmd;
        loop.setCommand(sys_cmd);

        if (now >= next_print) {
            const SystemStatus status = loop.getStatus();
            const LoopStats    stats  = loop.stats();

            auto slave_it = status.by_slave.find(args.slave);
            if (slave_it == status.by_slave.end() || !slave_it->second.has_value()) {
                std::printf("cycle=%lu wkc=%d cmd_60FF=%d speed_606C=unavailable\n",
                            static_cast<unsigned long>(stats.cycle_count),
                            stats.last_wkc, speed_cmd);
            } else {
                const auto& ds = std::any_cast<const DriveStatus&>(slave_it->second);
                std::printf(
                    "cycle=%lu wkc=%d "
                    "torque_max_out=%.6f torque_min_out=%.6f "
                    "vel_kp=%.6f vel_ki=%.6f torque_kp=%.6f "
                    "al=%s state=%s "
                    "cmd_6060=%d mode_6061=%d "
                    "cmd_60FF=%d speed_606C=%d "
                    "rx_cmd_2079=%.3f bus_v_2060=%.3f "
                    "status=0x%04X err=0x%04X\n",
                    static_cast<unsigned long>(stats.cycle_count),
                    stats.last_wkc,
                    static_cast<double>(vel_qr),
                    static_cast<double>(vel_is),
                    static_cast<double>(vel_kp),
                    static_cast<double>(vel_ki),
                    static_cast<double>(torque_kp),
                    alStateName(ds.al_state_code),
                    cia402Name(ds.cia402_state),
                    args.mode,
                    static_cast<int>(ds.mode_of_operation_display),
                    speed_cmd,
                    ds.measured_input_side_velocity_raw,
                    static_cast<double>(ds.velocity_command_received),
                    static_cast<double>(ds.bus_voltage),
                    static_cast<unsigned>(ds.status_word),
                    static_cast<unsigned>(ds.error_code)
                );
            }
            next_print += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(print_period));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    loop.stop();
    master.close();
    return 0;
}
