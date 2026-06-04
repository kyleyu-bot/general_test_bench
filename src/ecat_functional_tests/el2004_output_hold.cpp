// Hold EL2004 output channel 1 high, then clear it on timeout or Ctrl-C.
//
// Usage:
//   sudo ./el2004_output_hold [options]
//
// Options:
//   --topology <path>    Topology JSON file (default: config/topology.dyno2.template6.json)
//   --slave    <name>    Configured EL2004 slave name (default: digital_IO)
//   --hold-s   <s>       Seconds to hold output 1 high (default: 60)
//   --print-hz <hz>      Status print rate while holding (default: 2)
//   --rt-priority <1-99> Loop thread SCHED_FIFO priority (0 = default scheduler)
//   --cpu-affinity <cpu> Comma-separated CPU indices (e.g. 2 or 2,3)

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/loop.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/default_adapter_factory.hpp"
#include "ethercat_core/devices/beckhoff/el2004/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el2004/data_types.hpp"

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
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

using namespace ethercat_core;
using namespace ethercat_core::beckhoff::el2004;

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{false};
static void onSignal(int) { g_shutdown.store(true); }

// ── Argument defaults ─────────────────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY = "config/ethercat_device_config/topology.dyno2.template7.json";
static constexpr const char* DEFAULT_SLAVE    = "digital_IO";
static constexpr double      DEFAULT_HOLD_S   = 60.0;
static constexpr double      DEFAULT_PRINT_HZ = 2.0;

struct Args {
    std::string   topology     = DEFAULT_TOPOLOGY;
    std::string   slave        = DEFAULT_SLAVE;
    double        hold_s       = DEFAULT_HOLD_S;
    double        print_hz     = DEFAULT_PRINT_HZ;
    int           rt_priority  = 0;
    std::set<int> cpu_affinity;
};

static void printUsage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  --topology <path>      Topology JSON       (default: %s)\n"
        "  --slave    <name>      EL2004 slave name   (default: %s)\n"
        "  --hold-s   <s>         Hold duration       (default: %.1f s)\n"
        "  --print-hz <hz>        Print rate          (default: %.1f Hz)\n"
        "  --rt-priority <1-99>   SCHED_FIFO priority (0 = default)\n"
        "  --cpu-affinity <cpu>   CPU index(es), comma-separated (e.g. 2 or 2,3)\n",
        prog,
        DEFAULT_TOPOLOGY, DEFAULT_SLAVE, DEFAULT_HOLD_S, DEFAULT_PRINT_HZ
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
    if (cpus.empty()) throw std::invalid_argument("cpu-affinity must include at least one CPU");
    return cpus;
}

static Args parseArgs(int argc, char** argv) {
    Args a;
    static struct option long_opts[] = {
        {"topology",     required_argument, nullptr, 't'},
        {"slave",        required_argument, nullptr, 's'},
        {"hold-s",       required_argument, nullptr, 'H'},
        {"print-hz",     required_argument, nullptr, 'p'},
        {"rt-priority",  required_argument, nullptr, 'r'},
        {"cpu-affinity", required_argument, nullptr, 'c'},
        {"help",         no_argument,       nullptr, 'h'},
        {nullptr,        0,                 nullptr,  0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "t:s:H:p:r:c:h", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 't': a.topology     = optarg;                    break;
        case 's': a.slave        = optarg;                    break;
        case 'H': a.hold_s       = std::stod(optarg);         break;
        case 'p': a.print_hz     = std::stod(optarg);         break;
        case 'r': a.rt_priority  = std::stoi(optarg);         break;
        case 'c': a.cpu_affinity = parseCpuAffinity(optarg);  break;
        case 'h': printUsage(argv[0]); std::exit(0);
        default:  printUsage(argv[0]); std::exit(2);
        }
    }
    return a;
}

// ── Helper: send a command and wait for a few cycles to flush ─────────────────

static void sendAndFlush(EthercatLoop& loop, const std::string& slave,
                         const Command& cmd, int cycle_hz) {
    SystemCommand sys;
    sys.by_slave[slave] = cmd;
    loop.setCommand(sys);
    // Wait for at least 2 cycles to ensure the PDO is sent.
    const int wait_ms = std::max(2, 2000 / std::max(cycle_hz, 1));
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);

    MasterConfig cfg;
    try {
        cfg = loadTopology(args.topology);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load topology '%s': %s\n", args.topology.c_str(), e.what());
        return 1;
    }

    // Keep only the target EL2004 slave — skip drive PDO mapping entirely.
    cfg.slaves.erase(
        std::remove_if(cfg.slaves.begin(), cfg.slaves.end(),
            [&args](const SlaveConfig& sc) { return sc.name != args.slave; }),
        cfg.slaves.end()
    );

    EthercatMaster master(cfg, ethercat_core::makeDefaultAdapterFactory());
    MasterRuntime* rt = nullptr;

    try {
        rt = &master.initialize();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Master init failed: %s\n", e.what());
        return 1;
    }

    if (rt->adapters.find(args.slave) == rt->adapters.end()) {
        std::fprintf(stderr, "Unknown slave '%s'. Available:", args.slave.c_str());
        for (auto& [k, _] : rt->adapters) std::fprintf(stderr, " %s", k.c_str());
        std::fprintf(stderr, "\n");
        master.close();
        return 1;
    }

    const int soem_idx = rt->slave_index.at(args.slave);

    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority  = std::clamp(args.rt_priority, 0, 99);
    rt_cfg.cpu_affinity = args.cpu_affinity;

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    // Diagnostic: confirm SOEM mapped an output buffer for the EL2004.
    // Note: SOEM sets Obytes=0 when Obits < 8 (EL2004 has 4 output bits), so use Obits.
    {
        const int   obits = static_cast<int>(ec_slave[soem_idx].Obits);
        const int   obytes_effective = (obits + 7) / 8;
        const void* optr  = ec_slave[soem_idx].outputs;
        std::printf("EL2004 '%s': soem_idx=%d Obits=%d effective_bytes=%d outputs_ptr=%s state=0x%02X\n",
            args.slave.c_str(), soem_idx, obits, obytes_effective,
            optr ? "OK" : "NULL",
            static_cast<unsigned>(ec_slave[soem_idx].state));
        if (obytes_effective == 0 || optr == nullptr) {
            std::fprintf(stderr, "ERROR: SOEM output buffer not mapped for '%s'. "
                "PDO may not have been configured.\n", args.slave.c_str());
            master.close();
            return 1;
        }
    }

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);
    loop.start();

    // Set output 1 high.
    sendAndFlush(loop, args.slave, Command{.output_1 = true}, cfg.cycle_hz);
    std::printf(
        "Set EL2004 output 1 high "
        "(PDO object 1 / 0x7010:01 in the configured mapping).\n"
    );

    const auto   t0           = std::chrono::steady_clock::now();
    const auto   deadline     = t0 + std::chrono::duration<double>(std::max(0.0, args.hold_s));
    const double print_period = 1.0 / std::max(args.print_hz, 0.1);
    auto         next_print   = t0;
    const char*  cleared_by   = "timeout";

    while (!g_shutdown.load() && std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();

        if (now >= next_print) {
            const SystemStatus status   = loop.getStatus();
            const double remaining_s    = std::chrono::duration<double>(deadline - now).count();

            uint8_t output_byte = 0;
            auto slave_it = status.by_slave.find(args.slave);
            if (slave_it != status.by_slave.end() && slave_it->second.has_value()) {
                output_byte = std::any_cast<const Status&>(slave_it->second).output_byte;
            }

            std::printf("holding output_1=1 output_byte=0x%02X remaining_s=%.1f\n",
                output_byte, remaining_s);

            next_print += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(print_period));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (g_shutdown.load()) cleared_by = "signal";

    // Clear output 1.
    sendAndFlush(loop, args.slave, Command{}, cfg.cycle_hz);

    const SystemStatus final_status = loop.getStatus();
    uint8_t output_byte = 0;
    auto slave_it = final_status.by_slave.find(args.slave);
    if (slave_it != final_status.by_slave.end() && slave_it->second.has_value()) {
        output_byte = std::any_cast<const Status&>(slave_it->second).output_byte;
    }
    std::printf("Cleared EL2004 output 1 via %s. output_byte=0x%02X\n",
        cleared_by, output_byte);

    loop.stop();
    master.close();
    return 0;
}
