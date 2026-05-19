// Read and print Beckhoff ELM3002 TxPDO data in the cyclic loop.
//
// Usage:
//   sudo ./elm3002_data_read [options]
//
// Options:
//   --topology <path>    Topology JSON file (default: config/topology.dyno2.template6.json)
//   --slave    <name>    Configured ELM3002 slave name (default: analog_input_interface)
//   --duration <s>       Monitor duration in seconds (default: 60)
//   --print-hz <hz>      Terminal update rate (default: 5)
//   --rt-priority <1-99> Loop thread SCHED_FIFO priority (0 = default scheduler)
//   --cpu-affinity <cpu> Comma-separated CPU indices to pin loop thread to (e.g. 2 or 2,3)

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/loop.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/default_adapter_factory.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/adapter.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/data_types.hpp"

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
using namespace ethercat_core::beckhoff::elm3002;

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{false};
static void onSignal(int) { g_shutdown.store(true); }

// ── Argument defaults ─────────────────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY  = "config/ethercat_device_config/topology.dyno2.template6.json";
static constexpr const char* DEFAULT_SLAVE     = "analog_input_interface";
static constexpr double      DEFAULT_DURATION  = 60.0;
static constexpr double      DEFAULT_PRINT_HZ  = 5.0;

struct Args {
    std::string topology     = DEFAULT_TOPOLOGY;
    std::string slave        = DEFAULT_SLAVE;
    double      duration_s   = DEFAULT_DURATION;
    double      print_hz     = DEFAULT_PRINT_HZ;
    int         rt_priority  = 0;
    std::set<int> cpu_affinity;
};

static void printUsage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  --topology <path>      Topology JSON       (default: %s)\n"
        "  --slave    <name>      ELM3002 slave name  (default: %s)\n"
        "  --duration <s>         Monitor duration    (default: %.1f s)\n"
        "  --print-hz <hz>        Print rate          (default: %.1f Hz)\n"
        "  --rt-priority <1-99>   SCHED_FIFO priority (0 = default)\n"
        "  --cpu-affinity <cpu>   CPU index(es), comma-separated (e.g. 2 or 2,3)\n",
        prog,
        DEFAULT_TOPOLOGY, DEFAULT_SLAVE, DEFAULT_DURATION, DEFAULT_PRINT_HZ
    );
}

// Parse "2" or "2,3" into a set of CPU indices.
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
        {"duration",     required_argument, nullptr, 'd'},
        {"print-hz",     required_argument, nullptr, 'p'},
        {"rt-priority",  required_argument, nullptr, 'r'},
        {"cpu-affinity", required_argument, nullptr, 'c'},
        {"help",         no_argument,       nullptr, 'h'},
        {nullptr,        0,                 nullptr,  0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "t:s:d:p:r:c:h", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 't': a.topology    = optarg;            break;
        case 's': a.slave       = optarg;            break;
        case 'd': a.duration_s  = std::stod(optarg); break;
        case 'p': a.print_hz    = std::stod(optarg); break;
        case 'r': a.rt_priority = std::stoi(optarg); break;
        case 'c': a.cpu_affinity = parseCpuAffinity(optarg); break;
        case 'h': printUsage(argv[0]); std::exit(0);
        default:  printUsage(argv[0]); std::exit(2);
        }
    }
    return a;
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

    EthercatMaster master(cfg, ethercat_core::makeDefaultAdapterFactory());
    MasterRuntime* rt = nullptr;

    try {
        rt = &master.initialize();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Master init failed: %s\n", e.what());
        return 1;
    }

    // Verify slave exists and is an ELM3002.
    auto adapter_it = rt->adapters.find(args.slave);
    if (adapter_it == rt->adapters.end()) {
        std::fprintf(stderr, "Unknown slave '%s'. Available:", args.slave.c_str());
        for (auto& [k, _] : rt->adapters) std::fprintf(stderr, " %s", k.c_str());
        std::fprintf(stderr, "\n");
        master.close();
        return 1;
    }

    auto* adapter = dynamic_cast<Elm3002Adapter*>(adapter_it->second.get());
    if (!adapter) {
        std::fprintf(stderr, "Slave '%s' is not an ELM3002 adapter.\n", args.slave.c_str());
        master.close();
        return 1;
    }

    const int soem_idx = rt->slave_index.at(args.slave);

    // Build RT config.
    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority = std::clamp(args.rt_priority, 0, 99);
    rt_cfg.cpu_affinity = args.cpu_affinity;

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);
    loop.start();

    std::printf(
        "Monitoring '%s' at position %d for %.1fs  |  "
        "rt_priority=%d cpu_affinity=",
        args.slave.c_str(), soem_idx - 1, args.duration_s,
        std::clamp(args.rt_priority, 0, 99)
    );
    if (args.cpu_affinity.empty()) {
        std::printf("none\n");
    } else {
        bool first = true;
        for (int c : args.cpu_affinity) { std::printf("%s%d", first ? "" : ",", c); first = false; }
        std::printf("\n");
    }

    const auto   t0           = std::chrono::steady_clock::now();
    const auto   deadline     = t0 + std::chrono::duration<double>(std::max(0.0, args.duration_s));
    const double print_period = 1.0 / std::max(args.print_hz, 0.1);
    auto         next_print   = t0;

    while (!g_shutdown.load() && std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();

        if (now >= next_print) {
            const SystemStatus status = loop.getStatus();
            const LoopStats    stats  = loop.stats();
            const std::string  al     = alStateName(static_cast<int>(ec_slave[soem_idx].state));
            const double       cycle_us = static_cast<double>(stats.last_cycle_time_ns) / 1000.0;

            auto slave_it = status.by_slave.find(args.slave);
            if (slave_it == status.by_slave.end() || !slave_it->second.has_value()) {
                std::printf(
                    "al=%s cycle_us=%.1f "
                    "pai_status_1=unavailable pai_samples_1=unavailable "
                    "pai_status_2=unavailable pai_samples_2=unavailable\n",
                    al.c_str(), cycle_us
                );
            } else {
                const auto& d = std::any_cast<const Data&>(slave_it->second);
                const float ch1_voltage = Elm3002Adapter::scaleAdcToVoltage(d.pai_samples_1);
                const float ch2_voltage = Elm3002Adapter::scaleAdcToVoltage(d.pai_samples_2);
                const float ch1_torque  = adapter->scaledTorqueCh1(d);
                const float ch2_torque  = adapter->scaledTorqueCh2(d);
                std::printf(
                    "al=%s cycle_us=%.1f "
                    "ch1_voltage=%.4fV ch1_torque=%.4f "
                    "ch2_voltage=%.4fV ch2_torque=%.4f\n",
                    al.c_str(), cycle_us,
                    static_cast<double>(ch1_voltage),
                    static_cast<double>(ch1_torque),
                    static_cast<double>(ch2_voltage),
                    static_cast<double>(ch2_torque)
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
