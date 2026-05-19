// Print ELM3002 TxPDO datagram sections in hex for protocol inspection.
//
// Section choices:
//   0  Full datagram (all PDO bytes, 24 bytes)
//   1  0x1A00  PAI Status ch1   (pai_status_1,   4 bytes)
//   2  0x1A01  PAI Samples ch1  (pai_samples_1,  4 bytes)
//   3  0x1A10  Timestamp        (timestamp,       8 bytes)
//   4  0x1A21  PAI Status ch2   (pai_status_2,   4 bytes)
//   5  0x1A22  PAI Samples ch2  (pai_samples_2,  4 bytes)
//
// Usage:
//   sudo ./elm3002_pdo_check <section> [options]
//
// Options:
//   --topology <path>    Topology JSON file (default: config/topology.dyno2.template6.json)
//   --slave    <name>    Configured ELM3002 slave name (default: analog_input_interface)
//   --duration <s>       Monitor duration in seconds (default: 60)
//   --print-hz <hz>      Terminal update rate (default: 5)
//   --rt-priority <1-99> Loop thread SCHED_FIFO priority (0 = default scheduler)
//   --cpu-affinity <cpu> Comma-separated CPU indices (e.g. 2 or 2,3)

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

// ── Section metadata ──────────────────────────────────────────────────────────

struct SectionMeta {
    const char* field_name;
    const char* label;
    int         offset;   // byte offset in raw PDO
    int         size;     // byte count
};

// Must match elm3002/data_types.hpp layout (TX_PDO_SIZE = 24 bytes):
//   offset 0  : pai_status_1   uint32  4 bytes
//   offset 4  : pai_samples_1  int32   4 bytes
//   offset 8  : timestamp      uint64  8 bytes
//   offset 16 : pai_status_2   uint32  4 bytes
//   offset 20 : pai_samples_2  int32   4 bytes
static constexpr SectionMeta SECTIONS[] = {
    // 0: full datagram
    {"full",          "full datagram",          0,  TX_PDO_SIZE},
    // 1: 0x1A00 PAI Status ch1
    {"pai_status_1",  "0x1A00 PAI Status ch1",  0,  4},
    // 2: 0x1A01 PAI Samples ch1
    {"pai_samples_1", "0x1A01 PAI Samples ch1", 4,  4},
    // 3: 0x1A10 Timestamp
    {"timestamp",     "0x1A10 Timestamp",        8,  8},
    // 4: 0x1A21 PAI Status ch2
    {"pai_status_2",  "0x1A21 PAI Status ch2",  16, 4},
    // 5: 0x1A22 PAI Samples ch2
    {"pai_samples_2", "0x1A22 PAI Samples ch2", 20, 4},
};
static constexpr int NUM_SECTIONS = static_cast<int>(sizeof(SECTIONS) / sizeof(SECTIONS[0]));

// ── Argument defaults ─────────────────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY = "config/ethercat_device_config/topology.dyno2.template6.json";
static constexpr const char* DEFAULT_SLAVE    = "analog_input_interface";
static constexpr double      DEFAULT_DURATION = 60.0;
static constexpr double      DEFAULT_PRINT_HZ = 5.0;

struct Args {
    int         section      = 0;
    std::string topology     = DEFAULT_TOPOLOGY;
    std::string slave        = DEFAULT_SLAVE;
    double      duration_s   = DEFAULT_DURATION;
    double      print_hz     = DEFAULT_PRINT_HZ;
    int         rt_priority  = 0;
    std::set<int> cpu_affinity;
};

static void printUsage(const char* prog) {
    std::printf(
        "Usage: %s <section> [options]\n"
        "\n"
        "Section:\n",
        prog
    );
    for (int i = 0; i < NUM_SECTIONS; ++i)
        std::printf("  %d  %s\n", i, SECTIONS[i].label);
    std::printf(
        "\nOptions:\n"
        "  --topology <path>      Topology JSON       (default: %s)\n"
        "  --slave    <name>      ELM3002 slave name  (default: %s)\n"
        "  --duration <s>         Monitor duration    (default: %.1f s)\n"
        "  --print-hz <hz>        Print rate          (default: %.1f Hz)\n"
        "  --rt-priority <1-99>   SCHED_FIFO priority (0 = default)\n"
        "  --cpu-affinity <cpu>   CPU index(es), comma-separated (e.g. 2 or 2,3)\n",
        DEFAULT_TOPOLOGY, DEFAULT_SLAVE, DEFAULT_DURATION, DEFAULT_PRINT_HZ
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
    // First positional argument is section number.
    if (argc < 2 || argv[1][0] == '-') {
        printUsage(argv[0]);
        std::exit(2);
    }
    Args a;
    a.section = std::stoi(argv[1]);
    if (a.section < 0 || a.section >= NUM_SECTIONS) {
        std::fprintf(stderr, "error: section must be 0–%d\n", NUM_SECTIONS - 1);
        std::exit(2);
    }

    // Shift so getopt_long sees remaining args.
    argc -= 1;
    argv += 1;

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
        case 't': a.topology     = optarg;                    break;
        case 's': a.slave        = optarg;                    break;
        case 'd': a.duration_s   = std::stod(optarg);         break;
        case 'p': a.print_hz     = std::stod(optarg);         break;
        case 'r': a.rt_priority  = std::stoi(optarg);         break;
        case 'c': a.cpu_affinity = parseCpuAffinity(optarg);  break;
        case 'h': printUsage(argv[0]); std::exit(0);
        default:  printUsage(argv[0]); std::exit(2);
        }
    }
    return a;
}

// ── Hex dump helper ───────────────────────────────────────────────────────────

// Prints: "<label padded to 30 chars> [NN bytes]  XX XX XX ..."
static void printHexDump(const char* label, const uint8_t* data, int size) {
    std::printf("%-30s [%2d bytes]  ", label, size);
    for (int i = 0; i < size; ++i) std::printf("%02X%s", data[i], (i + 1 < size) ? " " : "");
    std::printf("  ");
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);
    const SectionMeta& sec = SECTIONS[args.section];

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

    if (!dynamic_cast<Elm3002Adapter*>(adapter_it->second.get())) {
        std::fprintf(stderr, "Slave '%s' is not an ELM3002 adapter.\n", args.slave.c_str());
        master.close();
        return 1;
    }

    const int soem_idx = rt->slave_index.at(args.slave);

    // Build RT config.
    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority  = std::clamp(args.rt_priority, 0, 99);
    rt_cfg.cpu_affinity = args.cpu_affinity;

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);
    loop.start();

    // Wait for first valid PDO cycle.
    std::printf("Waiting for first valid PDO cycle...\n");
    while (!g_shutdown.load() && loop.getStatus().stamp_ns == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    const int64_t stale_threshold_ns = static_cast<int64_t>(3LL * 1'000'000'000LL / cfg.cycle_hz);

    std::printf("Monitoring '%s' at position %d for %.1fs  |  section: %s  |  rt_priority=%d cpu_affinity=",
        args.slave.c_str(), soem_idx - 1, args.duration_s, sec.label,
        std::clamp(args.rt_priority, 0, 99));
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

            const std::string pdo_ping = [&] {
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                    "wkc=%d cycles=%lu exec_us=%.1f period_us=%.1f wake_us=%.1f al=%s",
                    stats.last_wkc,
                    static_cast<unsigned long>(stats.cycle_count),
                    static_cast<double>(stats.last_cycle_time_ns)    / 1000.0,
                    static_cast<double>(stats.last_period_ns)        / 1000.0,
                    static_cast<double>(stats.last_wakeup_latency_ns)/ 1000.0,
                    al.c_str()
                );
                return std::string(buf);
            }();

            // Check data age.
            using namespace std::chrono;
            const int64_t now_ns  = duration_cast<nanoseconds>(
                steady_clock::now().time_since_epoch()).count();
            const int64_t age_ns  = now_ns - status.stamp_ns;

            auto slave_it = status.by_slave.find(args.slave);
            const bool have_data  = (slave_it != status.by_slave.end())
                                 && slave_it->second.has_value();

            if (age_ns > stale_threshold_ns) {
                std::printf("[STALE %.1fms] %s\n",
                    static_cast<double>(age_ns) / 1e6, pdo_ping.c_str());
            } else if (!have_data) {
                std::printf("pdo=unavailable  %s\n", pdo_ping.c_str());
            } else {
                const auto& d = std::any_cast<const Data&>(slave_it->second);
                if (d.raw_pdo.empty()) {
                    std::printf("pdo=unavailable  %s\n", pdo_ping.c_str());
                } else {
                    const uint8_t* base = d.raw_pdo.data();
                    const int      total = static_cast<int>(d.raw_pdo.size());
                    const int      off   = sec.offset;
                    const int      sz    = std::min(sec.size, total - off);
                    if (sz > 0)
                        printHexDump(sec.label, base + off, sz);
                    else
                        std::printf("%-30s [out of range]  ", sec.label);
                    std::printf("%s\n", pdo_ping.c_str());
                }
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
