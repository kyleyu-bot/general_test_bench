// Scan EtherCAT slaves using SOEM and print identity information.
//
// Usage:
//   sudo ./scan_soem [--iface <ifname>]
//
// Requires CAP_NET_RAW (raw socket) — run with sudo or set the capability:
//   sudo setcap cap_net_raw+ep ./scan_soem

#include <getopt.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

extern "C" {
#include "ethercat.h"
}

static constexpr const char* DEFAULT_IFACE = "ecat0";

static std::string sysfsRead(const std::string& iface, const char* leaf) {
    std::ifstream f("/sys/class/net/" + iface + "/" + leaf);
    if (!f.is_open()) return "unknown";
    std::string val;
    std::getline(f, val);
    return val;
}

static bool ifaceExists(const std::string& iface) {
    std::ifstream f("/sys/class/net/" + iface);
    return f.is_open() || (access(("/sys/class/net/" + iface).c_str(), F_OK) == 0);
}

static void printAvailableIfaces() {
    std::printf("Available interfaces:");
    // List /sys/class/net entries.
    FILE* ls = popen("ls /sys/class/net", "r");
    if (!ls) return;
    char buf[64];
    bool first = true;
    while (fgets(buf, sizeof(buf), ls)) {
        // Strip newline.
        buf[strcspn(buf, "\n")] = '\0';
        std::printf("%s %s", first ? "" : ",", buf);
        first = false;
    }
    pclose(ls);
    std::printf("\n");
}

static void printUsage(const char* prog) {
    std::printf("Usage: %s [--iface <ifname>]\n", prog);
    std::printf("  --iface   Network interface to scan (default: %s)\n", DEFAULT_IFACE);
}

int main(int argc, char** argv) {
    std::string iface = DEFAULT_IFACE;

    static struct option long_opts[] = {
        {"iface", required_argument, nullptr, 'i'},
        {"help",  no_argument,       nullptr, 'h'},
        {nullptr, 0,                 nullptr,  0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "i:h", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 'i': iface = optarg; break;
        case 'h': printUsage(argv[0]); return 0;
        default:  printUsage(argv[0]); return 2;
        }
    }

    // Check the interface exists in sysfs before trying to open a raw socket.
    if (!ifaceExists(iface)) {
        std::printf("Interface '%s' does not exist.\n", iface.c_str());
        printAvailableIfaces();
        return 2;
    }

    const std::string operstate = sysfsRead(iface, "operstate");
    std::printf("Opening interface '%s' (operstate=%s)\n",
                iface.c_str(), operstate.c_str());

    if (ec_init(iface.c_str()) <= 0) {
        std::printf("Failed to open interface '%s'.\n", iface.c_str());
        std::printf("If the interface is correct, run with elevated privileges:\n");
        std::printf("  sudo %s --iface %s\n", argv[0], iface.c_str());
        return 1;
    }

    const int slave_count = ec_config_init(FALSE);

    if (slave_count <= 0) {
        std::printf("No EtherCAT slaves found.\n");
        ec_close();
        return 0;
    }

    std::printf("Found %d slave%s:\n\n", slave_count, slave_count == 1 ? "" : "s");
    std::printf("  %-4s  %-10s  %-10s  %-10s  %-6s  %s\n",
                "Pos", "Vendor", "ProductID", "Revision", "Alias", "Name");
    std::printf("  %-4s  %-10s  %-10s  %-10s  %-6s  %s\n",
                "---", "------", "---------", "--------", "-----", "----");

    for (int i = 1; i <= slave_count; ++i) {
        const ec_slavet& sl = ec_slave[i];
        std::printf("  %-4d  0x%08X  0x%08X  0x%08X  %-6d  %s\n",
                    i - 1,                         // 0-based position (matches topology config)
                    static_cast<unsigned>(sl.eep_man),
                    static_cast<unsigned>(sl.eep_id),
                    static_cast<unsigned>(sl.eep_rev),
                    static_cast<int>(sl.aliasadr),
                    sl.name);
    }

    std::printf("\n");
    ec_close();
    return 0;
}
