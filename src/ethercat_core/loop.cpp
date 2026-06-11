#include "ethercat_core/loop.hpp"

// SOEM process data exchange.
extern "C" {
#include "ethercat.h"
}

// POSIX real-time
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <chrono>
#include <cstring>
#include <stdexcept>

namespace ethercat_core {

// ── Monotonic nanosecond clock helpers ────────────────────────────────────────

static int64_t monotonicNs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// Absolute sleep until abs_ns on CLOCK_MONOTONIC.
static void sleepUntilNs(int64_t abs_ns) {
    struct timespec ts;
    ts.tv_sec  = abs_ns / 1'000'000'000LL;
    ts.tv_nsec = abs_ns % 1'000'000'000LL;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

// ── EthercatLoop ──────────────────────────────────────────────────────────────

EthercatLoop::EthercatLoop(
    MasterRuntime& runtime,
    int            cycle_hz,
    LoopRtConfig   rt_config)
    : runtime_(runtime)
    , cycle_ns_(1'000'000'000LL / cycle_hz)
    , rt_config_(std::move(rt_config))
{
    if (cycle_hz <= 0) throw std::invalid_argument("cycle_hz must be > 0");
}

EthercatLoop::~EthercatLoop() {
    if (thread_.joinable()) {
        stop_flag_.store(true, std::memory_order_relaxed);
        thread_.join();
    }
}

SystemStatus EthercatLoop::runOnce() {
    const int64_t start_ns = monotonicNs();

    // Snapshot the pending command under lock.
    SystemCommand cmd;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        cmd = pending_command_;
        cmd.seq += 1;
        cmd.stamp_ns = start_ns;
        pending_command_.seq = cmd.seq;
    }

    // Encode and write output PDOs for each slave.
    // Note: SOEM sets Obytes=0 when Obits < 8 (e.g. EL2004 with 4 output bits).
    // Use Obits to compute the effective byte count so sub-byte slaves are handled.
    for (auto& [name, adapter] : runtime_.adapters) {
        const int soem_idx = runtime_.slave_index.at(name);
        auto* out_ptr      = ec_slave[soem_idx].outputs;
        const int out_bits = static_cast<int>(ec_slave[soem_idx].Obits);
        const int out_size = (out_bits + 7) / 8;

        if (out_ptr == nullptr || out_size == 0) continue;

        auto it = cmd.by_slave.find(name);
        if (it != cmd.by_slave.end()) {
            auto payload = adapter->packRxPdo(it->second);
            const int copy_bytes = std::min(static_cast<int>(payload.size()), out_size);
            std::memcpy(out_ptr, payload.data(), static_cast<std::size_t>(copy_bytes));
        } else {
            std::memset(out_ptr, 0, static_cast<std::size_t>(out_size));
        }
    }

    ec_send_processdata();
    const int wkc = ec_receive_processdata(EC_TIMEOUTRET);

    const int64_t end_ns        = monotonicNs();
    const int64_t cycle_time_ns = end_ns - start_ns;
    const int64_t dc_error_ns   = cycle_time_ns - cycle_ns_;

    // Decode input PDOs.
    SystemStatus status;
    status.seq      = cmd.seq;
    status.stamp_ns = end_ns;

    for (auto& [name, adapter] : runtime_.adapters) {
        const int soem_idx = runtime_.slave_index.at(name);
        const uint8_t* in_ptr  = ec_slave[soem_idx].inputs;
        const int      in_bits = static_cast<int>(ec_slave[soem_idx].Ibits);
        const int      in_size = (in_bits + 7) / 8;

        if (in_ptr == nullptr || in_size == 0) {
            // Input-less slaves (e.g. EL2004 in output-only mode): still
            // call unpackTxPdo with an empty buffer so the adapter can
            // return a default-constructed status.
            status.by_slave[name] = adapter->unpackTxPdo(
                nullptr, 0, cmd.seq, end_ns, cycle_time_ns, dc_error_ns
            );
        } else {
            status.by_slave[name] = adapter->unpackTxPdo(
                in_ptr, in_size, cmd.seq, end_ns, cycle_time_ns, dc_error_ns
            );
        }
    }

    // Update shared state under lock.
    {
        std::lock_guard<std::mutex> lk(mutex_);
        stats_.cycle_count++;
        stats_.last_wkc           = wkc;
        stats_.last_cycle_time_ns = cycle_time_ns;
        stats_.last_dc_error_ns   = dc_error_ns;

        // Only publish status when the bus confirmed delivery.  A zero WKC
        // means receive_processdata got no fresh frame — publishing stale
        // input bytes with a new timestamp would mislead readers.
        if (wkc > 0) {
            latest_status_ = status;
        }
    }

    return status;
}

void EthercatLoop::start() {
    if (thread_.joinable()) return;
    stop_flag_.store(false, std::memory_order_relaxed);
    thread_ = std::thread(&EthercatLoop::runForever, this);
}

void EthercatLoop::stop(double /*timeout_s*/) {
    // Signal the loop to exit, then join.  The loop checks stop_flag_ between
    // cycles, so join() returns within one cycle period (~1 ms at 1 kHz).
    stop_flag_.store(true, std::memory_order_relaxed);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void EthercatLoop::setCycleCallback(CycleCallback cb) {
    cycle_callback_ = std::move(cb);
}

void EthercatLoop::setCommand(const SystemCommand& cmd) {
    std::lock_guard<std::mutex> lk(mutex_);
    // Preserve the accumulated sequence number.
    const uint64_t current_seq = pending_command_.seq;
    pending_command_           = cmd;
    pending_command_.seq       = current_seq;
}

SystemStatus EthercatLoop::getStatus() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return latest_status_;
}

LoopStats EthercatLoop::stats() const {
    std::lock_guard<std::mutex> lk(mutex_);
    LoopStats s = stats_;
    s.callback_errors = callback_errors_.load(std::memory_order_relaxed);
    return s;
}

void EthercatLoop::applyRtConfig() {
    // CPU affinity.
    if (!rt_config_.cpu_affinity.empty()) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        for (int cpu : rt_config_.cpu_affinity) {
            CPU_SET(cpu, &cpuset);
        }
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
            // Non-fatal: print a warning but continue.
            std::fprintf(stderr,
                "[EthercatLoop] WARNING: pthread_setaffinity_np failed (errno=%d)\n",
                errno);
        }
    }

    // SCHED_FIFO real-time priority.
    if (rt_config_.rt_priority > 0) {
        struct sched_param param{};
        param.sched_priority = rt_config_.rt_priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            std::fprintf(stderr,
                "[EthercatLoop] WARNING: SCHED_FIFO priority=%d failed (errno=%d). "
                "Need CAP_SYS_NICE or root.\n",
                rt_config_.rt_priority, errno);
        }
    }
}

void EthercatLoop::runForever() {
    applyRtConfig();

    int64_t next_tick     = monotonicNs();
    int64_t prev_start_ns = 0;

    while (!stop_flag_.load(std::memory_order_relaxed)) {
        const int64_t start_ns      = monotonicNs();
        const int64_t period_ns     = (prev_start_ns == 0) ? 0 : (start_ns - prev_start_ns);
        const int64_t wakeup_latency = start_ns - next_tick;

        const SystemStatus cycle_status = runOnce();

        LoopStats cycle_stats;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            stats_.last_period_ns          = period_ns;
            stats_.last_wakeup_latency_ns  = wakeup_latency;
            cycle_stats = stats_;
        }

        if (cycle_callback_) {
            // The callback must never take down the RT thread: during a bus
            // loss it may see stale/partial status and throw (bad_any_cast).
            // Keep the previous pending_command_ — the drive PDO watchdog
            // covers prolonged failure.
            try {
                SystemCommand next = cycle_callback_(cycle_status, cycle_stats);
                std::lock_guard<std::mutex> lk(mutex_);
                next.seq = stats_.cycle_count;
                pending_command_ = std::move(next);
            } catch (const std::exception& e) {
                const uint64_t n =
                    callback_errors_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (n == 1 || n % 1000 == 0) {
                    std::fprintf(stderr,
                        "[EthercatLoop] cycle callback exception (#%llu): %s\n",
                        static_cast<unsigned long long>(n), e.what());
                }
            } catch (...) {
                const uint64_t n =
                    callback_errors_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (n == 1 || n % 1000 == 0) {
                    std::fprintf(stderr,
                        "[EthercatLoop] cycle callback unknown exception (#%llu)\n",
                        static_cast<unsigned long long>(n));
                }
            }
        }

        prev_start_ns = start_ns;
        next_tick    += cycle_ns_;
        const int64_t now = monotonicNs();

        if (now >= next_tick) {
            // Missed deadline — reset to avoid chasing accumulated lag.
            next_tick = now;
        } else {
            sleepUntilNs(next_tick);
        }
    }
}

} // namespace ethercat_core
