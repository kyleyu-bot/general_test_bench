#pragma once

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/master.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <set>
#include <thread>

namespace ethercat_core {

struct LoopStats {
    uint64_t cycle_count           = 0;
    int      last_wkc              = 0;
    int64_t  last_cycle_time_ns    = 0;
    int64_t  last_dc_error_ns      = 0;
    int64_t  last_period_ns        = 0;
    int64_t  last_wakeup_latency_ns = 0;
    uint64_t callback_errors       = 0;  // exceptions thrown by the cycle callback
};

struct LoopRtConfig {
    int            rt_priority  = 0;   // SCHED_FIFO priority 1–99; 0 = default
    std::set<int>  cpu_affinity;        // CPU indices to pin to; empty = no change
};

// Cyclic EtherCAT process-data loop.
//
// Call start() to launch a background real-time thread, then drive the loop
// via set_command() / get_status() from other threads.  For single-threaded
// use, call run_once() directly.
class EthercatLoop {
public:
    // Callback invoked from the RT thread after every completed cycle.
    // Must be fast and non-blocking (no I/O, no heavy allocation).
    // status and stats reflect the cycle just completed.
    using CycleCallback = std::function<SystemCommand(const SystemStatus&, const LoopStats&)>;

    explicit EthercatLoop(
        MasterRuntime&  runtime,
        int             cycle_hz  = 1000,
        LoopRtConfig    rt_config = {}
    );
    ~EthercatLoop();

    // Execute exactly one PDO exchange cycle.  Safe to call from any thread
    // when not using start()/stop() (i.e., in single-threaded mode).
    SystemStatus runOnce();

    void start();
    void stop(double timeout_s = 2.0);

    // Register a callback fired from the RT thread after each cycle.
    // Call before start().  Pass nullptr to clear.
    void setCycleCallback(CycleCallback cb);

    // Thread-safe command setter — takes effect on the next cycle.
    void setCommand(const SystemCommand& cmd);

    // Thread-safe snapshot of the latest status.
    SystemStatus getStatus() const;

    // Thread-safe snapshot of loop statistics.
    LoopStats stats() const;

private:
    void applyRtConfig();
    void runForever();

    MasterRuntime&  runtime_;
    int64_t         cycle_ns_;
    LoopRtConfig    rt_config_;

    mutable std::mutex mutex_;
    SystemCommand      pending_command_;
    SystemStatus       latest_status_;
    LoopStats          stats_;

    std::thread        thread_;
    std::atomic<bool>  stop_flag_{false};
    std::atomic<uint64_t> callback_errors_{0};

    CycleCallback      cycle_callback_;   // called from RT thread — must be cheap
};

} // namespace ethercat_core
