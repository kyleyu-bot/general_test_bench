#!/usr/bin/env python3
"""Read and print ELM3002 PAI Status ch1 and ch2 decoded fields in the cyclic loop."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Allow direct execution before install.
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ethercat_core.loop import EthercatLoop
from ethercat_core.master import EthercatMaster, al_state_name, load_topology, resolve_slave_position
from ethercat_core.archive.devices.beckhoff.elm3002.adapter import Elm3002SlaveAdapter
from ethercat_core.archive.devices.beckhoff.elm3002.data_types import Elm3002Data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read and print ELM3002 PAI Status ch1 and ch2 decoded fields."
    )
    parser.add_argument(
        "--topology",
        default="config/ethercat_device_config/topology.dyno2.template3.json",
        help="Path to topology JSON file.",
    )
    parser.add_argument(
        "--slave",
        default="analog_input_interface",
        help="Configured ELM3002 slave name to observe.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=60.0,
        help="Monitor duration in seconds.",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=5.0,
        help="Terminal update rate.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_topology(args.topology)
    resolved_position = resolve_slave_position(cfg, args.slave)
    for slave_cfg in cfg.slaves:
        if slave_cfg.name == args.slave:
            slave_cfg.position = resolved_position
            break

    master = EthercatMaster(cfg)

    try:
        runtime = master.initialize()
        adapter = runtime.adapters.get(args.slave)
        if not isinstance(adapter, Elm3002SlaveAdapter):
            raise RuntimeError(
                f"Slave '{args.slave}' is not an ELM3002. Adapter={type(adapter).__name__}"
            )

        loop = EthercatLoop(runtime, cycle_hz=cfg.cycle_hz)
        loop.start()

        # Wait for the first valid PDO cycle (wkc > 0) before reading.
        print("Waiting for first valid PDO cycle...")
        while loop.get_status().stamp_ns == 0:
            time.sleep(0.001)

        # Data older than 3 cycle periods is considered stale.
        stale_threshold_ns = int(3 * 1_000_000_000 / cfg.cycle_hz)

        deadline = time.monotonic() + max(0.0, args.duration_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = time.monotonic()

        print(
            f"Monitoring '{args.slave}' at position {resolved_position} "
            f"for {args.duration_s:.1f}s"
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_print:
                status = loop.get_status()
                data = status.by_slave.get(args.slave)
                age_ns = time.monotonic_ns() - status.stamp_ns
                stats = loop.stats
                slave = runtime.slaves_by_name[args.slave]
                al = al_state_name(int(slave.state))
                cycle_us = f"{stats.last_cycle_time_ns / 1000:.1f}"
                if age_ns > stale_threshold_ns:
                    print(f"[STALE {age_ns / 1e6:.1f}ms] al={al} cycle_us={cycle_us} wkc=0 — no fresh frame received")
                elif not isinstance(data, Elm3002Data):
                    print(f"al={al} cycle_us={cycle_us} pai_status_1=unavailable  pai_status_2=unavailable")
                else:
                    ps1 = adapter.get_pai_status_1(data)
                    ps2 = adapter.get_pai_status_2(data)
                    print(
                        f"al={al} cycle_us={cycle_us}  ch1  num_samples={ps1.num_samples}  error={ps1.error}"
                        f"  underrange={ps1.underrange}  overrange={ps1.overrange}"
                        f"  diag={ps1.diag}  txpdo_state={ps1.txpdo_state}"
                        f"  input_cycle_counter={ps1.input_cycle_counter}"
                    )
                    print(
                        f"al={al}  ch2  num_samples={ps2.num_samples}  error={ps2.error}"
                        f"  underrange={ps2.underrange}  overrange={ps2.overrange}"
                        f"  diag={ps2.diag}  txpdo_state={ps2.txpdo_state}"
                        f"  input_cycle_counter={ps2.input_cycle_counter}"
                    )
                    print()
                next_print = now + print_period
            time.sleep(0.005)

        loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
