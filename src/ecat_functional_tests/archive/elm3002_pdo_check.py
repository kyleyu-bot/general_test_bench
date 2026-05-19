#!/usr/bin/env python3
"""Print ELM3002 TxPDO datagram sections in hex for protocol inspection.

Section choices
---------------
  0  Full datagram (all PDO bytes)
  1  0x1A00  PAI Status ch1   (pai_status_1,   4 bytes) — num_samples (U8) + flags
  2  0x1A01  PAI Samples ch1  (pai_samples_1,  4 bytes) — subindex001 (INT32 ADC)
  3  0x1A10  Timestamp        (timestamp,       8 bytes)
  4  0x1A21  PAI Status ch2   (pai_status_2,   4 bytes) — num_samples (U8) + flags
  5  0x1A22  PAI Samples ch2  (pai_samples_2,  4 bytes) — subindex001 (INT32 ADC)

Reference: ELM3002.java / YoELM3002.java (phantom-hardware repo)
  - PAI Status PDO carries num_samples (U8) + status flags
  - PAI Samples PDO carries subindex001 (Signed32) — the raw ADC measurement
"""

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

from ethercat_core.loop import EthercatLoop, LoopConfig
from ethercat_core.master import EthercatMaster, al_state_name, load_topology, resolve_slave_position
from ethercat_core.archive.devices.beckhoff.elm3002.adapter import Elm3002SlaveAdapter
from ethercat_core.archive.devices.beckhoff.elm3002.data_types import ELM3002_TX_PDO_FIELDS, Elm3002Data

# Map section index → (PDO name, PDO index label)
_SECTION_META = {
    0: ("full",          "full datagram"),
    1: ("pai_status_1",  "0x1A00  PAI Status ch1"),
    2: ("pai_samples_1", "0x1A01  PAI Samples ch1"),
    3: ("timestamp",     "0x1A10  Timestamp"),
    4: ("pai_status_2",  "0x1A21  PAI Status ch2"),
    5: ("pai_samples_2", "0x1A22  PAI Samples ch2"),
}

# Pre-build offset/size lookup from the canonical field definitions.
_FIELD_SLICE: dict[str, tuple[int, int]] = {
    f.name: (f.offset, f.offset + f.size) for f in ELM3002_TX_PDO_FIELDS
}


def _extract_section(raw_pdo: bytes, section: int) -> bytes:
    if section == 0:
        return raw_pdo
    field_name, _ = _SECTION_META[section]
    start, end = _FIELD_SLICE[field_name]
    return raw_pdo[start:end]


def _hex_dump(label: str, data: bytes) -> str:
    hex_bytes = " ".join(f"{b:02X}" for b in data)
    return f"{label:<30s} [{len(data):2d} bytes]  {hex_bytes}"


def _parse_cpu_affinity(value: str) -> set[int]:
    cpus: set[int] = set()
    for item in value.split(","):
        token = item.strip()
        if not token:
            continue
        cpu = int(token, 10)
        if cpu < 0:
            raise argparse.ArgumentTypeError("CPU indices must be >= 0.")
        cpus.add(cpu)
    if not cpus:
        raise argparse.ArgumentTypeError("CPU affinity must include at least one CPU.")
    return cpus


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print ELM3002 TxPDO sections as hex for protocol inspection.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join(
            f"  {k}: {v[1]}" for k, v in _SECTION_META.items()
        ),
    )
    parser.add_argument(
        "section",
        type=int,
        choices=list(_SECTION_META),
        metavar="SECTION",
        help="PDO section to print (0=full, 1=1A00, 2=1A01, 3=1A10, 4=1A21, 5=1A22)",
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
    parser.add_argument(
        "--rt-priority",
        type=int,
        default=0,
        help="Loop thread SCHED_FIFO priority (1-99). 0 keeps default scheduler.",
    )
    parser.add_argument(
        "--cpu-affinity",
        type=_parse_cpu_affinity,
        default=set(),
        help="Comma-separated CPU indices for the loop thread, for example '2' or '2,3'.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_topology(args.topology)

    # Keep only the target ELM3002 slave — do not initialize main_drive or el5032.
    cfg.slaves = [s for s in cfg.slaves if s.name == args.slave]
    if not cfg.slaves:
        print(f"ERROR: slave '{args.slave}' not found in topology.", file=sys.stderr)
        return 1

    # Inject DC sync mode (SYNC0) SDO writes — applied in PREOP via pdo_mapping.
    # 0x1C32:01 = output SM sync mode, 0x1C33:01 = input SM sync mode, value 0x02 = DC SYNC0.
    for slave_cfg in cfg.slaves:
        if slave_cfg.name == args.slave:
            slave_cfg.pdo_mapping = [
                {"index": 0x1C32, "subindex": 0x01, "value": 0x02, "size": 2},
                {"index": 0x1C33, "subindex": 0x01, "value": 0x02, "size": 2},
                # {"index": 0x1C32, "subindex": 0x02, "value": 0x003D0900, "size": 4},
                # {"index": 0x1C33, "subindex": 0x02, "value": 0x003D0900, "size": 4},
            ] + slave_cfg.pdo_mapping

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

        loop = EthercatLoop(
            runtime,
            cycle_hz=cfg.cycle_hz,
            rt_config=LoopConfig(
                rt_priority=max(0, min(args.rt_priority, 99)),
                cpu_affinity=args.cpu_affinity,
            ),
        )
        loop.start()

        # Wait for the first valid PDO cycle (wkc > 0) before reading.
        print("Waiting for first valid PDO cycle...")
        while loop.get_status().stamp_ns == 0:
            time.sleep(0.001)

        stale_threshold_ns = int(3 * 1_000_000_000 / cfg.cycle_hz)

        section_label = _SECTION_META[args.section][1]
        deadline = time.monotonic() + max(0.0, args.duration_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = time.monotonic()

        print(
            f"Monitoring '{args.slave}' at position {resolved_position} "
            f"for {args.duration_s:.1f}s  |  section: {section_label}  |  "
            f"rt_priority={max(0, min(args.rt_priority, 99))} cpu_affinity={sorted(args.cpu_affinity) or 'none'}"
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_print:
                stats = loop.stats
                status = loop.get_status()
                data = status.by_slave.get(args.slave)
                age_ns = time.monotonic_ns() - status.stamp_ns

                # PDO ping: report working counter from the cyclic loop.
                slave = runtime.slaves_by_name[args.slave]
                pdo_ping = (
                    f"wkc={stats.last_wkc} "
                    f"cycles={stats.cycle_count} "
                    f"exec_us={stats.last_cycle_time_ns/1000:.1f} "
                    f"period_us={stats.last_period_ns/1000:.1f} "
                    f"wake_us={stats.last_wakeup_latency_ns/1000:.1f} "
                    f"al={al_state_name(int(slave.state))}"
                )

                # # Register 0x0130 read (approach TBD — SDO and PDO both need review).
                # try:
                #     raw_0130 = slave.sdo_read(0x0130, 0)
                #     reg_0130_hex = bytes(raw_0130).hex() if isinstance(raw_0130, (bytes, bytearray)) else f"{int(raw_0130):x}"
                # except Exception as exc:
                #     reg_0130_hex = f"ERR({exc})"

                if age_ns > stale_threshold_ns:
                    print(f"[STALE {age_ns / 1e6:.1f}ms] {pdo_ping}")
                elif not isinstance(data, Elm3002Data) or not data.raw_pdo:
                    print(f"pdo=unavailable  {pdo_ping}")
                else:
                    payload = _extract_section(data.raw_pdo, args.section)
                    print(f"{_hex_dump(section_label, payload)}  {pdo_ping}")
                next_print = now + print_period
            time.sleep(0.005)

        loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
