#!/usr/bin/env python3
"""Read Beckhoff ELM3002 registers through SDO and optionally sample live PDO data."""

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
from ethercat_core.master import (
    EthercatMaster,
    MasterConfigError,
    al_state_name,
    load_topology,
    require_pysoem,
    resolve_slave_position,
)
from ethercat_core.archive.devices.beckhoff.elm3002.adapter import Elm3002SlaveAdapter
from ethercat_core.archive.devices.beckhoff.elm3002.data_types import ELM3002_TX_PDO_FIELDS, Elm3002Data


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
        description="Poll a Beckhoff ELM3002 register through SDO and optionally print a live PDO field."
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
        "--index",
        type=lambda x: int(x, 0),
        default=0x6000,
        help="SDO object index to read, for example 0x3101.",
    )
    parser.add_argument(
        "--subindex",
        type=lambda x: int(x, 0),
        default=0x01,
        help="SDO object subindex to read, for example 0x01.",
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
        help="Register print rate.",
    )
    parser.add_argument(
        "--pdo-index",
        type=lambda x: int(x, 0),
        default=0x1A01,
        help="Optional mapped TxPDO field index to print from live process data, for example 0x1A01.",
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
    require_pysoem()

    cfg = load_topology(args.topology)

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

    pdo_field = next(
        (field for field in ELM3002_TX_PDO_FIELDS if field.pdo_index == args.pdo_index),
        None,
    )
    if pdo_field is None:
        available = ", ".join(f"0x{field.pdo_index:04X}" for field in ELM3002_TX_PDO_FIELDS)
        raise ValueError(
            f"Unsupported ELM3002 PDO field 0x{args.pdo_index:04X}. Available: {available}"
        )

    master = EthercatMaster(cfg)
    loop: EthercatLoop | None = None
    try:
        runtime = master.initialize()
        adapter = runtime.adapters.get(args.slave)
        if not isinstance(adapter, Elm3002SlaveAdapter):
            raise RuntimeError(
                f"Slave '{args.slave}' is not an ELM3002. Adapter={type(adapter).__name__}"
            )

        slave = runtime.slaves_by_name[args.slave]
        loop = EthercatLoop(
            runtime,
            cycle_hz=cfg.cycle_hz,
            rt_config=LoopConfig(
                rt_priority=max(0, min(args.rt_priority, 99)),
                cpu_affinity=args.cpu_affinity,
            ),
        )
        loop.start()

        deadline = time.monotonic() + max(0.0, args.duration_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = time.monotonic()

        print(
            f"Monitoring '{args.slave}' at position {resolved_position} "
            f"register 0x{args.index:04X}:{args.subindex:02X} through SDO "
            f"and live PDO 0x{args.pdo_index:04X} "
            f"for {args.duration_s:.1f}s  |  "
            f"rt_priority={max(0, min(args.rt_priority, 99))} cpu_affinity={sorted(args.cpu_affinity) or 'none'}"
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_print:
                raw = slave.sdo_read(args.index, args.subindex)
                if isinstance(raw, int):
                    value = int(raw)
                    width = max(1, (value.bit_length() + 8) // 8)
                    payload_hex = value.to_bytes(width, "little", signed=value < 0).hex()
                elif isinstance(raw, (bytes, bytearray)):
                    payload = bytes(raw)
                    payload_hex = payload.hex()
                    value = int.from_bytes(payload, "little", signed=True)
                else:
                    raise TypeError(f"Unexpected SDO payload type: {type(raw)}")

                status = loop.get_status().by_slave.get(args.slave)
                pdo_raw_hex = "unavailable"
                pdo_value = "unavailable"
                if isinstance(status, Elm3002Data):
                    field_end = pdo_field.offset + pdo_field.size
                    if len(status.raw_pdo) >= field_end:
                        pdo_slice = status.raw_pdo[pdo_field.offset:field_end]
                        pdo_raw_hex = pdo_slice.hex()
                        pdo_value = str(
                            int.from_bytes(
                                pdo_slice,
                                byteorder="little",
                                signed=pdo_field.signed,
                            )
                        )

                al = al_state_name(int(slave.state))
                cycle_us = loop.stats.last_cycle_time_ns / 1000
                print(
                    f"al={al} cycle_us={cycle_us:.1f} "
                    f"raw_sdo={payload_hex} "
                    f"register_0x{args.index:04X}_{args.subindex:02X}={value} "
                    f"pdo_0x{args.pdo_index:04X}_raw={pdo_raw_hex} "
                    f"pdo_0x{args.pdo_index:04X}={pdo_value}"
                )
                next_print = now + print_period
            time.sleep(0.005)

        return 0
    finally:
        if loop is not None:
            loop.stop()
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
