#!/usr/bin/env python3
"""Minimal pysoem bus scan with clearer interface diagnostics."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import pysoem

DEFAULT_IFACE = "ecat0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan EtherCAT slaves with pysoem.")
    parser.add_argument(
        "--iface",
        default=DEFAULT_IFACE,
        help=f"Network interface to open (default: {DEFAULT_IFACE}).",
    )
    return parser.parse_args()


def _sysfs_path(iface: str, leaf: str) -> Path:
    return Path("/sys/class/net") / iface / leaf


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def main() -> int:
    args = parse_args()
    iface = args.iface

    if not _sysfs_path(iface, "").exists():
        print(f"Interface '{iface}' does not exist.")
        available = sorted(os.listdir("/sys/class/net"))
        print("Available interfaces:", ", ".join(available))
        return 2

    operstate = _read_text(_sysfs_path(iface, "operstate")) or "unknown"
    print(f"Opening interface '{iface}' (operstate={operstate})")

    master = pysoem.Master()
    try:
        master.open(iface)
    except ConnectionError as exc:
        print(f"Failed to open interface '{iface}': {exc}")
        print("If the interface is correct, run with elevated privileges.")
        print(f"Example: sudo {Path(__file__).name} --iface {iface}")
        return 1

    try:
        slave_count = master.config_init()
        if slave_count > 0:
            print(f"Found {slave_count} slaves")
            for i, slave in enumerate(master.slaves):
                print(
                    f"Slave {i}: "
                    f"man=0x{slave.man:08x}, "
                    f"id=0x{slave.id:08x}, "
                    f"rev=0x{slave.rev:08x}, "
                    f"alias={int.from_bytes(slave.eeprom_read(4)[:2], 'little')}, "
                    f"name='{slave.name}'"
                )
        else:
            print("No slaves found")
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
