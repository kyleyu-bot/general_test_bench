#!/usr/bin/env python3
"""Integrated dyno test: drive speed command + EL5032 encoder + ELM3002 torque readback."""

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

from ethercat_core.data_types import SystemCommand
from ethercat_core.loop import EthercatLoop, LoopConfig
from ethercat_core.master import EthercatMaster, al_state_name, load_topology, resolve_slave_position
from ethercat_core.archive.devices.beckhoff.elm3002.adapter import Elm3002SlaveAdapter
from ethercat_core.archive.devices.beckhoff.elm3002.data_types import Elm3002Data
from ethercat_core.devices.beckhoff.el5032.adapter import El5032SlaveAdapter
from ethercat_core.devices.beckhoff.el5032.data_types import El5032Data
from ethercat_core.devices.motor_drives.Novanta.Everest.data_types import Command, ModeOfOperation


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
        description="Integrated dyno test: speed command + encoder + torque readback."
    )
    parser.add_argument(
        "--topology",
        default="config/ethercat_device_config/topology.dyno2.template3.json",
        help="Path to topology JSON file (must include all 4 devices).",
    )
    parser.add_argument(
        "--drive-slave",
        default="main_drive",
        help="Configured DS402 drive slave name.",
    )
    parser.add_argument(
        "--encoder-slave",
        default="encoder_interface",
        help="Configured EL5032 encoder slave name.",
    )
    parser.add_argument(
        "--torque-slave",
        default="analog_input_interface",
        help="Configured ELM3002 torque input slave name.",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=0,
        help="Speed command as int32 sent to the main_drive (0x60FF).",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=60.0,
        help="Total test duration in seconds.",
    )
    parser.add_argument(
        "--fault-reset-s",
        type=float,
        default=0.5,
        help="Fault-reset phase duration at test start.",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=5.0,
        help="Terminal status print rate in Hz.",
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
        help="Comma-separated CPU indices for the loop thread, e.g. '2' or '2,3'.",
    )
    return parser.parse_args()


def _clamp_i32(value: int) -> int:
    return max(-2147483648, min(2147483647, value))


def main() -> int:
    args = parse_args()

    cfg = load_topology(args.topology)

    # Resolve positions for all relevant slaves.
    for slave_name in (args.drive_slave, args.encoder_slave, args.torque_slave):
        resolved = resolve_slave_position(cfg, slave_name)
        for slave_cfg in cfg.slaves:
            if slave_cfg.name == slave_name:
                slave_cfg.position = resolved
                break

    master = EthercatMaster(cfg)

    try:
        runtime = master.initialize()

        # Validate adapters.
        drive_adapter = runtime.adapters.get(args.drive_slave)
        if drive_adapter is None:
            raise RuntimeError(
                f"Drive slave '{args.drive_slave}' not found. "
                f"Available: {list(runtime.adapters.keys())}"
            )

        encoder_adapter = runtime.adapters.get(args.encoder_slave)
        if not isinstance(encoder_adapter, El5032SlaveAdapter):
            raise RuntimeError(
                f"Slave '{args.encoder_slave}' is not an EL5032. "
                f"Adapter={type(encoder_adapter).__name__}"
            )

        torque_adapter = runtime.adapters.get(args.torque_slave)
        if not isinstance(torque_adapter, Elm3002SlaveAdapter):
            raise RuntimeError(
                f"Slave '{args.torque_slave}' is not an ELM3002. "
                f"Adapter={type(torque_adapter).__name__}"
            )

        rt_priority = max(0, min(args.rt_priority, 99))
        loop = EthercatLoop(
            runtime,
            cycle_hz=cfg.cycle_hz,
            rt_config=LoopConfig(
                rt_priority=rt_priority,
                cpu_affinity=args.cpu_affinity,
            ),
        )
        loop.start()

        t0 = time.monotonic()
        deadline = t0 + max(0.0, args.duration_s)
        reset_deadline = t0 + max(0.0, args.fault_reset_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = t0

        speed_cmd_i32 = _clamp_i32(int(args.speed))

        startup_params = dict(runtime.startup_params.get(args.drive_slave, {}))
        torque_kp = float(startup_params.get("motor_kt", 0.0))
        if abs(torque_kp) > 1e-9:
            torque_kp = 1.0 / torque_kp
        vel_qr = float(startup_params.get("torque_loop_max_output", 0.0))
        vel_is = float(startup_params.get("torque_loop_min_output", 0.0))
        vel_kp = float(startup_params.get("velocity_loop_kp", 0.0))
        vel_ki = float(startup_params.get("velocity_loop_ki", 0.0))
        vel_kd = float(startup_params.get("velocity_loop_kd", 0.0))
        pos_kp = float(startup_params.get("position_loop_kp", 0.0))
        pos_ki = float(startup_params.get("position_loop_ki", 0.0))
        pos_kd = float(startup_params.get("position_loop_kd", 0.0))

        print(
            f"Starting integrated dyno test | "
            f"speed_cmd={speed_cmd_i32} duration={args.duration_s:.1f}s | "
            f"rt_priority={rt_priority} cpu_affinity={sorted(args.cpu_affinity) or 'none'}"
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            in_reset = now < reset_deadline

            cmd = Command(
                mode_of_operation=ModeOfOperation.CYCLIC_SYNC_VELOCITY,
                target_torque_nm=0.0,
                target_velocity_rad_s=float(speed_cmd_i32),
                target_position_rad=0.0,
                torque_kp=torque_kp,
                torque_loop_max_output=vel_qr,
                torque_loop_min_output=vel_is,
                velocity_loop_kp=vel_kp,
                velocity_loop_ki=vel_ki,
                velocity_loop_kd=vel_kd,
                position_loop_kp=pos_kp,
                position_loop_ki=pos_ki,
                position_loop_kd=pos_kd,
                enable_drive=not in_reset,
                clear_fault=in_reset,
            )
            loop.set_command(SystemCommand(by_slave={args.drive_slave: cmd}))

            if now >= next_print:
                status = loop.get_status()
                stats = loop.stats

                # Drive status.
                ds = status.by_slave.get(args.drive_slave)
                drive_slave = runtime.slaves_by_name[args.drive_slave]
                drive_al = al_state_name(int(drive_slave.state))
                if ds is None:
                    drive_str = f"al={drive_al} state=unavailable speed_cmd={speed_cmd_i32} speed_fb=unavailable"
                else:
                    drive_str = (
                        f"al={drive_al} "
                        f"state={ds.cia402_state.name} "
                        f"speed_cmd={speed_cmd_i32} "
                        f"speed_fb={int(ds.measured_velocity_rad_s)}"
                    )

                # EL5032 encoder.
                enc_data = status.by_slave.get(args.encoder_slave)
                if not isinstance(enc_data, El5032Data):
                    enc_str = "external_encoder_count=unavailable"
                else:
                    enc_str = f"external_encoder_count={encoder_adapter.get_encoder_count_25bit(enc_data)}"

                # ELM3002 torque.
                torque_data = status.by_slave.get(args.torque_slave)
                if not isinstance(torque_data, Elm3002Data):
                    torque_str = "ch1_torque=unavailable ch2_torque=unavailable"
                else:
                    torque_str = (
                        f"ch1_torque={torque_adapter.get_pai_samples_1_scaled_torque(torque_data):.4f} "
                        f"ch2_torque={torque_adapter.get_pai_samples_2_scaled_torque(torque_data):.4f}"
                    )

                cycle_us = f"{stats.last_cycle_time_ns / 1000:.1f}"
                print(f"cycle_us={cycle_us} | {drive_str} | {enc_str} | {torque_str}")
                next_print = now + print_period

            time.sleep(0.005)

        loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
