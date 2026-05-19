#!/usr/bin/env python3
"""Read and print Beckhoff ELM3002 TxPDO data in the cyclic loop."""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path

# Allow direct execution before install.
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ethercat_core.loop import EthercatLoop, LoopConfig
from ethercat_core.master import EthercatMaster, al_state_name, load_topology, resolve_slave_position
from ethercat_core.archive.devices.beckhoff.elm3002.adapter import Elm3002SlaveAdapter
from ethercat_core.archive.devices.beckhoff.elm3002.data_types import Elm3002Data


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
        description="Read and print ELM3002 TxPDO data using the cyclic loop."
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
        "--plot",
        action="store_true",
        help="Show a live pyqtgraph window for ELM3002 processed values.",
    )
    parser.add_argument(
        "--plot-window-s",
        type=float,
        default=10.0,
        help="Visible history window in seconds when --plot is enabled.",
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


def _format_status_line(adapter: Elm3002SlaveAdapter, data: Elm3002Data) -> str:
    return (
        f"ch1_voltage={adapter.get_pai_samples_1_scaled_voltage(data):.4f}V "
        f"ch1_torque={adapter.get_pai_samples_1_scaled_torque(data):.4f} "
        f"ch2_voltage={adapter.get_pai_samples_2_scaled_voltage(data):.4f}V "
        f"ch2_torque={adapter.get_pai_samples_2_scaled_torque(data):.4f}"
    )


def _run_live_plot(
    *,
    loop: EthercatLoop,
    adapter: Elm3002SlaveAdapter,
    slave_name: str,
    duration_s: float,
    window_s: float,
) -> int:
    try:
        import pyqtgraph as pg  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Live plotting requires pyqtgraph. Install it in the active interpreter, "
            "for example: python3 -m pip install pyqtgraph"
        ) from exc

    app = pg.mkQApp("ELM3002 Live Plot")
    win = pg.GraphicsLayoutWidget(title=f"ELM3002 Live Plot: {slave_name}")
    win.resize(1200, 700)

    voltage_plot = win.addPlot(title="Voltage")
    voltage_plot.showGrid(x=True, y=True, alpha=0.3)
    voltage_plot.addLegend()
    voltage_plot.setLabel("left", "Voltage", units="V")
    voltage_plot.setLabel("bottom", "Time", units="s")
    ch1_voltage_curve = voltage_plot.plot(
        pen=pg.mkPen("#1768AC", width=2), name="ch1 voltage"
    )
    ch2_voltage_curve = voltage_plot.plot(
        pen=pg.mkPen("#F26419", width=2), name="ch2 voltage"
    )

    win.nextRow()
    torque_plot = win.addPlot(title="Torque")
    torque_plot.showGrid(x=True, y=True, alpha=0.3)
    torque_plot.addLegend()
    torque_plot.setLabel("left", "Torque")
    torque_plot.setLabel("bottom", "Time", units="s")
    ch1_torque_curve = torque_plot.plot(
        pen=pg.mkPen("#2F9C95", width=2), name="ch1 torque"
    )
    ch2_torque_curve = torque_plot.plot(
        pen=pg.mkPen("#C1292E", width=2), name="ch2 torque"
    )

    time_hist: deque[float] = deque()
    ch1_voltage_hist: deque[float] = deque()
    ch2_voltage_hist: deque[float] = deque()
    ch1_torque_hist: deque[float] = deque()
    ch2_torque_hist: deque[float] = deque()

    start = time.monotonic()
    deadline = start + max(0.0, duration_s)

    def update() -> None:
        now = time.monotonic()
        if now >= deadline:
            timer.stop()
            win.close()
            app.quit()
            return

        status = loop.get_status().by_slave.get(slave_name)
        if not isinstance(status, Elm3002Data):
            return

        t = now - start
        time_hist.append(t)
        ch1_voltage_hist.append(adapter.get_pai_samples_1_scaled_voltage(status))
        ch2_voltage_hist.append(adapter.get_pai_samples_2_scaled_voltage(status))
        ch1_torque_hist.append(adapter.get_pai_samples_1_scaled_torque(status))
        ch2_torque_hist.append(adapter.get_pai_samples_2_scaled_torque(status))

        while time_hist and (t - time_hist[0]) > window_s:
            time_hist.popleft()
            ch1_voltage_hist.popleft()
            ch2_voltage_hist.popleft()
            ch1_torque_hist.popleft()
            ch2_torque_hist.popleft()

        xs = list(time_hist)
        ch1_voltage_curve.setData(xs, list(ch1_voltage_hist))
        ch2_voltage_curve.setData(xs, list(ch2_voltage_hist))
        ch1_torque_curve.setData(xs, list(ch1_torque_hist))
        ch2_torque_curve.setData(xs, list(ch2_torque_hist))

        if xs:
            x_min = max(0.0, xs[-1] - window_s)
            x_max = max(window_s, xs[-1])
            voltage_plot.setXRange(x_min, x_max, padding=0.0)
            torque_plot.setXRange(x_min, x_max, padding=0.0)

    timer = pg.QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)
    win.show()
    app.exec()
    return 0


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
            f"for {args.duration_s:.1f}s  |  "
            f"rt_priority={max(0, min(args.rt_priority, 99))} cpu_affinity={sorted(args.cpu_affinity) or 'none'}"
        )

        if args.plot:
            return _run_live_plot(
                loop=loop,
                adapter=adapter,
                slave_name=args.slave,
                duration_s=args.duration_s,
                window_s=max(0.5, args.plot_window_s),
            )

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_print:
                status = loop.get_status()
                data = status.by_slave.get(args.slave)
                stats = loop.stats
                slave = runtime.slaves_by_name[args.slave]
                al = al_state_name(int(slave.state))
                cycle_us = f"{stats.last_cycle_time_ns / 1000:.1f}"
                if not isinstance(data, Elm3002Data):
                    print(f"al={al} cycle_us={cycle_us} pai_status_1=unavailable  pai_samples_1=unavailable  pai_status_2=unavailable  pai_samples_2=unavailable")
                else:
                    print(f"al={al} cycle_us={cycle_us} {_format_status_line(adapter, data)}")
                next_print = now + print_period
            time.sleep(0.005)

        loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
