"""
Pure analysis module for Kt (torque constant) extraction.

No GUI dependencies — safe to import from test scripts or headless pipelines.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

import numpy as np
import pandas as pd
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

_DEVIATION_THRESHOLD = 0.01  # Nm — minimum torque deviation to detect ramp start

_DRIVE_COLS = {
    "main": ("main_rx_torque_command", "main_tx_iq_actual", "main_gear_ratio"),
    "dut":  ("dut_rx_torque_command",  "dut_tx_iq_actual",  "dut_gear_ratio"),
}


# ── CSV helpers ───────────────────────────────────────────────────────────────

def _find_csv(folder: str) -> Path:
    p = Path(folder)
    for name in ("dyno_pdo.csv.gz", "dyno_pdo.csv"):
        if (p / name).exists():
            return p / name
    if p.suffix in (".csv", ".gz") and p.exists():
        return p
    raise FileNotFoundError(f"No dyno_pdo.csv(.gz) found in {folder}")


def _latest_log(root: str = "test_data_log") -> Path:
    base = Path(root)
    csvs = list(base.glob("*/*/dyno_pdo.csv.gz")) + list(base.glob("*/*/dyno_pdo.csv"))
    if not csvs:
        raise FileNotFoundError(f"No dyno_pdo.csv(.gz) found under {root}")
    return max(csvs, key=lambda p: p.stat().st_mtime).parent


def _load_csv(csv_path: Path) -> tuple[np.ndarray, list[str]]:
    df_raw = pd.read_csv(csv_path)
    header = list(df_raw.columns)
    data   = df_raw.to_numpy(dtype=float)
    return data, header


# ── Validation and test-type detection ───────────────────────────────────────

def _validate_kt_data(data: np.ndarray, header: list[str], drive: str) -> None:
    torque_cmd_col, _, _ = _DRIVE_COLS[drive]
    if torque_cmd_col not in header:
        raise ValueError(
            f"Column '{torque_cmd_col}' not found. Is this a torque-ramp log?"
        )
    idx = header.index(torque_cmd_col)
    torque_cmd = data[:, idx]
    if not np.any(np.abs(torque_cmd - torque_cmd[0]) > _DEVIATION_THRESHOLD):
        raise ValueError(
            "No torque command deviation detected in this log.\n"
            "Select a folder from a torque-ramp test."
        )


def _is_2way_test(csv_path: Path) -> bool:
    return "torque_ramp_2way" in csv_path.parent.name


# ── Segment detection ─────────────────────────────────────────────────────────

def _detect_ramp_segment(
    data: np.ndarray, header: list[str], drive: str, torque_sensor_col: str
) -> tuple[np.ndarray, np.ndarray, float]:
    torque_cmd_col, iq_col, gear_col = _DRIVE_COLS[drive]

    torque_cmd = data[:, header.index(torque_cmd_col)]
    iq         = data[:, header.index(iq_col)]
    torque_sns = data[:, header.index(torque_sensor_col)]
    gear_ratio = data[0, header.index(gear_col)] if gear_col in header else 1.0

    initial = torque_cmd[0]
    dev_indices = np.where(np.abs(torque_cmd - initial) > _DEVIATION_THRESHOLD)[0]
    if len(dev_indices) == 0:
        raise ValueError("No torque deviation found.")
    t0_idx = max(0, dev_indices[0] - 1)
    t1_idx = int(np.argmax(np.abs(torque_cmd)))

    if t1_idx <= t0_idx:
        raise ValueError(
            f"Peak torque index ({t1_idx}) is not after ramp start ({t0_idx})."
        )

    return iq[t0_idx : t1_idx + 1], torque_sns[t0_idx : t1_idx + 1], float(gear_ratio)


def _detect_2way_segments(
    data: np.ndarray, header: list[str], drive: str, torque_sensor_col: str
) -> tuple[tuple, tuple, tuple, float]:
    """Return ((iq1,tor1), (iq2,tor2), (iq3,tor3), gear_ratio) for the three ramp legs."""
    torque_cmd_col, iq_col, gear_col = _DRIVE_COLS[drive]

    torque_cmd = data[:, header.index(torque_cmd_col)]
    iq         = data[:, header.index(iq_col)]
    torque_sns = data[:, header.index(torque_sensor_col)]
    gear_ratio = data[0, header.index(gear_col)] if gear_col in header else 1.0

    initial = torque_cmd[0]

    dev = np.where(np.abs(torque_cmd - initial) > _DEVIATION_THRESHOLD)[0]
    if len(dev) == 0:
        raise ValueError("No torque deviation found.")
    t0_idx = max(0, dev[0] - 1)

    t1_idx = int(np.argmax(torque_cmd))
    t2_idx = int(np.argmin(torque_cmd[t1_idx:])) + t1_idx

    after_t2 = torque_cmd[t2_idx:]
    returned = np.where(np.abs(after_t2 - initial) < _DEVIATION_THRESHOLD)[0]
    t3_idx = int(returned[0]) + t2_idx if len(returned) > 0 else len(torque_cmd) - 1

    seg1 = (iq[t0_idx : t1_idx + 1], torque_sns[t0_idx : t1_idx + 1])
    seg2 = (iq[t1_idx : t2_idx + 1], torque_sns[t1_idx : t2_idx + 1])
    seg3 = (iq[t2_idx : t3_idx + 1], torque_sns[t2_idx : t3_idx + 1])
    return seg1, seg2, seg3, float(gear_ratio)


# ── Linear fit ────────────────────────────────────────────────────────────────

def _linear_fit(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    """Return (slope, intercept, R²)."""
    coeffs = np.polyfit(x, y, 1)
    slope, intercept = float(coeffs[0]), float(coeffs[1])
    y_fit = slope * x + intercept
    ss_res = np.sum((y - y_fit) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return slope, intercept, r2


def _detect_torque_inversion(
    data: np.ndarray, header: list[str], drive: str, torque_sensor_col: str
) -> bool:
    """Return True when torque sensor sign opposes the commanded current sign."""
    _, iq_col, _ = _DRIVE_COLS[drive]
    current_col = next(
        (
            col for col in (
                f"{drive}_rx_iq_command",
                f"{drive}_tx_iq_command",
                iq_col,
            )
            if col in header and np.any(np.abs(data[:, header.index(col)]) > _DEVIATION_THRESHOLD)
        ),
        iq_col,
    )
    current = data[:, header.index(current_col)]
    torque = data[:, header.index(torque_sensor_col)]

    peak_idx = int(np.argmax(np.abs(current)))
    peak_iq = current[peak_idx]
    peak_torque = torque[peak_idx]

    if np.isclose(peak_iq, 0.0) or np.isclose(peak_torque, 0.0):
        return False
    return bool(np.sign(peak_iq) != np.sign(peak_torque))


# ── Figure builder (backend-agnostic) ─────────────────────────────────────────

def _expand_data_area(ax, x_margin: float = 0.06, y_margin: float = 0.12) -> None:
    """Give plotted data breathing room inside the axes without expanding the crop."""
    ax.margins(x=x_margin, y=y_margin)
    ax.autoscale_view()


def _finish_axis(ax, x_margin: float = 0.06, y_margin: float = 0.12) -> None:
    ax.minorticks_on()
    ax.grid(True, which="major", alpha=0.3)
    ax.grid(True, which="minor", alpha=0.12)
    _expand_data_area(ax, x_margin=x_margin, y_margin=y_margin)


def _draw_time_axis(
    ax,
    t_s: np.ndarray,
    values: np.ndarray,
    ylabel: str,
    title: str,
    color: str | None = None,
) -> None:
    plot_kwargs = {"linewidth": 0.8}
    if color is not None:
        plot_kwargs["color"] = color
    ax.plot(t_s, values, **plot_kwargs)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    _finish_axis(ax, x_margin=0.05, y_margin=0.18)


def _draw_kt_axis(
    ax,
    iq: np.ndarray,
    torque: np.ndarray,
    fit: tuple,
    ylabel: str,
    title: str,
) -> None:
    kt, intercept, r2 = fit
    iq_fit = np.linspace(iq.min(), iq.max(), 200)

    ax.scatter(iq, torque, s=4, alpha=0.5, label="data")
    for _idx, _dx in [(np.argmin(iq), 5), (np.argmax(iq), -5)]:
        ax.annotate(
            f"({torque[_idx]:.2f}, {iq[_idx]:.2f})",
            xy=(iq[_idx], torque[_idx]),
            xytext=(_dx, 5), textcoords="offset points",
            fontsize=7, ha="left" if _dx > 0 else "right",
        )
    ax.plot(iq_fit, kt * iq_fit + intercept, "r-",
            label=f"fit: Kt = {kt:.4f} Nm/A\nR² = {r2:.4f}")
    ax.set_xlabel("Iq actual (A)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(fontsize=9)
    _finish_axis(ax)


def make_kt_figure(
    seg1: tuple,
    tor1: np.ndarray,
    gear_ratio: float,
    fit1_out: tuple,
    fit1_mot: tuple,
    sensor_col: str,
    extra_legs,           # None  or  list of (iq, tor, fit_out, fit_mot)
    t_s: np.ndarray,
    full_iq: np.ndarray,
    full_torque: np.ndarray,
) -> tuple[Figure, list]:
    """
    Build and return (fig, axes_info).

    axes_info contains preview axes plus per-axis redraw callbacks for PNG saving.
    Uses matplotlib.figure.Figure directly — no pyplot, no backend dependency.
    """
    n_fit_rows = 1 if extra_legs is None else 3
    n_rows     = n_fit_rows + 1   # row 0 = time-series overview
    fig = Figure(figsize=(12, 5 * n_rows), tight_layout=True)

    axes_info: list[tuple] = []

    # ── Row 0: raw time-series ────────────────────────────────────────────────
    ax_iq = fig.add_subplot(n_rows, 2, 1)
    draw_iq_time = lambda ax: _draw_time_axis(
        ax, t_s, full_iq, "Iq actual (A)", "Iq actual — full run")
    draw_iq_time(ax_iq)
    axes_info.append((ax_iq, "iq_vs_time", draw_iq_time, (8, 5)))

    ax_tor = fig.add_subplot(n_rows, 2, 2)
    draw_torque_time = lambda ax: _draw_time_axis(
        ax, t_s, full_torque, f"{sensor_col} (Nm)",
        f"{sensor_col} — full run", color="tab:orange")
    draw_torque_time(ax_tor)
    axes_info.append((ax_tor, "torque_vs_time", draw_torque_time, (8, 5)))

    leg_labels = ["Seg 1 (+ ramp)", "Seg 2 (+ → −)", "Seg 3 (− return)"]
    leg_stems  = ["seg1_pos_ramp",  "seg2_sweep",     "seg3_neg_return"]
    legs = [(seg1[0], tor1, fit1_out, fit1_mot)]
    if extra_legs is not None:
        legs += list(extra_legs)

    for row, (iq, tor, fit_out, fit_mot) in enumerate(legs):
        label  = leg_labels[row]
        stem   = leg_stems[row]
        base   = (row + 1) * 2 + 1   # +1 row offset for time-series row

        ax_out = fig.add_subplot(n_rows, 2, base)
        draw_output = lambda ax, iq=iq, tor=tor, fit_out=fit_out, label=label: _draw_kt_axis(
            ax, iq, tor, fit_out, f"{sensor_col} (Nm)",
            f"{label} — Output shaft Kt")
        draw_output(ax_out)
        axes_info.append((ax_out, f"{stem}_output_kt", draw_output, (8, 6)))

        ax_mot = fig.add_subplot(n_rows, 2, base + 1)
        draw_motor = lambda ax, iq=iq, tor=tor, fit_mot=fit_mot, label=label: _draw_kt_axis(
            ax, iq, tor / gear_ratio, fit_mot,
            f"{sensor_col} / gear_ratio (Nm)",
            f"{label} — Motor shaft Kt  (GR: {gear_ratio:.4f})")
        draw_motor(ax_mot)
        axes_info.append((ax_mot, f"{stem}_motor_kt", draw_motor, (8, 6)))

    return fig, axes_info


def save_kt_subplots(
    fig: Figure,
    axes_info: list[tuple],
    out_dir: str | Path,
    dpi: int = 150,
    pad_inches: float = 0.12,
) -> list[Path]:
    """Save each axes in axes_info as its own tightly cropped PNG."""
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    canvas = fig.canvas
    if not hasattr(canvas, "get_renderer"):
        canvas = FigureCanvasAgg(fig)
    canvas.draw()
    renderer = canvas.get_renderer()

    saved_paths: list[Path] = []
    for item in axes_info:
        ax, stem = item[:2]
        path = out_path / f"kt_{stem}.png"

        if len(item) >= 3:
            draw_axis = item[2]
            figsize = item[3] if len(item) >= 4 else (8, 6)
            single_fig = Figure(figsize=figsize, tight_layout=True)
            single_ax = single_fig.add_subplot(1, 1, 1)
            draw_axis(single_ax)
            single_fig.savefig(str(path), dpi=dpi, bbox_inches="tight",
                               pad_inches=pad_inches)
        else:
            bbox = ax.get_tightbbox(renderer).transformed(fig.dpi_scale_trans.inverted())
            bbox = bbox.padded(pad_inches)
            fig.savefig(str(path), dpi=dpi, bbox_inches=bbox)

        saved_paths.append(path)

    return saved_paths


# ── Serial number + output directory ─────────────────────────────────────────

def _read_serial_number(log_folder: Path) -> str | None:
    sn_file = log_folder / "actuator_serial_number.txt"
    if not sn_file.exists():
        return None
    try:
        text = sn_file.read_text()
        m = re.search(r'=\s*"?([^"\n\r]+)"?', text)
        if m:
            return m.group(1).strip()
    except OSError:
        pass
    return None


def _resolve_output_dir(log_folder: Path) -> Path:
    sn = _read_serial_number(log_folder)
    if sn:
        repo_root = Path(__file__).resolve().parents[4]
        out = repo_root / "actuator_test_log" / sn / log_folder.name
    else:
        out = log_folder
    out.mkdir(parents=True, exist_ok=True)
    return out


# ── Full headless pipeline ────────────────────────────────────────────────────

def run_kt_analysis(
    log_folder: str | Path,
    drive: str = "main",
    torque_sensor: str = "ch1",
    invert: bool | None = None,
) -> dict:
    """
    Load CSV, detect ramp segments, fit Kt, save PNGs and kt_values.txt.

    invert=None (default): auto-detect by comparing the sign of the peak current
    against the sensor reading at that same timestamp. If they differ the sensor
    channel is flipped, matching the "Invert torque" checkbox in the GUI.
    Pass invert=True/False to override.

    Returns a dict with Kt values (keyed kt_output, kt_motor, or with _segN suffix
    for 2-way tests). Raises on bad data so the caller can decide how to handle it.
    """
    log_folder = Path(log_folder)
    csv_path   = _find_csv(str(log_folder))
    data, header = _load_csv(csv_path)
    _validate_kt_data(data, header, drive)

    sensor_col = f"torque_{torque_sensor}_nm"
    if sensor_col not in header:
        raise ValueError(f"Column '{sensor_col}' not found in CSV.")

    two_way = _is_2way_test(csv_path)

    if two_way:
        seg1, seg2, seg3, gear_ratio = _detect_2way_segments(
            data, header, drive, sensor_col)
    else:
        iq, torque, gear_ratio = _detect_ramp_segment(
            data, header, drive, sensor_col)
        seg1 = (iq, torque)

    if invert is None:
        invert = _detect_torque_inversion(data, header, drive, sensor_col)

    def _fits(iq, torque):
        if invert:
            torque = -torque
        return torque, _linear_fit(iq, torque), _linear_fit(iq, torque / gear_ratio)

    tor1, fit1_out, fit1_mot = _fits(*seg1)
    extra_legs = None
    if two_way:
        tor2, fit2_out, fit2_mot = _fits(*seg2)
        tor3, fit3_out, fit3_mot = _fits(*seg3)
        extra_legs = [
            (seg2[0], tor2, fit2_out, fit2_mot),
            (seg3[0], tor3, fit3_out, fit3_mot),
        ]

    _, iq_col, _ = _DRIVE_COLS[drive]
    t_s         = (data[:, header.index("stamp_ns")] - data[0, header.index("stamp_ns")]) * 1e-9
    full_iq     = data[:, header.index(iq_col)]
    full_torque = data[:, header.index(sensor_col)]
    if invert:
        full_torque = -full_torque

    fig, axes_info = make_kt_figure(
        seg1, tor1, gear_ratio,
        fit1_out, fit1_mot, sensor_col, extra_legs,
        t_s, full_iq, full_torque,
    )

    out_dir = _resolve_output_dir(log_folder)

    save_kt_subplots(fig, axes_info, out_dir)

    # Build results dict and write kt_values.txt
    kt_results: dict = {}
    lines: list[str] = []

    if two_way:
        for i, (fit_out, fit_mot) in enumerate(
            [(fit1_out, fit1_mot), (fit2_out, fit2_mot), (fit3_out, fit3_mot)], start=1
        ):
            key_out = f"kt_output_seg{i}"
            key_mot = f"kt_motor_seg{i}"
            kt_results[key_out] = fit_out[0]
            kt_results[key_mot] = fit_mot[0]
            lines.append(f"{key_out} = {fit_out[0]:.6f}")
            lines.append(f"{key_mot} = {fit_mot[0]:.6f}")
    else:
        kt_results["kt_output"] = fit1_out[0]
        kt_results["kt_motor"]  = fit1_mot[0]
        lines.append(f"kt_output = {fit1_out[0]:.6f}")
        lines.append(f"kt_motor  = {fit1_mot[0]:.6f}")

    (out_dir / "kt_values.txt").write_text("\n".join(lines) + "\n")

    return kt_results
