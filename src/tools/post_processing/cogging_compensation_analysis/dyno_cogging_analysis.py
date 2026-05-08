"""
Pure analysis module for cogging torque extraction.

No GUI dependencies — safe to import from test scripts or headless pipelines.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator

_VEL_THRESHOLD_FRAC = 0.05   # fraction of peak |vel_cmd| to detect active segments

_DRIVE_COLS = {
    "main": ("main_rx_target_velocity", "main_tx_iq_actual", "main_tx_input_enc_pos"),
    "dut":  ("dut_rx_target_velocity",  "dut_tx_iq_actual",  "dut_tx_input_enc_pos"),
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


# ── Segment detection ─────────────────────────────────────────────────────────

def _detect_segments(
    data: np.ndarray, header: list[str], drive: str
) -> tuple[int, int, int]:
    """Return (t_start_idx, t_mid_idx, t_end_idx) for forward/reverse legs."""
    vel_col = _DRIVE_COLS[drive][0]
    if vel_col not in header:
        raise ValueError(f"Column '{vel_col}' not found. Is this a cogging test log?")

    vel_cmd = data[:, header.index(vel_col)]
    peak    = np.max(np.abs(vel_cmd))
    if peak == 0:
        raise ValueError("Velocity command is all-zero — no motion detected.")
    threshold = _VEL_THRESHOLD_FRAC * peak

    fwd = np.where(vel_cmd > threshold)[0]
    if len(fwd) == 0:
        raise ValueError("No forward velocity segment detected.")
    t_start = int(fwd[0])

    rev = np.where(vel_cmd[t_start:] < -threshold)[0]
    if len(rev) == 0:
        raise ValueError("No reverse velocity segment detected.")
    t_mid = int(rev[0]) + t_start

    ended = np.where(np.abs(vel_cmd[t_mid:]) < threshold)[0]
    t_end = int(ended[0]) + t_mid if len(ended) > 0 else len(vel_cmd) - 1

    return t_start, t_mid, t_end


# ── Encoder-space resampling ──────────────────────────────────────────────────

def _resample_to_enc_space(
    enc: np.ndarray, iq: np.ndarray, n_points: int = 2048
) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate iq onto n_points uniform encoder positions."""
    order = np.argsort(enc)
    common_enc   = np.linspace(enc.min(), enc.max(), n_points)
    iq_resampled = np.interp(common_enc, enc[order], iq[order])
    return common_enc, iq_resampled


# ── Figure builder (backend-agnostic) ─────────────────────────────────────────

def make_cogging_figure(
    common_enc: np.ndarray,
    iq1: np.ndarray,
    iq2: np.ndarray,
    iq_avg: np.ndarray,
    fft_seg1: np.ndarray,
    fft_seg2: np.ndarray,
    fft_avg: np.ndarray,
    harmonics: np.ndarray,
    t_s: np.ndarray,
    full_iq: np.ndarray,
    full_vel: np.ndarray,
    drive: str,
    harmonics_avg: np.ndarray | None = None,
) -> tuple[Figure, list]:
    """
    Build and return (fig, axes_info).

    axes_info is a list of (ax, stem_str) for per-axis PNG saving.
    Uses matplotlib.figure.Figure directly — no pyplot, no backend dependency.
    """
    n_rows = 4
    n_cols = 2
    fig = Figure(figsize=(14, 5 * n_rows), tight_layout=True)
    axes_info: list[tuple] = []

    vel_col_label = _DRIVE_COLS[drive][0]

    # ── Row 0: raw time-series ────────────────────────────────────────────────
    ax_iq = fig.add_subplot(n_rows, n_cols, 1)
    ax_iq.plot(t_s, full_iq, linewidth=0.8)
    ax_iq.set_xlabel("Time (s)")
    ax_iq.set_ylabel("Iq actual (A)")
    ax_iq.set_title("Iq actual — full run")
    ax_iq.grid(True, alpha=0.3)
    axes_info.append((ax_iq, "iq_vs_time"))

    ax_vel = fig.add_subplot(n_rows, n_cols, 2)
    ax_vel.plot(t_s, full_vel, linewidth=0.8, color="tab:orange")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel(f"{vel_col_label}")
    ax_vel.set_title("Velocity command — full run")
    ax_vel.grid(True, alpha=0.3)
    axes_info.append((ax_vel, "vel_vs_time"))

    # ── Row 1: Iq vs encoder position per segment ─────────────────────────────
    ax_s1 = fig.add_subplot(n_rows, n_cols, 3)
    ax_s1.plot(common_enc, iq1, linewidth=0.8)
    ax_s1.set_xlabel("Encoder position (counts)")
    ax_s1.set_ylabel("Iq actual (A)")
    ax_s1.set_title("Iq vs encoder — Forward")
    ax_s1.grid(True, alpha=0.3)
    axes_info.append((ax_s1, "seg1_iq_vs_enc"))

    ax_s2 = fig.add_subplot(n_rows, n_cols, 4)
    ax_s2.plot(common_enc, iq2, linewidth=0.8, color="tab:orange")
    ax_s2.set_xlabel("Encoder position (counts)")
    ax_s2.set_ylabel("Iq actual (A)")
    ax_s2.set_title("Iq vs encoder — Reverse")
    ax_s2.grid(True, alpha=0.3)
    axes_info.append((ax_s2, "seg2_iq_vs_enc"))

    # ── Row 2: averaged Iq vs encoder ────────────────────────────────────────
    ax_avg = fig.add_subplot(n_rows, n_cols, 5)
    ax_avg.plot(common_enc, iq_avg, linewidth=0.8, color="tab:green")
    ax_avg.set_xlabel("Encoder position (counts)")
    ax_avg.set_ylabel("Iq actual (A)")
    ax_avg.set_title("Iq vs encoder — Averaged (friction cancelled)")
    ax_avg.grid(True, alpha=0.3)
    axes_info.append((ax_avg, "avg_iq_vs_enc"))

    # Row 2 right — averaged FFT
    _h_avg = harmonics_avg if harmonics_avg is not None else harmonics
    ax_fa_main = fig.add_subplot(n_rows, n_cols, 6)
    ax_fa_main.plot(_h_avg, fft_avg, linewidth=0.8, color="tab:green")
    ax_fa_main.set_xlabel("Harmonic (cycles/rev)")
    ax_fa_main.set_ylabel("|FFT| (A)")
    ax_fa_main.set_title("FFT — Averaged")
    ax_fa_main.set_xlim(0, 200)
    ax_fa_main.xaxis.set_major_locator(MultipleLocator(10))
    ax_fa_main.xaxis.set_minor_locator(MultipleLocator(5))
    ax_fa_main.grid(True, which="major", alpha=0.3)
    ax_fa_main.grid(True, which="minor", alpha=0.12)
    axes_info.append((ax_fa_main, "fft_avg"))

    # ── Row 3: FFT harmonics ──────────────────────────────────────────────────
    ax_f1 = fig.add_subplot(n_rows, n_cols, 7)
    ax_f1.plot(harmonics, fft_seg1, linewidth=0.8)
    ax_f1.set_xlabel("Harmonic (cycles/rev)")
    ax_f1.set_ylabel("|FFT| (A)")
    ax_f1.set_title("FFT — Forward")
    ax_f1.set_xlim(0, 200)
    ax_f1.xaxis.set_major_locator(MultipleLocator(10))
    ax_f1.xaxis.set_minor_locator(MultipleLocator(5))
    ax_f1.grid(True, which="major", alpha=0.3)
    ax_f1.grid(True, which="minor", alpha=0.12)
    axes_info.append((ax_f1, "fft_seg1"))

    ax_f2 = fig.add_subplot(n_rows, n_cols, 8)
    ax_f2.plot(harmonics, fft_seg2, linewidth=0.8, color="tab:orange")
    ax_f2.set_xlabel("Harmonic (cycles/rev)")
    ax_f2.set_ylabel("|FFT| (A)")
    ax_f2.set_title("FFT — Reverse")
    ax_f2.set_xlim(0, 200)
    ax_f2.xaxis.set_major_locator(MultipleLocator(10))
    ax_f2.xaxis.set_minor_locator(MultipleLocator(5))
    ax_f2.grid(True, which="major", alpha=0.3)
    ax_f2.grid(True, which="minor", alpha=0.12)
    axes_info.append((ax_f2, "fft_seg2"))

    return fig, axes_info


def make_cogging_avg_fft_figure(
    common_enc: np.ndarray,
    iq_avg: np.ndarray,
    fft_avg: np.ndarray,
    harmonics: np.ndarray,
) -> tuple[Figure, list]:
    """Separate figure for averaged plots (saved as fft_avg and avg_iq_vs_enc)."""
    fig = Figure(figsize=(14, 10), tight_layout=True)
    axes_info: list[tuple] = []

    ax_avg = fig.add_subplot(2, 1, 1)
    ax_avg.plot(common_enc, iq_avg, linewidth=0.8, color="tab:green")
    ax_avg.set_xlabel("Encoder position (counts)")
    ax_avg.set_ylabel("Iq actual (A)")
    ax_avg.set_title("Iq vs encoder — Averaged (friction cancelled)")
    ax_avg.grid(True, alpha=0.3)
    axes_info.append((ax_avg, "avg_iq_vs_enc"))

    ax_fa = fig.add_subplot(2, 1, 2)
    ax_fa.plot(harmonics, fft_avg, linewidth=0.8, color="tab:green")
    ax_fa.set_xlabel("Harmonic (cycles/rev)")
    ax_fa.set_ylabel("|FFT| (A)")
    ax_fa.set_title("FFT — Averaged")
    ax_fa.set_xlim(0, 200)
    ax_fa.xaxis.set_major_locator(MultipleLocator(10))
    ax_fa.xaxis.set_minor_locator(MultipleLocator(5))
    ax_fa.grid(True, which="major", alpha=0.3)
    ax_fa.grid(True, which="minor", alpha=0.12)
    axes_info.append((ax_fa, "fft_avg"))

    return fig, axes_info


# ── FFT helper ────────────────────────────────────────────────────────────────

def _compute_fft(iq_uniform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (harmonics, magnitude) for positive frequencies only."""
    n     = len(iq_uniform)
    fft   = np.fft.rfft(iq_uniform)
    mag   = 2.0 * np.abs(fft) / n
    mag[0] /= 2.0   # DC component is not doubled
    harmonics = np.arange(len(mag), dtype=float)
    return harmonics, mag


# ── Full headless pipeline ────────────────────────────────────────────────────

def run_cogging_analysis(
    log_folder: str | Path,
    drive: str = "main",
) -> dict:
    """
    Load CSV, detect segments, resample to encoder space, run FFT, save PNGs.

    Returns dict with dominant_harmonic_seg1, dominant_harmonic_seg2,
    dominant_harmonic_avg.
    """
    log_folder = Path(log_folder)
    csv_path   = _find_csv(str(log_folder))
    data, header = _load_csv(csv_path)

    vel_col, iq_col, enc_col = _DRIVE_COLS[drive]
    for col in (vel_col, iq_col, enc_col):
        if col not in header:
            raise ValueError(f"Column '{col}' not found. Is this a cogging test log?")

    t_start, t_mid, t_end = _detect_segments(data, header, drive)

    iq_full  = data[:, header.index(iq_col)]
    vel_full = data[:, header.index(vel_col)]
    enc_full = data[:, header.index(enc_col)]
    t_s      = (data[:, header.index("stamp_ns")] - data[0, header.index("stamp_ns")]) * 1e-9

    seg1_enc = enc_full[t_start : t_mid + 1]
    seg1_iq  = iq_full[t_start  : t_mid + 1]
    seg2_enc = enc_full[t_mid   : t_end + 1]
    seg2_iq  = iq_full[t_mid    : t_end + 1]

    common_enc, iq1 = _resample_to_enc_space(seg1_enc, seg1_iq)
    _,          iq2 = _resample_to_enc_space(seg2_enc, seg2_iq)
    iq_avg          = (iq1 + iq2) / 2.0

    harmonics1, fft_seg1 = _compute_fft(iq1)
    harmonics2, fft_seg2 = _compute_fft(iq2)
    harmonics_avg, fft_avg = _compute_fft(iq_avg)

    fig_main, axes_main = make_cogging_figure(
        common_enc, iq1, iq2, iq_avg,
        fft_seg1, fft_seg2, fft_avg,
        harmonics1, t_s, iq_full, vel_full, drive,
        harmonics_avg=harmonics_avg,
    )

    out_dir = _resolve_output_dir(log_folder)

    from matplotlib.backends.backend_agg import FigureCanvasAgg
    for fig, axes_info in [(fig_main, axes_main)]:
        canvas   = FigureCanvasAgg(fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        for ax, stem in axes_info:
            bbox = ax.get_tightbbox(renderer).transformed(fig.dpi_scale_trans.inverted())
            fig.savefig(str(out_dir / f"cogging_{stem}.png"), dpi=150, bbox_inches=bbox)

    dom1 = int(np.argmax(fft_seg1[1:]) + 1)
    dom2 = int(np.argmax(fft_seg2[1:]) + 1)
    doma = int(np.argmax(fft_avg[1:])  + 1)

    lines = [
        f"dominant_harmonic_seg1 = {dom1}",
        f"dominant_harmonic_seg2 = {dom2}",
        f"dominant_harmonic_avg  = {doma}",
    ]
    (out_dir / "cogging_values.txt").write_text("\n".join(lines) + "\n")

    return {
        "dominant_harmonic_seg1": dom1,
        "dominant_harmonic_seg2": dom2,
        "dominant_harmonic_avg":  doma,
    }
