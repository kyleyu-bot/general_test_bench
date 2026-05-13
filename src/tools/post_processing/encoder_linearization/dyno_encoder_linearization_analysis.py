"""
Pure analysis module for encoder linearization test logs.

No GUI dependencies - safe to import from test scripts or headless pipelines.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import argparse
import re

import numpy as np
import pandas as pd
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator

TWO_PI = 2.0 * np.pi
EXTERNAL_ENCODER_BITS = 25
ACTUATOR_ENCODER_BITS = 20
DEFAULT_LUT_SIZE = 2048
LUT_SIZE_CHOICES = (64, 128, 256, 512, 1024, 2048, 4096, 8192)
_VEL_THRESHOLD_FRAC = 0.05

_DRIVE_COLS = {
    "main": {
        "velocity": "main_rx_target_velocity",
        "enable": "main_rx_enable",
        "input": "main_tx_input_enc_pos",
        "output": "main_tx_output_enc_pos",
        "gear": "main_gear_ratio",
    },
    "dut": {
        "velocity": "dut_rx_target_velocity",
        "enable": "dut_rx_enable",
        "input": "dut_tx_input_enc_pos",
        "output": "dut_tx_output_enc_pos",
        "gear": "dut_gear_ratio",
    },
}

_MIN_INPUT_REV_SPAN_RAD = 0.75 * TWO_PI
_MIN_INPUT_SEGMENT_SAMPLES = 16


@dataclass
class EncoderAnalysisSeries:
    label: str
    x_label: str
    y_label: str
    x_rad: np.ndarray
    y_rad: np.ndarray
    delta_rad: np.ndarray
    lut_phase_rad: np.ndarray
    lut_delta_rad: np.ndarray
    harmonics: np.ndarray
    fft_mag_rad: np.ndarray

    @property
    def rms_rad(self) -> float:
        return float(np.sqrt(np.mean(self.delta_rad ** 2)))

    @property
    def peak_to_peak_rad(self) -> float:
        return float(np.max(self.delta_rad) - np.min(self.delta_rad))

    @property
    def dominant_harmonic(self) -> int:
        if len(self.fft_mag_rad) <= 1:
            return 0
        return int(np.argmax(self.fft_mag_rad[1:]) + 1)


@dataclass
class EncoderLinearizationResult:
    csv_path: Path
    drive: str
    gear_ratio: float
    lut_size: int
    sample_count: int
    input_revolution_count: int
    input_side: EncoderAnalysisSeries
    output_side: EncoderAnalysisSeries
    input_segments: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=list)


# -- CSV helpers --------------------------------------------------------------

def _find_csv(folder: str | Path) -> Path:
    p = Path(folder)
    for name in ("dyno_pdo.csv.gz", "dyno_pdo.csv"):
        if (p / name).exists():
            return p / name
    if p.suffix in (".csv", ".gz") and p.exists():
        return p
    raise FileNotFoundError(f"No dyno_pdo.csv(.gz) found in {folder}")


def _latest_log(root: str | Path = "test_data_log") -> Path:
    base = Path(root)
    csvs = list(base.glob("*/*/dyno_pdo.csv.gz")) + list(base.glob("*/*/dyno_pdo.csv"))
    if not csvs:
        raise FileNotFoundError(f"No dyno_pdo.csv(.gz) found under {root}")
    return max(csvs, key=lambda p: p.stat().st_mtime).parent


def _load_csv(csv_path: Path) -> tuple[np.ndarray, list[str]]:
    df_raw = pd.read_csv(csv_path)
    header = list(df_raw.columns)
    data = df_raw.to_numpy(dtype=float)
    return data, header


# -- Output directory ---------------------------------------------------------

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
        hhmmss = log_folder.name.split("_")[0]
        out = repo_root / "actuator_test_log" / sn / f"{hhmmss}_encoder_linearization"
    else:
        out = log_folder
    out.mkdir(parents=True, exist_ok=True)
    return out


# -- Encoder math -------------------------------------------------------------

def _wrap_rad(rad: np.ndarray | float) -> np.ndarray | float:
    return np.mod(rad, TWO_PI)


def _signed_angle(rad: np.ndarray | float) -> np.ndarray | float:
    return np.arctan2(np.sin(rad), np.cos(rad))


def _counts_to_rad(counts: np.ndarray, bits: int) -> np.ndarray:
    return np.asarray(counts, dtype=float) * TWO_PI / float(1 << bits)


def _validate_lut_size(lut_size: int) -> int:
    lut_size = int(lut_size)
    if lut_size < 8:
        raise ValueError("LUT size must be at least 8.")
    return lut_size


def _require_columns(header: list[str], drive: str) -> None:
    if drive not in _DRIVE_COLS:
        raise ValueError(f"Unknown drive '{drive}'. Expected one of {sorted(_DRIVE_COLS)}.")
    cols = _DRIVE_COLS[drive]
    required = ["encoder_count", cols["velocity"], cols["input"], cols["output"]]
    missing = [col for col in required if col not in header]
    if missing:
        raise ValueError(f"Missing required column(s): {', '.join(missing)}")


def _resolve_drive(data: np.ndarray, header: list[str], drive: str) -> str:
    if drive != "auto":
        return drive

    candidates: list[tuple[float, str]] = []
    for candidate, cols in _DRIVE_COLS.items():
        required = ["encoder_count", cols["velocity"], cols["input"], cols["output"]]
        if any(col not in header for col in required):
            continue
        velocity = data[:, header.index(cols["velocity"])]
        peak = float(np.nanmax(np.abs(velocity)))
        if np.isfinite(peak):
            candidates.append((peak, candidate))

    if not candidates:
        raise ValueError("No drive columns found for encoder linearization analysis.")

    peak, selected = max(candidates, key=lambda item: item[0])
    if peak <= 0.0:
        raise ValueError("No nonzero velocity command found on main or dut.")
    return selected


def _active_motion_mask(data: np.ndarray, header: list[str], drive: str) -> np.ndarray:
    cols = _DRIVE_COLS[drive]
    vel = data[:, header.index(cols["velocity"])]
    peak = float(np.nanmax(np.abs(vel)))
    if peak <= 0.0 or not np.isfinite(peak):
        raise ValueError(
            f"No nonzero velocity command found for '{drive}'. "
            "Choose the driven side, or use drive='auto'."
        )
    mask = np.abs(vel) > (_VEL_THRESHOLD_FRAC * peak)

    enable_col = cols["enable"]
    if enable_col in header:
        enabled = data[:, header.index(enable_col)] > 0.5
        if np.any(enabled):
            mask &= enabled

    if int(np.count_nonzero(mask)) < 16:
        raise ValueError("Not enough active motion samples found in this log.")
    return mask


def _gear_ratio(data: np.ndarray, header: list[str], drive: str, mask: np.ndarray) -> float:
    gear_col = _DRIVE_COLS[drive]["gear"]
    if gear_col not in header:
        return 1.0
    values = data[mask, header.index(gear_col)]
    values = values[np.isfinite(values)]
    if values.size == 0:
        return 1.0
    ratio = float(np.nanmedian(values))
    if not np.isfinite(ratio) or abs(ratio) < 1e-9:
        return 1.0
    return ratio


def _fill_empty_bins(
    phase: np.ndarray,
    values: np.ndarray,
    valid: np.ndarray,
) -> np.ndarray:
    if np.all(valid):
        return _signed_angle(values)
    if not np.any(valid):
        raise ValueError("No valid LUT bins were populated.")

    x_valid = phase[valid]
    y_valid = np.unwrap(values[valid])
    if len(x_valid) == 1:
        return np.full_like(phase, _signed_angle(y_valid[0]), dtype=float)

    x_ext = np.concatenate(([x_valid[-1] - TWO_PI], x_valid, [x_valid[0] + TWO_PI]))
    y_ext = np.concatenate(([y_valid[-1]], y_valid, [y_valid[0]]))
    return _signed_angle(np.interp(phase, x_ext, y_ext))


def _resample_delta_lut(
    x_rad: np.ndarray,
    delta_rad: np.ndarray,
    lut_size: int,
) -> tuple[np.ndarray, np.ndarray]:
    lut_size = _validate_lut_size(lut_size)
    x = _wrap_rad(x_rad)
    delta = _signed_angle(delta_rad)

    bin_idx = np.floor(x * lut_size / TWO_PI).astype(int) % lut_size
    sin_sum = np.zeros(lut_size, dtype=float)
    cos_sum = np.zeros(lut_size, dtype=float)
    counts = np.zeros(lut_size, dtype=int)
    np.add.at(sin_sum, bin_idx, np.sin(delta))
    np.add.at(cos_sum, bin_idx, np.cos(delta))
    np.add.at(counts, bin_idx, 1)

    phase = (np.arange(lut_size, dtype=float) + 0.5) * TWO_PI / lut_size
    lut = np.zeros(lut_size, dtype=float)
    valid = counts > 0
    lut[valid] = np.arctan2(sin_sum[valid] / counts[valid], cos_sum[valid] / counts[valid])
    lut = _fill_empty_bins(phase, lut, valid)
    return phase, lut


def _compute_fft(signal_uniform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centered = signal_uniform - np.mean(signal_uniform)
    n = len(centered)
    fft = np.fft.rfft(centered)
    mag = 2.0 * np.abs(fft) / n
    if len(mag):
        mag[0] /= 2.0
    harmonics = np.arange(len(mag), dtype=float)
    return harmonics, mag


def _interp_or_extrap(x: np.ndarray, y: np.ndarray, target: float) -> float:
    """Linear interpolate/extrapolate y at target, tolerating duplicate x samples."""
    order = np.argsort(x)
    x_sorted = x[order]
    y_sorted = y[order]
    x_unique, inverse = np.unique(x_sorted, return_inverse=True)
    if len(x_unique) == 0:
        raise ValueError("Cannot interpolate an empty segment.")
    y_unique = np.bincount(inverse, weights=y_sorted) / np.bincount(inverse)
    if len(x_unique) == 1:
        return float(y_unique[0])

    if target <= x_unique[0]:
        x0, x1 = x_unique[0], x_unique[1]
        y0, y1 = y_unique[0], y_unique[1]
    elif target >= x_unique[-1]:
        x0, x1 = x_unique[-2], x_unique[-1]
        y0, y1 = y_unique[-2], y_unique[-1]
    else:
        return float(np.interp(target, x_unique, y_unique))

    if abs(x1 - x0) < 1e-12:
        return float(y0)
    return float(y0 + (target - x0) * (y1 - y0) / (x1 - x0))


def _nonzero_sign(values: np.ndarray) -> np.ndarray:
    signs = np.sign(values)
    if len(signs) == 0:
        return signs
    last = 0.0
    for i, val in enumerate(signs):
        if val == 0.0:
            signs[i] = last
        else:
            last = val
    last = 0.0
    for i in range(len(signs) - 1, -1, -1):
        if signs[i] == 0.0:
            signs[i] = last
        else:
            last = signs[i]
    return signs


def _process_input_segments(
    input_phase: np.ndarray,
    ext_phase: np.ndarray,
    gear_ratio: float = 1.0,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """
    Return [(input_seg, ext_processed), ...] for each complete revolution.

    Revolution boundaries are indices where the input encoder wraps (|diff| > π).
    Only segments between consecutive wrap events are included (complete revolutions).

    For each segment:
      1. np.unwrap both signals to remove wrap-arounds within the segment.
      2. Flip ext if its net direction opposes input.
      3. Subtract the ext value interpolated at input=0 so ext=0 when input=0.
      4. Multiply by gear_ratio so the ext span covers [0, 2π] per input revolution.
    """
    wrap_idx   = np.where(np.abs(np.diff(input_phase)) > np.pi)[0] + 1
    boundaries = np.concatenate(([0], wrap_idx, [len(input_phase)]))

    segments: list[tuple[np.ndarray, np.ndarray]] = []
    for start, end in zip(boundaries[1:-1], boundaries[2:-1]):
        input_seg = input_phase[start:end]
        ext_seg   = ext_phase[start:end]
        if len(input_seg) < _MIN_INPUT_SEGMENT_SAMPLES:
            continue

        ext_unwrapped   = np.unwrap(ext_seg)
        input_unwrapped = np.unwrap(input_seg)

        input_span = float(input_unwrapped[-1] - input_unwrapped[0])
        ext_span   = float(ext_unwrapped[-1]   - ext_unwrapped[0])
        if abs(ext_span) > 1e-9 and np.sign(ext_span) != np.sign(input_span):
            ext_unwrapped = -ext_unwrapped

        ext_at_zero = _interp_or_extrap(input_unwrapped, ext_unwrapped, 0.0)
        ext_scaled  = (ext_unwrapped - ext_at_zero) * gear_ratio

        segments.append((_wrap_rad(input_unwrapped), _wrap_rad(ext_scaled)))
    return segments


def _processed_input_side_data(
    input_phase_rad: np.ndarray,
    external_phase_rad: np.ndarray,
    velocity: np.ndarray,
    gear_ratio: float = 1.0,
) -> tuple[np.ndarray, np.ndarray, int, list[tuple[np.ndarray, np.ndarray]]]:
    """Return (input_concat, ext_concat, revolution_count, per_segment_list)."""
    finite = (
        np.isfinite(input_phase_rad) &
        np.isfinite(external_phase_rad) &
        np.isfinite(velocity)
    )
    input_phase = _wrap_rad(input_phase_rad[finite])
    ext_phase   = _wrap_rad(external_phase_rad[finite])

    if len(input_phase) < _MIN_INPUT_SEGMENT_SAMPLES:
        raise ValueError("Not enough valid input-side samples.")

    segments = _process_input_segments(input_phase, ext_phase, gear_ratio)

    if not segments:
        raise ValueError(
            "No complete input-side revolutions were found. "
            "Check that the log contains active motion across encoder wraps."
        )

    input_concat = np.concatenate([s[0] for s in segments])
    ext_concat   = np.concatenate([s[1] for s in segments])
    return input_concat, ext_concat, len(segments), segments


def _analyze_series(
    label: str,
    x_label: str,
    y_label: str,
    x_rad: np.ndarray,
    y_rad: np.ndarray,
    lut_size: int,
) -> EncoderAnalysisSeries:
    finite = np.isfinite(x_rad) & np.isfinite(y_rad)
    x = _wrap_rad(x_rad[finite])
    y = _wrap_rad(y_rad[finite])
    if len(x) < 16:
        raise ValueError(f"Not enough valid samples for {label}.")

    delta = _signed_angle(x - y)
    lut_phase, lut_delta = _resample_delta_lut(x, delta, lut_size)
    harmonics, fft_mag = _compute_fft(lut_delta)
    return EncoderAnalysisSeries(
        label=label,
        x_label=x_label,
        y_label=y_label,
        x_rad=x,
        y_rad=y,
        delta_rad=delta,
        lut_phase_rad=lut_phase,
        lut_delta_rad=lut_delta,
        harmonics=harmonics,
        fft_mag_rad=fft_mag,
    )


def analyze_encoder_linearization(
    log_folder: str | Path,
    drive: str = "main",
    lut_size: int = DEFAULT_LUT_SIZE,
) -> EncoderLinearizationResult:
    """
    Load a dyno_pdo.csv log and compute input/output encoder linearization LUTs.

    The returned LUT columns represent reference minus actuator encoder phase:
    input_lut_rad  = wrap(actuator_input - processed_EL5032)
    output_lut_rad = wrap(EL5032 - wrapped_actuator_output)
    """
    lut_size = _validate_lut_size(lut_size)
    csv_path = _find_csv(log_folder)
    data, header = _load_csv(csv_path)
    drive = _resolve_drive(data, header, drive)
    _require_columns(header, drive)

    cols = _DRIVE_COLS[drive]
    active = _active_motion_mask(data, header, drive)
    gear = _gear_ratio(data, header, drive, active)

    ext_rad = _wrap_rad(_counts_to_rad(data[:, header.index("encoder_count")], EXTERNAL_ENCODER_BITS))
    input_rad = _wrap_rad(_counts_to_rad(data[:, header.index(cols["input"])], ACTUATOR_ENCODER_BITS))
    output_accum_rad = _counts_to_rad(data[:, header.index(cols["output"])], ACTUATOR_ENCODER_BITS)
    output_wrapped_rad = _wrap_rad(output_accum_rad)
    velocity = data[:, header.index(cols["velocity"])]

    input_x_rad, input_y_rad, input_rev_count, input_segs = _processed_input_side_data(
        input_rad[active],
        ext_rad[active],
        velocity[active],
        gear_ratio=gear,
    )

    input_side = _analyze_series(
        "Input side",
        "Processed EL5032 angle per input rev (rad)",
        "Actuator input encoder angle (rad)",
        input_y_rad,
        input_x_rad,
        lut_size,
    )
    output_side = _analyze_series(
        "Output side",
        "EL5032, wrapped (rad)",
        "Actuator output encoder, wrapped (rad)",
        ext_rad[active],
        output_wrapped_rad[active],
        lut_size,
    )

    return EncoderLinearizationResult(
        csv_path=csv_path,
        drive=drive,
        gear_ratio=gear,
        lut_size=lut_size,
        sample_count=int(np.count_nonzero(active)),
        input_revolution_count=input_rev_count,
        input_side=input_side,
        output_side=output_side,
        input_segments=input_segs,
    )


# -- LUT and plot saving ------------------------------------------------------

def make_lut_dataframe(result: EncoderLinearizationResult) -> pd.DataFrame:
    return pd.DataFrame(
        {
            "index": np.arange(result.lut_size, dtype=int),
            "phase_rad": result.input_side.lut_phase_rad,
            "input_lut_rad": result.input_side.lut_delta_rad,
            "output_lut_rad": result.output_side.lut_delta_rad,
        }
    )


def save_lut_csv(
    result: EncoderLinearizationResult,
    out_dir: str | Path,
    filename: str | None = None,
) -> Path:
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    if filename is None:
        filename = f"encoder_linearization_lut_{result.drive}_{result.lut_size}.csv"
    path = out_path / filename
    make_lut_dataframe(result).to_csv(path, index=False)
    return path


def _decimated_xy(x: np.ndarray, y: np.ndarray, max_points: int = 25000) -> tuple[np.ndarray, np.ndarray]:
    step = max(1, len(x) // max_points)
    return x[::step], y[::step]


def _draw_xy_axis(ax, series: EncoderAnalysisSeries, show_fit: bool = False) -> None:
    x, y = _decimated_xy(series.x_rad, series.y_rad)
    ax.scatter(x, y, s=3, alpha=0.25, linewidths=0)
    ax.plot([0.0, TWO_PI], [0.0, TWO_PI], color="black", linewidth=0.8, alpha=0.55, label="ideal")
    if show_fit:
        fx = series.x_rad[np.isfinite(series.x_rad) & np.isfinite(series.y_rad)]
        fy = series.y_rad[np.isfinite(series.x_rad) & np.isfinite(series.y_rad)]
        slope, intercept = np.polyfit(fx, fy, 1)
        fy_hat = slope * fx + intercept
        ss_res = float(np.sum((fy - fy_hat) ** 2))
        ss_tot = float(np.sum((fy - float(np.mean(fy))) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0
        fit_x = np.array([0.0, TWO_PI])
        ax.plot(fit_x, slope * fit_x + intercept, color="tab:red", linewidth=1.2,
                label=f"fit  y={slope:.4f}x{intercept:+.4f}  R²={r2:.6f}")
        ax.legend(fontsize=8)
    ax.set_xlabel(series.x_label)
    ax.set_ylabel(series.y_label)
    ax.set_title(f"{series.label}: encoder angle XY")
    ax.set_xlim(0.0, TWO_PI)
    ax.set_ylim(0.0, TWO_PI)
    ax.grid(True, alpha=0.3)


def _draw_delta_axis(ax, series: EncoderAnalysisSeries) -> None:
    x, d = _decimated_xy(series.x_rad, series.delta_rad)
    ax.scatter(x, d, s=3, alpha=0.18, linewidths=0, label="samples")
    ax.plot(series.lut_phase_rad, series.lut_delta_rad, color="tab:red",
            linewidth=1.1, label=f"LUT ({len(series.lut_phase_rad)})")
    ax.set_xlabel(series.x_label)
    ax.set_ylabel("error (rad)")
    ax.set_title(
        f"{series.label}: delta vs reference "
        f"(rms {series.rms_rad:.6f} rad, p-p {series.peak_to_peak_rad:.6f} rad)"
    )
    ax.set_xlim(0.0, TWO_PI)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def _draw_fft_axis(ax, series: EncoderAnalysisSeries) -> None:
    x_max = min(200, max(1, int(series.harmonics[-1])))
    visible = series.harmonics <= x_max
    ax.bar(series.harmonics[visible], series.fft_mag_rad[visible],
           width=0.85, align="center")
    ax.set_xlabel("Harmonic (cycles/rev)")
    ax.set_ylabel("|FFT| of delta (rad)")
    ax.set_title(f"{series.label}: spatial FFT (dominant {series.dominant_harmonic})")
    ax.set_xlim(0, x_max)
    ax.xaxis.set_major_locator(MultipleLocator(10))
    ax.xaxis.set_minor_locator(MultipleLocator(5))
    ax.grid(True, which="major", alpha=0.3)
    ax.grid(True, which="minor", alpha=0.12)


def make_segment_debug_figure(result: EncoderLinearizationResult) -> Figure:
    """Plot each input-revolution segment individually for debug inspection."""
    import math
    segments = result.input_segments
    n = len(segments)
    if n == 0:
        raise ValueError("No segments to plot.")
    ncols = math.ceil(math.sqrt(n))
    nrows = math.ceil(n / ncols)
    fig = Figure(figsize=(4 * ncols, 4 * nrows), tight_layout=True)
    for i, (inp, ext) in enumerate(segments, start=1):
        ax = fig.add_subplot(nrows, ncols, i)
        x, y = _decimated_xy(inp, ext)
        ax.scatter(x, y, s=3, alpha=0.35, linewidths=0)
        ax.plot([0.0, TWO_PI], [0.0, TWO_PI], color="black", linewidth=0.8, alpha=0.55)
        ax.set_title(f"Seg {i}/{n}", fontsize=9)
        ax.set_xlim(0.0, TWO_PI)
        ax.set_ylim(0.0, TWO_PI)
        ax.set_xlabel("Input enc (rad)", fontsize=7)
        ax.set_ylabel("Ext enc scaled (rad)", fontsize=7)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)
    fig.suptitle(
        f"Input segments debug — {result.drive} "
        f"(gear {result.gear_ratio:.4g}, {n} complete revs)",
        fontsize=11,
    )
    return fig


def make_encoder_linearization_figure(
    result: EncoderLinearizationResult,
) -> tuple[Figure, list]:
    fig = Figure(figsize=(14, 13), tight_layout=True)
    axes_info: list[tuple] = []

    series_list = [result.input_side, result.output_side]
    stems = ["input", "output"]

    for col, (series, stem) in enumerate(zip(series_list, stems), start=1):
        ax_xy = fig.add_subplot(3, 2, col)
        _draw_xy_axis(ax_xy, series, show_fit=(stem == "input"))
        axes_info.append((ax_xy, f"{stem}_xy"))

        ax_delta = fig.add_subplot(3, 2, 2 + col)
        _draw_delta_axis(ax_delta, series)
        axes_info.append((ax_delta, f"{stem}_delta_lut"))

        ax_fft = fig.add_subplot(3, 2, 4 + col)
        _draw_fft_axis(ax_fft, series)
        axes_info.append((ax_fft, f"{stem}_spatial_fft"))

    fig.suptitle(
        f"Encoder Linearization - {result.drive} "
        f"(gear ratio {result.gear_ratio:.6g}, samples {result.sample_count}, LUT {result.lut_size})",
        fontsize=13,
    )
    return fig, axes_info


def save_encoder_subplots(
    fig: Figure,
    axes_info: list[tuple],
    out_dir: str | Path,
    dpi: int = 150,
    pad_inches: float = 0.12,
) -> list[Path]:
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    canvas = fig.canvas
    if not hasattr(canvas, "get_renderer"):
        canvas = FigureCanvasAgg(fig)
    canvas.draw()
    renderer = canvas.get_renderer()

    saved_paths: list[Path] = []
    for ax, stem in axes_info:
        bbox = ax.get_tightbbox(renderer).transformed(fig.dpi_scale_trans.inverted())
        bbox = bbox.padded(pad_inches)
        path = out_path / f"encoder_linearization_{stem}.png"
        fig.savefig(str(path), dpi=dpi, bbox_inches=bbox)
        saved_paths.append(path)
    return saved_paths


def _summary_lines(result: EncoderLinearizationResult) -> list[str]:
    return [
        f"drive = {result.drive}",
        f"gear_ratio = {result.gear_ratio:.9g}",
        f"active_sample_count = {result.sample_count}",
        f"input_revolution_count = {result.input_revolution_count}",
        f"lut_size = {result.lut_size}",
        f"input_rms_rad = {result.input_side.rms_rad:.9g}",
        f"input_peak_to_peak_rad = {result.input_side.peak_to_peak_rad:.9g}",
        f"input_dominant_harmonic = {result.input_side.dominant_harmonic}",
        f"output_rms_rad = {result.output_side.rms_rad:.9g}",
        f"output_peak_to_peak_rad = {result.output_side.peak_to_peak_rad:.9g}",
        f"output_dominant_harmonic = {result.output_side.dominant_harmonic}",
    ]


def run_encoder_linearization_analysis(
    log_folder: str | Path,
    drive: str = "main",
    lut_size: int = DEFAULT_LUT_SIZE,
    debug_segments: bool = False,
) -> dict:
    """
    Load CSV, compute input/output encoder LUTs, save plots, LUT CSV, and summary.

    When debug_segments=True: save a per-segment scatter figure instead of the
    normal analysis plots; skip LUT CSV and summary.
    """
    log_folder = Path(log_folder)
    result = analyze_encoder_linearization(log_folder, drive=drive, lut_size=lut_size)
    out_dir = _resolve_output_dir(log_folder)

    if debug_segments:
        fig = make_segment_debug_figure(result)
        canvas = FigureCanvasAgg(fig)
        canvas.draw()
        debug_path = out_dir / "encoder_linearization_segments_debug.png"
        fig.savefig(str(debug_path), dpi=150)
        return {
            "debug_figure_path": str(debug_path),
            "input_revolution_count": result.input_revolution_count,
        }

    fig, axes_info = make_encoder_linearization_figure(result)
    save_encoder_subplots(fig, axes_info, out_dir)
    lut_path = save_lut_csv(result, out_dir)
    summary_path = out_dir / "encoder_linearization_values.txt"
    summary_path.write_text("\n".join(_summary_lines(result)) + "\n")
    return {
        "lut_path": str(lut_path),
        "summary_path": str(summary_path),
        "input_rms_rad": result.input_side.rms_rad,
        "input_peak_to_peak_rad": result.input_side.peak_to_peak_rad,
        "input_dominant_harmonic": result.input_side.dominant_harmonic,
        "output_rms_rad": result.output_side.rms_rad,
        "output_peak_to_peak_rad": result.output_side.peak_to_peak_rad,
        "output_dominant_harmonic": result.output_side.dominant_harmonic,
    }


def _main() -> None:
    parser = argparse.ArgumentParser(description="Analyze encoder linearization dyno logs.")
    parser.add_argument("path", nargs="?", default="test_data_log",
                        help="Run folder, dyno_pdo.csv(.gz), or log root when --latest is used.")
    parser.add_argument("--latest", action="store_true",
                        help="Use the latest dyno_pdo.csv(.gz) under path.")
    parser.add_argument("--drive", choices=["auto", *sorted(_DRIVE_COLS)], default="auto")
    parser.add_argument("--lut-size", type=int, default=DEFAULT_LUT_SIZE)
    parser.add_argument("--debug-segments", action="store_true",
                        help="Plot each input revolution segment individually; skip normal analysis.")
    args = parser.parse_args()

    folder = _latest_log(args.path) if args.latest else Path(args.path)
    result = run_encoder_linearization_analysis(
        folder,
        drive=args.drive,
        lut_size=args.lut_size,
        debug_segments=args.debug_segments,
    )
    for key, value in result.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    _main()
