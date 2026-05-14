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
from scipy.signal import butter, filtfilt

TWO_PI = 2.0 * np.pi
EXTERNAL_ENCODER_BITS = 25
ACTUATOR_ENCODER_BITS = 20
DEFAULT_LUT_SIZE = 2048
LUT_SIZE_CHOICES = (64, 128, 256, 512, 1024, 2048, 4096, 8192)
_VEL_THRESHOLD_FRAC = 0.05
BUTTER_ORDER = 4
BUTTER_CUTOFF_CPR = 500.0       # default low-pass cutoff in cycles per revolution
RESAMPLE_PTS_PER_REV = 1 << 20  # 2^20 uniform spatial points per revolution

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
class OutputSideAnalysis:
    """Output-side encoder analysis: continuous-signal pipeline with Butterworth filtering."""

    # Per-sample scatter data (all revolution segments concatenated, per-segment DC-aligned)
    x_rad: np.ndarray           # ext encoder wrapped to [0, 2π] — x-axis for plots
    delta_rad: np.ndarray       # filtered delta (used as `delta_rad` in main figure)
    delta_raw: np.ndarray       # unfiltered delta (same alignment)

    # Statistics
    delta_raw_min: float
    delta_raw_max: float
    delta_filtered_min: float
    delta_filtered_max: float
    delta_raw_rss: float        # sqrt(sum(delta^2))
    delta_filtered_rss: float

    # Spatial average (RESAMPLE_PTS_PER_REV uniform points)
    average_delta_phase: np.ndarray
    average_delta: np.ndarray

    # LUT
    lut_phase_rad: np.ndarray
    lut_delta_rad: np.ndarray
    harmonics: np.ndarray
    fft_mag_rad: np.ndarray

    # Metadata
    revolution_count: int

    # --- Duck-type interface matching EncoderAnalysisSeries for the main figure ---
    label: str = "Output side"
    x_label: str = "External encoder angle (rad)"
    y_label: str = "Actuator output encoder angle (rad)"

    @property
    def y_rad(self) -> np.ndarray:
        return _wrap_rad(self.x_rad - self.delta_rad)

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
    output_side: OutputSideAnalysis
    input_segments: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=list)
    output_segments: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=list)
    output_revolution_count: int = 0


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
      4. Scale ext by an empirical gear ratio (input_span / ext_span) so ext covers
         exactly the same angular span as input, eliminating linear LUT drift from
         any inaccuracy in the nominal gear_ratio column value.
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
        if abs(input_span) < _MIN_INPUT_REV_SPAN_RAD:
            continue
        if abs(ext_span) > 1e-9 and np.sign(ext_span) != np.sign(input_span):
            ext_unwrapped = -ext_unwrapped
            ext_span = -ext_span

        effective_gear = input_span / ext_span if abs(ext_span) > 1e-9 else gear_ratio
        ext_at_zero = _interp_or_extrap(input_unwrapped, ext_unwrapped, 0.0)
        ext_scaled  = (ext_unwrapped - ext_at_zero) * effective_gear

        segments.append((_wrap_rad(input_unwrapped), _wrap_rad(ext_scaled)))
    return segments


def _process_output_segments(
    ext_phase: np.ndarray,
    out_phase: np.ndarray,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """
    Return [(ext_seg, out_seg), ...] for each complete EL5032 revolution.

    Segments are bounded by EL5032 wrap events (EL5032 is on the output shaft, 1:1).
    For each segment: unwrap both signals, flip EL5032 if it opposes the output encoder,
    then zero-align the output encoder at ext=0.
    """
    wrap_idx   = np.where(np.abs(np.diff(ext_phase)) > np.pi)[0] + 1
    boundaries = np.concatenate(([0], wrap_idx, [len(ext_phase)]))

    segments: list[tuple[np.ndarray, np.ndarray]] = []
    for start, end in zip(boundaries[1:-1], boundaries[2:-1]):
        ext_seg = ext_phase[start:end]
        out_seg = out_phase[start:end]
        if len(ext_seg) < _MIN_INPUT_SEGMENT_SAMPLES:
            continue

        ext_unwrapped = np.unwrap(ext_seg)
        out_unwrapped = np.unwrap(out_seg)

        ext_span = float(ext_unwrapped[-1] - ext_unwrapped[0])
        out_span = float(out_unwrapped[-1] - out_unwrapped[0])
        if abs(ext_span) < _MIN_INPUT_REV_SPAN_RAD:
            continue
        if abs(out_span) > 1e-9 and np.sign(ext_span) != np.sign(out_span):
            ext_unwrapped = -ext_unwrapped

        out_at_zero = _interp_or_extrap(ext_unwrapped, out_unwrapped, 0.0)
        segments.append((_wrap_rad(ext_unwrapped), _wrap_rad(out_unwrapped - out_at_zero)))
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


def _make_ext_continuous(ext_rad_wrapped: np.ndarray) -> np.ndarray:
    """Unwrap wrapped [0, 2π) external encoder to a monotonic continuous signal."""
    return np.unwrap(ext_rad_wrapped)


def _butter_filtfilt_spatial(
    delta: np.ndarray,
    ext_continuous: np.ndarray,
    cutoff_cpr: float,
    order: int,
) -> np.ndarray:
    """Zero-phase Butterworth low-pass filter on delta, with cutoff in cycles/revolution."""
    total_revs = abs(ext_continuous[-1] - ext_continuous[0]) / TWO_PI
    if total_revs < 0.5:
        return delta.copy()
    n_samples_per_rev = len(ext_continuous) / total_revs
    wn = cutoff_cpr / (n_samples_per_rev / 2.0)
    wn = float(np.clip(wn, 1e-6, 1.0 - 1e-6))
    b, a = butter(order, wn, btype="low")
    padlen = min(3 * max(len(a), len(b)), len(delta) - 1)
    return filtfilt(b, a, delta, padlen=padlen)


def _resection_output(
    ext_continuous: np.ndarray,
    delta_raw: np.ndarray,
    delta_filtered: np.ndarray,
    n_pts: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray]]:
    """
    Split both delta signals into per-revolution segments, align each by mean-DC removal,
    and spatially bin the filtered delta into n_pts uniform bins in [0, 2π].

    Uses min/max of ext_continuous (not start/end) so bidirectional tests — where the
    encoder returns to its starting angle — are handled correctly.
    Spatial binning (not np.interp) handles non-monotonic data within each window.

    Returns (ext_wrapped_all, raw_aligned_all, filt_aligned_all, resampled_filt_segs).
    """
    k_min = int(np.floor(np.min(ext_continuous) / TWO_PI))
    k_max = int(np.floor(np.max(ext_continuous) / TWO_PI))

    uniform_phase = np.linspace(0.0, TWO_PI, n_pts, endpoint=False)
    ext_wrap_list: list[np.ndarray] = []
    raw_list:      list[np.ndarray] = []
    filt_list:     list[np.ndarray] = []
    resampled:     list[np.ndarray] = []

    for k in range(k_min, k_max):
        lo = k * TWO_PI
        hi = (k + 1) * TWO_PI
        mask = (ext_continuous >= lo) & (ext_continuous < hi)
        if np.count_nonzero(mask) < _MIN_INPUT_SEGMENT_SAMPLES:
            continue
        ext_norm = ext_continuous[mask] - lo   # [0, 2π)
        raw_seg  = delta_raw[mask]
        filt_seg = delta_filtered[mask]

        # Per-segment DC alignment: subtract the segment mean
        raw_aligned  = raw_seg  - float(np.mean(raw_seg))
        filt_aligned = filt_seg - float(np.mean(filt_seg))

        ext_wrap_list.append(ext_norm)
        raw_list.append(raw_aligned)
        filt_list.append(filt_aligned)

        # Spatial binning: circular mean per bin — handles non-monotonic ext_norm
        # (bidirectional data hits the same angle twice per window)
        bin_idx = np.floor(ext_norm * n_pts / TWO_PI).astype(int) % n_pts
        sin_sum = np.zeros(n_pts, dtype=float)
        cos_sum = np.zeros(n_pts, dtype=float)
        counts  = np.zeros(n_pts, dtype=int)
        np.add.at(sin_sum, bin_idx, np.sin(filt_aligned))
        np.add.at(cos_sum, bin_idx, np.cos(filt_aligned))
        np.add.at(counts,  bin_idx, 1)

        valid = counts > 0
        c = np.where(counts > 0, counts, 1)
        seg_mean = np.where(valid,
                            np.arctan2(sin_sum / c, cos_sum / c),
                            0.0)
        if not np.all(valid):
            seg_mean = _fill_empty_bins(uniform_phase, seg_mean, valid)

        resampled.append(seg_mean)

    if not ext_wrap_list:
        raise ValueError("No complete output-side revolutions found in the active data.")

    return (
        np.concatenate(ext_wrap_list),
        np.concatenate(raw_list),
        np.concatenate(filt_list),
        resampled,
    )


def _circular_average(segs: list[np.ndarray], n_pts: int) -> np.ndarray:
    """Circular mean of a list of equal-length arrays."""
    sin_sum = np.zeros(n_pts, dtype=float)
    cos_sum = np.zeros(n_pts, dtype=float)
    for seg in segs:
        sin_sum += np.sin(seg)
        cos_sum += np.cos(seg)
    return np.arctan2(sin_sum, cos_sum)


def _analyze_output_side(
    ext_rad_wrapped: np.ndarray,
    output_accum_rad: np.ndarray,
    lut_size: int,
    butter_order: int = BUTTER_ORDER,
    butter_cutoff_cpr: float = BUTTER_CUTOFF_CPR,
) -> OutputSideAnalysis:
    """
    New output-side pipeline:
      1. Make ext encoder continuous via unwrap.
      2. Direction-align with the output actuator encoder.
      3. Compute raw delta = ext_continuous - output_accum.
      4. Apply zero-phase Butterworth filter → filtered delta.
      5. Re-section into per-revolution segments, DC-align each, resample to 2^20 pts.
      6. Circular average across segments → average_delta.
      7. Resample average_delta to lut_size → LUT.
    """
    ext_continuous = _make_ext_continuous(ext_rad_wrapped)
    out_continuous = np.unwrap(output_accum_rad)

    # Direction alignment: use peak excursion direction (robust for bidirectional tests
    # where start ≈ end and span ≈ 0)
    ext_fwd = float(np.max(ext_continuous) - ext_continuous[0])
    ext_bwd = float(ext_continuous[0] - np.min(ext_continuous))
    ext_dir = 1 if ext_fwd >= ext_bwd else -1
    out_fwd = float(np.max(out_continuous) - out_continuous[0])
    out_bwd = float(out_continuous[0] - np.min(out_continuous))
    out_dir = 1 if out_fwd >= out_bwd else -1
    if ext_dir != out_dir:
        ext_continuous = -ext_continuous

    delta_raw_cont     = ext_continuous - out_continuous
    delta_filt_cont    = _butter_filtfilt_spatial(
        delta_raw_cont, ext_continuous, butter_cutoff_cpr, butter_order
    )

    ext_wrap_all, raw_aligned, filt_aligned, resampled_segs = _resection_output(
        ext_continuous, delta_raw_cont, delta_filt_cont, RESAMPLE_PTS_PER_REV
    )

    # Statistics
    delta_raw_min  = float(np.min(raw_aligned))
    delta_raw_max  = float(np.max(raw_aligned))
    delta_filt_min = float(np.min(filt_aligned))
    delta_filt_max = float(np.max(filt_aligned))
    delta_raw_rss  = float(np.sqrt(np.sum(raw_aligned ** 2)))
    delta_filt_rss = float(np.sqrt(np.sum(filt_aligned ** 2)))

    # Spatial average
    avg_phase  = np.linspace(0.0, TWO_PI, RESAMPLE_PTS_PER_REV, endpoint=False)
    avg_delta  = _circular_average(resampled_segs, RESAMPLE_PTS_PER_REV)

    # LUT via uniform decimation of the spatial average
    lut_phase = np.linspace(0.0, TWO_PI, lut_size, endpoint=False)
    lut_delta = np.interp(lut_phase, avg_phase, avg_delta, period=TWO_PI)

    harmonics, fft_mag = _compute_fft(lut_delta)

    return OutputSideAnalysis(
        x_rad=ext_wrap_all,
        delta_rad=filt_aligned,
        delta_raw=raw_aligned,
        delta_raw_min=delta_raw_min,
        delta_raw_max=delta_raw_max,
        delta_filtered_min=delta_filt_min,
        delta_filtered_max=delta_filt_max,
        delta_raw_rss=delta_raw_rss,
        delta_filtered_rss=delta_filt_rss,
        average_delta_phase=avg_phase,
        average_delta=avg_delta,
        lut_phase_rad=lut_phase,
        lut_delta_rad=lut_delta,
        harmonics=harmonics,
        fft_mag_rad=fft_mag,
        revolution_count=len(resampled_segs),
    )


def analyze_encoder_linearization(
    log_folder: str | Path,
    drive: str = "main",
    lut_size: int = DEFAULT_LUT_SIZE,
    butter_order: int = BUTTER_ORDER,
    butter_cutoff_cpr: float = BUTTER_CUTOFF_CPR,
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
    output_side = _analyze_output_side(
        ext_rad[active],
        output_accum_rad[active],
        lut_size=lut_size,
        butter_order=butter_order,
        butter_cutoff_cpr=butter_cutoff_cpr,
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
        output_segments=[],
        output_revolution_count=output_side.revolution_count,
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


def _make_segments_figure(
    segments: list[tuple[np.ndarray, np.ndarray]],
    x_label: str,
    y_label: str,
    title: str,
) -> Figure:
    import math
    n = len(segments)
    if n == 0:
        raise ValueError("No segments to plot.")
    ncols = math.ceil(math.sqrt(n))
    nrows = math.ceil(n / ncols)
    fig = Figure(figsize=(4 * ncols, 4 * nrows), tight_layout=True)
    for i, (xs, ys) in enumerate(segments, start=1):
        ax = fig.add_subplot(nrows, ncols, i)
        xd, yd = _decimated_xy(xs, ys)
        ax.scatter(xd, yd, s=3, alpha=0.35, linewidths=0)
        ax.plot([0.0, TWO_PI], [0.0, TWO_PI], color="black", linewidth=0.8, alpha=0.55)
        ax.set_title(f"Seg {i}/{n}", fontsize=9)
        ax.set_xlim(0.0, TWO_PI)
        ax.set_ylim(0.0, TWO_PI)
        ax.set_xlabel(x_label, fontsize=7)
        ax.set_ylabel(y_label, fontsize=7)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)
    fig.suptitle(title, fontsize=11)
    return fig


def make_segment_debug_figure(result: EncoderLinearizationResult) -> Figure:
    """Plot each input-revolution segment individually for debug inspection."""
    return _make_segments_figure(
        result.input_segments,
        x_label="Input enc (rad)",
        y_label="Ext enc scaled (rad)",
        title=(
            f"Input segments debug — {result.drive} "
            f"(gear {result.gear_ratio:.4g}, {result.input_revolution_count} complete revs)"
        ),
    )


def make_output_segment_debug_figure(result: EncoderLinearizationResult) -> Figure:
    """Plot each output-revolution segment individually for debug inspection."""
    return _make_segments_figure(
        result.output_segments,
        x_label="External encoder angle (rad)",
        y_label="Actuator output encoder angle (rad)",
        title=(
            f"Output segments debug — {result.drive} "
            f"({result.output_revolution_count} complete revs)"
        ),
    )


def make_output_analysis_figure(result: EncoderLinearizationResult):
    """
    Interactive output-side analysis figure (use with matplotlib.pyplot.show()).

    Single axis showing:
      - scatter: raw delta vs ext angle (all segments, decimated)
      - scatter: filtered delta vs ext angle (all segments, decimated)
      - 4 horizontal dashed lines: min/max of raw and filtered delta
      - 2 horizontal dotted lines: RSS of raw and filtered delta
      - line: average_delta (decimated)
      - line: LUT
    """
    import matplotlib.pyplot as plt

    out = result.output_side
    lut_size = result.lut_size

    fig, ax = plt.subplots(figsize=(14, 7))

    # Decimate scatter to at most 50k pts each
    x_raw, d_raw   = _decimated_xy(out.x_rad, out.delta_raw, max_points=50_000)
    x_filt, d_filt = _decimated_xy(out.x_rad, out.delta_rad, max_points=50_000)

    ax.scatter(x_raw,  d_raw,  s=2, alpha=0.15, linewidths=0,
               color="grey",      label="raw delta")
    ax.scatter(x_filt, d_filt, s=2, alpha=0.20, linewidths=0,
               color="steelblue", label="filtered delta")

    # 4 horizontal dashed lines: raw & filtered min/max
    ax.axhline(out.delta_raw_min,      color="grey",      linestyle="--", linewidth=1.0,
               label=f"raw min {out.delta_raw_min:.5f}")
    ax.axhline(out.delta_raw_max,      color="grey",      linestyle="--", linewidth=1.0,
               label=f"raw max {out.delta_raw_max:.5f}")
    ax.axhline(out.delta_filtered_min, color="steelblue", linestyle="--", linewidth=1.0,
               label=f"filt min {out.delta_filtered_min:.5f}")
    ax.axhline(out.delta_filtered_max, color="steelblue", linestyle="--", linewidth=1.0,
               label=f"filt max {out.delta_filtered_max:.5f}")

    # 2 horizontal dotted lines: RSS
    ax.axhline(out.delta_raw_rss,      color="grey",      linestyle=":",  linewidth=1.2,
               label=f"raw RSS {out.delta_raw_rss:.5f}")
    ax.axhline(out.delta_filtered_rss, color="steelblue", linestyle=":",  linewidth=1.2,
               label=f"filt RSS {out.delta_filtered_rss:.5f}")

    # Average delta (decimated from 2^20 to ≤25k pts)
    avg_x, avg_y = _decimated_xy(out.average_delta_phase, out.average_delta, max_points=25_000)
    ax.plot(avg_x, avg_y, color="darkorange", linewidth=1.4, label="avg delta")

    # LUT
    ax.plot(out.lut_phase_rad, out.lut_delta_rad, color="tab:red", linewidth=1.6,
            label=f"LUT ({lut_size})")

    raw_pp   = out.delta_raw_max   - out.delta_raw_min
    filt_pp  = out.delta_filtered_max - out.delta_filtered_min
    ax.set_title(
        f"Output side — {result.drive}  |  "
        f"raw p-p {raw_pp:.5f} rad  filtered p-p {filt_pp:.5f} rad  |  "
        f"raw RSS {out.delta_raw_rss:.5f}  filt RSS {out.delta_filtered_rss:.5f}  |  "
        f"{result.output_revolution_count} revs"
    )
    ax.set_xlabel("External encoder angle (rad)")
    ax.set_ylabel("Delta (rad)")
    ax.set_xlim(0.0, TWO_PI)
    ax.xaxis.set_major_locator(MultipleLocator(np.pi / 2))
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
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
        _draw_xy_axis(ax_xy, series, show_fit=True)
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
        f"output_revolution_count = {result.output_revolution_count}",
        f"lut_size = {result.lut_size}",
        f"input_rms_rad = {result.input_side.rms_rad:.9g}",
        f"input_peak_to_peak_rad = {result.input_side.peak_to_peak_rad:.9g}",
        f"input_dominant_harmonic = {result.input_side.dominant_harmonic}",
        f"output_rms_rad = {result.output_side.rms_rad:.9g}",
        f"output_peak_to_peak_rad = {result.output_side.peak_to_peak_rad:.9g}",
        f"output_dominant_harmonic = {result.output_side.dominant_harmonic}",
        f"output_raw_peak_to_peak_rad = {result.output_side.delta_raw_max - result.output_side.delta_raw_min:.9g}",
        f"output_filtered_peak_to_peak_rad = {result.output_side.delta_filtered_max - result.output_side.delta_filtered_min:.9g}",
        f"output_raw_rss_rad = {result.output_side.delta_raw_rss:.9g}",
        f"output_filtered_rss_rad = {result.output_side.delta_filtered_rss:.9g}",
    ]


def run_encoder_linearization_analysis(
    log_folder: str | Path,
    drive: str = "main",
    lut_size: int = DEFAULT_LUT_SIZE,
    debug_segments: bool = False,
    debug_output_segments: bool = False,
    butter_order: int = BUTTER_ORDER,
    butter_cutoff_cpr: float = BUTTER_CUTOFF_CPR,
) -> dict:
    """
    Load CSV, compute input/output encoder LUTs, save plots, LUT CSV, and summary.

    When debug_segments=True: save a per-segment scatter figure instead of the
    normal analysis plots; skip LUT CSV and summary.
    When debug_output_segments=True: same but for output-side segments.
    """
    log_folder = Path(log_folder)
    result = analyze_encoder_linearization(
        log_folder, drive=drive, lut_size=lut_size,
        butter_order=butter_order, butter_cutoff_cpr=butter_cutoff_cpr,
    )
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

    if debug_output_segments:
        fig = make_output_segment_debug_figure(result)
        canvas = FigureCanvasAgg(fig)
        canvas.draw()
        debug_path = out_dir / "encoder_linearization_output_segments_debug.png"
        fig.savefig(str(debug_path), dpi=150)
        return {
            "debug_figure_path": str(debug_path),
            "output_revolution_count": result.output_revolution_count,
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
        "output_raw_peak_to_peak_rad": result.output_side.delta_raw_max - result.output_side.delta_raw_min,
        "output_filtered_peak_to_peak_rad": result.output_side.delta_filtered_max - result.output_side.delta_filtered_min,
        "output_raw_rss_rad": result.output_side.delta_raw_rss,
        "output_filtered_rss_rad": result.output_side.delta_filtered_rss,
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
    parser.add_argument("--debug-output-segments", action="store_true",
                        help="Plot each output revolution segment individually; skip normal analysis.")
    parser.add_argument("--show-output-plot", action="store_true",
                        help="Show interactive output-side analysis plot (no PNG saved).")
    parser.add_argument("--butter-order", type=int, default=BUTTER_ORDER,
                        help="Butterworth filter order (default %(default)s).")
    parser.add_argument("--butter-cutoff-cpr", type=float, default=BUTTER_CUTOFF_CPR,
                        help="Butterworth low-pass cutoff in cycles/revolution (default %(default)s).")
    args = parser.parse_args()

    folder = _latest_log(args.path) if args.latest else Path(args.path)

    if args.show_output_plot:
        import matplotlib.pyplot as plt
        enc_result = analyze_encoder_linearization(
            folder,
            drive=args.drive,
            lut_size=args.lut_size,
            butter_order=args.butter_order,
            butter_cutoff_cpr=args.butter_cutoff_cpr,
        )
        make_output_analysis_figure(enc_result)
        plt.show()
        return

    output = run_encoder_linearization_analysis(
        folder,
        drive=args.drive,
        lut_size=args.lut_size,
        debug_segments=args.debug_segments,
        debug_output_segments=args.debug_output_segments,
        butter_order=args.butter_order,
        butter_cutoff_cpr=args.butter_cutoff_cpr,
    )
    for key, value in output.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    _main()
