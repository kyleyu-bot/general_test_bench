#!/usr/bin/env python3
"""Shared Bode-analysis helpers for dyno_pdo.csv logs."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import gzip
import io

import numpy as np


@dataclass(frozen=True)
class BodePreset:
    title: str
    reference: str
    response: str
    units: str
    description: str


PRESETS: dict[str, BodePreset] = {
    "main_position": BodePreset(
        "Main Position Loop",
        "main_rx_target_position",
        "main_tx_output_enc_pos",
        "encoder counts",
        "Main drive command position to measured output encoder position.",
    ),
    "dut_position": BodePreset(
        "DUT Position Loop",
        "dut_rx_target_position",
        "dut_tx_output_enc_pos",
        "encoder counts",
        "DUT command position to measured output encoder position.",
    ),
    "main_velocity": BodePreset(
        "Main Velocity Loop",
        "main_rx_target_velocity",
        "main_tx_motor_velocity",
        "mrev/s",
        "Main command velocity to measured motor velocity.",
    ),
    "dut_velocity": BodePreset(
        "DUT Velocity Loop",
        "dut_rx_target_velocity",
        "dut_tx_motor_velocity",
        "mrev/s",
        "DUT command velocity to measured motor velocity.",
    ),
    "main_current": BodePreset(
        "Main Current Loop",
        "main_tx_iq_command",
        "main_tx_iq_actual",
        "A",
        "Main drive q-axis current command to measured q-axis current.",
    ),
    "dut_current": BodePreset(
        "DUT Current Loop",
        "dut_tx_iq_command",
        "dut_tx_iq_actual",
        "A",
        "DUT q-axis current command to measured q-axis current.",
    ),
    "main_torque_drive": BodePreset(
        "Main Drive Torque Estimate",
        "main_rx_torque_command",
        "main_tx_torque_nm",
        "Nm",
        "Main torque command to drive-estimated torque.",
    ),
    "dut_torque_drive": BodePreset(
        "DUT Drive Torque Estimate",
        "dut_rx_torque_command",
        "dut_tx_torque_nm",
        "Nm",
        "DUT torque command to drive-estimated torque.",
    ),
    "main_torque_ch1": BodePreset(
        "Main Torque Sensor CH1",
        "main_rx_torque_command",
        "torque_ch1_nm",
        "Nm",
        "Main torque command to external torque sensor channel 1.",
    ),
    "dut_torque_ch2": BodePreset(
        "DUT Torque Sensor CH2",
        "dut_rx_torque_command",
        "torque_ch2_nm",
        "Nm",
        "DUT torque command to external torque sensor channel 2.",
    ),
    "main_transmission": BodePreset(
        "Main Transmission",
        "main_tx_input_enc_pos",
        "main_tx_output_enc_pos",
        "encoder counts",
        "Main input encoder motion to output encoder motion.",
    ),
    "dut_transmission": BodePreset(
        "DUT Transmission",
        "dut_tx_input_enc_pos",
        "dut_tx_output_enc_pos",
        "encoder counts",
        "DUT input encoder motion to output encoder motion.",
    ),
}


@dataclass
class BodeResult:
    csv_path: Path
    title: str
    ref_name: str
    resp_name: str
    units: str
    time: np.ndarray
    reference: np.ndarray
    response: np.ndarray
    response_raw: np.ndarray
    chirp_frequency: np.ndarray | None
    fs: float
    nperseg: int
    frequency: np.ndarray
    magnitude_db: np.ndarray
    phase_deg: np.ndarray
    mask: np.ndarray
    f_3db: float | None
    f_90: float | None


def resolve_csv_path(path: str | Path) -> Path:
    p = Path(path).expanduser()
    if p.is_file():
        return p
    if p.is_dir():
        for name in ("dyno_pdo.csv.gz", "dyno_pdo.csv"):
            if (p / name).is_file():
                return p / name
        all_gz  = sorted(p.glob("**/dyno_pdo.csv.gz"))
        all_csv = sorted(p.glob("**/dyno_pdo.csv"))
        candidates = all_gz + all_csv
        if candidates:
            return max(candidates, key=lambda x: x.stat().st_mtime)
    raise FileNotFoundError(f"No dyno_pdo.csv(.gz) found at {p}")


def latest_log(root: str | Path = "test_data_log") -> Path:
    base = Path(root).expanduser()
    all_gz  = list(base.glob("**/dyno_pdo.csv.gz"))
    all_csv = list(base.glob("**/dyno_pdo.csv"))
    matches = all_gz + all_csv
    if not matches:
        raise FileNotFoundError(f"No dyno_pdo.csv(.gz) files found under {root}")
    return max(matches, key=lambda x: x.stat().st_mtime)


def _open_csv(csv_path: Path):
    """Return a text-mode file handle for either .csv or .csv.gz."""
    if csv_path.suffix == ".gz":
        return io.TextIOWrapper(gzip.open(csv_path, "rb"), newline="")
    return csv_path.open(newline="")


def read_csv_header(path: str | Path) -> list[str]:
    csv_path = resolve_csv_path(path)
    with _open_csv(csv_path) as fh:
        return next(csv.reader(fh), [])


def read_csv_columns(path: str | Path) -> dict[str, np.ndarray]:
    csv_path = resolve_csv_path(path)
    with _open_csv(csv_path) as fh:
        reader = csv.DictReader(fh)
        columns = {name: [] for name in reader.fieldnames or []}
        for row in reader:
            for name in columns:
                value = row.get(name, "")
                if value == "":
                    columns[name].append(np.nan)
                else:
                    columns[name].append(float(value))
    return {name: np.asarray(values, dtype=float) for name, values in columns.items()}


def time_from_columns(columns: dict[str, np.ndarray]) -> np.ndarray:
    if "stamp_ns" in columns and columns["stamp_ns"].size:
        t = (columns["stamp_ns"] - columns["stamp_ns"][0]) * 1e-9
    elif "cycle_count" in columns and "period_ns" in columns:
        period_s = np.nanmedian(columns["period_ns"]) * 1e-9
        t = (columns["cycle_count"] - columns["cycle_count"][0]) * period_s
    else:
        raise KeyError("Need stamp_ns, or cycle_count plus period_ns, to build a time vector")
    return np.asarray(t, dtype=float)


def zero_phase_lowpass(signal: np.ndarray, fs: float, cutoff_hz: float, order: int = 2) -> np.ndarray:
    nyq = fs / 2.0
    if cutoff_hz <= 0.0 or cutoff_hz >= nyq:
        return np.asarray(signal, dtype=float)
    freq = np.fft.rfftfreq(len(signal), d=1.0 / fs)
    spectrum = np.fft.rfft(signal)
    # Butterworth-like magnitude mask in the frequency domain. It is symmetric
    # by construction through irfft, so it does not introduce phase delay.
    response = 1.0 / np.sqrt(1.0 + np.power(freq / cutoff_hz, 2 * max(1, order)))
    return np.fft.irfft(spectrum * response, n=len(signal))


def chirp_frequency_profile(
    time: np.ndarray,
    f_start: float | None,
    f_end: float | None,
    duration: float | None,
    kind: str,
) -> np.ndarray | None:
    if f_start is None or f_end is None or duration is None or duration <= 0:
        return None
    t = np.clip(time - time[0], 0.0, duration)
    if kind == "exponential":
        f0 = max(float(f_start), 1e-6)
        f1 = max(float(f_end), f0)
        if f1 == f0:
            freq = np.full_like(t, f0)
        else:
            freq = f0 * np.power(f1 / f0, t / duration)
    else:
        freq = float(f_start) + (float(f_end) - float(f_start)) * t / duration
    return np.where((time - time[0]) <= duration, freq, 0.0)


def _valid_window(time: np.ndarray, arrays: list[np.ndarray], t0: float | None, t1: float | None) -> np.ndarray:
    mask = np.isfinite(time)
    if t0 is not None:
        mask &= time >= t0
    if t1 is not None:
        mask &= time <= t1
    for arr in arrays:
        mask &= np.isfinite(arr)
    return mask


def _crossing_frequency(f: np.ndarray, mag_db: np.ndarray, phase_deg: np.ndarray) -> tuple[float | None, float | None]:
    f_3db = None
    if mag_db.size >= 2:
        win = max(5, len(mag_db) // 30)
        if win % 2 == 0:
            win += 1
        win = min(win, len(mag_db))
        mag_smooth = np.convolve(mag_db, np.ones(win) / win, mode="same")
        peaks = _find_prominent_peaks(mag_smooth, prominence=10.0)
        peak_idx = int(peaks[0]) if peaks.size else int(np.argmax(mag_smooth[: max(2, int(len(mag_smooth) * 0.9))]))
        sustain = max(5, len(mag_db) // 100)
        for j in range(peak_idx, len(mag_db) - 1):
            if mag_db[j] >= -3.0 and mag_db[j + 1] < -3.0:
                following = mag_db[j + 1 : min(j + 1 + sustain, len(mag_db))]
                if following.size and float(np.median(following)) < -3.0:
                    f_3db = f[j] + (f[j + 1] - f[j]) * (-3.0 - mag_db[j]) / (mag_db[j + 1] - mag_db[j])
                    break

    f_90 = None
    if phase_deg.size >= 2:
        start = np.where(f >= 10.0)[0]
        start_idx = int(start[0]) if start.size else 0
        for j in range(start_idx, len(phase_deg) - 1):
            if phase_deg[j] >= -90.0 and phase_deg[j + 1] < -90.0:
                f_90 = f[j] + (f[j + 1] - f[j]) * (-90.0 - phase_deg[j]) / (phase_deg[j + 1] - phase_deg[j])
                break
    return f_3db, f_90


def _find_prominent_peaks(values: np.ndarray, prominence: float) -> np.ndarray:
    if values.size < 3:
        return np.asarray([], dtype=int)
    peaks: list[int] = []
    for i in range(1, len(values) - 1):
        if values[i] <= values[i - 1] or values[i] <= values[i + 1]:
            continue
        left_min = float(np.min(values[: i + 1]))
        right_min = float(np.min(values[i:]))
        if values[i] - max(left_min, right_min) >= prominence:
            peaks.append(i)
    return np.asarray(peaks, dtype=int)


def _welch_csd(reference: np.ndarray, response: np.ndarray, fs: float, nperseg: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    nperseg = min(nperseg, len(reference), len(response))
    if nperseg < 8:
        raise ValueError("nperseg is too small for a spectral estimate")
    step = max(1, nperseg // 2)
    starts = list(range(0, len(reference) - nperseg + 1, step))
    if not starts:
        starts = [0]
        nperseg = len(reference)

    window = np.hanning(nperseg)
    window_power = fs * np.sum(window * window)
    pxy_accum = None
    pxx_accum = None
    for start in starts:
        x = reference[start : start + nperseg]
        y = response[start : start + nperseg]
        x = (x - np.mean(x)) * window
        y = (y - np.mean(y)) * window
        x_fft = np.fft.rfft(x)
        y_fft = np.fft.rfft(y)
        pxy = np.conj(x_fft) * y_fft / window_power
        pxx = np.conj(x_fft) * x_fft / window_power
        pxy_accum = pxy if pxy_accum is None else pxy_accum + pxy
        pxx_accum = pxx if pxx_accum is None else pxx_accum + pxx

    pxy_avg = pxy_accum / len(starts)
    pxx_avg = pxx_accum / len(starts)
    freq = np.fft.rfftfreq(nperseg, d=1.0 / fs)
    return freq, pxy_avg, pxx_avg


def compute_bode(
    csv_path: str | Path,
    preset_name: str | None = None,
    reference: str | None = None,
    response: str | None = None,
    title: str | None = None,
    units: str | None = None,
    f_min: float | None = None,
    f_max: float | None = None,
    chirp_start_hz: float | None = 0.1,
    chirp_end_hz: float | None = 10.0,
    chirp_duration_s: float | None = 10.0,
    chirp_kind: str = "linear",
    trim_start_s: float | None = None,
    trim_end_s: float | None = None,
    ref_scale: float = 1.0,
    detrend: bool = True,
    invert_response: bool = False,
    lowpass_hz: float | None = None,
    filter_order: int = 2,
    freq_resolution_hz: float = 0.02,
) -> BodeResult:
    csv_resolved = resolve_csv_path(csv_path)
    columns = read_csv_columns(csv_resolved)
    preset = PRESETS.get(preset_name or "") if preset_name else None

    ref_name = reference or (preset.reference if preset else None)
    resp_name = response or (preset.response if preset else None)
    if not ref_name or not resp_name:
        raise ValueError("Choose a preset or pass both reference and response columns")
    missing = [name for name in (ref_name, resp_name) if name not in columns]
    if missing:
        available = ", ".join(columns.keys())
        raise KeyError(f"Missing column(s): {', '.join(missing)}\nAvailable columns:\n{available}")

    time = time_from_columns(columns)
    ref = np.asarray(columns[ref_name], dtype=float) * ref_scale
    resp_raw = np.asarray(columns[resp_name], dtype=float)
    if invert_response:
        resp_raw = -resp_raw
    keep = _valid_window(time, [ref, resp_raw], trim_start_s, trim_end_s)
    time = time[keep]
    ref = ref[keep]
    resp_raw = resp_raw[keep]
    chirp_freq = chirp_frequency_profile(time, chirp_start_hz, chirp_end_hz, chirp_duration_s, chirp_kind)

    if time.size < 8:
        raise ValueError("Not enough samples after trimming")

    if detrend:
        ref = ref - np.mean(ref)
        resp_raw = resp_raw - np.mean(resp_raw)
    resp = zero_phase_lowpass(resp_raw, 1.0 / np.median(np.diff(time)), lowpass_hz, filter_order) if lowpass_hz else resp_raw

    dt = np.median(np.diff(time))
    fs = 1.0 / dt
    nperseg = int(2 ** np.round(np.log2(fs / freq_resolution_hz)))
    nperseg = max(8, min(nperseg, len(time) // 4))

    f, pxy, pxx = _welch_csd(ref, resp, fs=fs, nperseg=nperseg)
    h = pxy / (pxx + 1e-30)
    mag_db = 20.0 * np.log10(np.abs(h) + 1e-12)
    phase_deg = np.degrees(np.angle(h))

    if f_min is None and chirp_freq is not None and np.any(chirp_freq > 0):
        f_min = max(float(np.min(chirp_freq[chirp_freq > 0])), 0.01)
    if f_max is None and chirp_freq is not None and np.any(chirp_freq > 0):
        f_max = float(np.max(chirp_freq))
    f_min = 0.01 if f_min is None else float(f_min)
    f_max = fs / 2.0 if f_max is None or f_max <= 0 else float(f_max)
    band = (f >= f_min) & (f <= f_max) & (f > 0)

    f_band = f[band]
    mag_band = mag_db[band]
    phase_band = phase_deg[band]
    f_3db, f_90 = _crossing_frequency(f_band, mag_band, phase_band)

    return BodeResult(
        csv_path=csv_resolved,
        title=title or (preset.title if preset else f"{ref_name} to {resp_name}"),
        ref_name=ref_name,
        resp_name=resp_name,
        units=units or (preset.units if preset else ""),
        time=time,
        reference=ref,
        response=resp,
        response_raw=resp_raw,
        chirp_frequency=chirp_freq,
        fs=fs,
        nperseg=nperseg,
        frequency=f,
        magnitude_db=mag_db,
        phase_deg=phase_deg,
        mask=band,
        f_3db=f_3db,
        f_90=f_90,
    )


def make_bode_figure(result: BodeResult, show_raw: bool = False):
    import matplotlib.pyplot as plt

    f = result.frequency
    mask = result.mask
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 8), constrained_layout=True)
    ax2.sharex(ax1)
    fig.suptitle(
        f"{result.title}\n"
        f"ref: {result.ref_name}   |   resp: {result.resp_name}\n"
        f"{result.csv_path}    fs={result.fs:.1f} Hz    nperseg={result.nperseg}",
        fontsize=11,
    )

    ax1.semilogx(f[mask], result.magnitude_db[mask], color="steelblue", linewidth=1.2)
    ax1.set_ylabel("Magnitude (dB)")
    ax1.grid(True, which="both", linestyle="--", alpha=0.5)
    ax1.axhline(0, color="green", linestyle=":", linewidth=0.8, label="0 dB")
    ax1.axhline(-3, color="red", linestyle=":", linewidth=0.8, label="-3 dB")
    if result.f_3db is not None:
        ax1.axvline(result.f_3db, color="red", linestyle="--", linewidth=0.9)
        ax1.plot(result.f_3db, -3, "ro", markersize=5)
        ax1.annotate(f"-3 dB @ {result.f_3db:.2f} Hz", (result.f_3db, -3), xytext=(8, -14),
                     textcoords="offset points", fontsize=9, color="red")
    if np.any(mask):
        f_band = f[mask]
        analysis = [
            f"Band: {float(f_band[0]):.3g}-{float(f_band[-1]):.3g} Hz",
            f"-3 dB: {result.f_3db:.3g} Hz" if result.f_3db is not None else "-3 dB: not found",
            f"-90 deg: {result.f_90:.3g} Hz" if result.f_90 is not None else "-90 deg: not found",
        ]
        ax1.text(
            0.02,
            0.96,
            "\n".join(analysis),
            transform=ax1.transAxes,
            ha="left",
            va="top",
            fontsize=9,
            bbox={"facecolor": "white", "edgecolor": "0.8", "alpha": 0.75, "pad": 4},
        )
    ax1.legend(fontsize=9)

    ax2.semilogx(f[mask], result.phase_deg[mask], color="darkorange", linewidth=1.2)
    ax2.set_ylabel("Phase (deg)")
    ax2.set_xlabel("Frequency (Hz)")
    ax2.set_ylim(-190, 190)
    ax2.set_yticks([-180, -135, -90, -45, 0, 45, 90, 135, 180])
    ax2.axhline(0, color="green", linestyle=":", linewidth=0.8)
    ax2.axhline(-90, color="purple", linestyle=":", linewidth=0.8)
    ax2.axhline(-180, color="red", linestyle=":", linewidth=0.8)
    if result.f_90 is not None:
        ax2.axvline(result.f_90, color="purple", linestyle="--", linewidth=0.9)
        ax2.plot(result.f_90, -90, "o", color="purple", markersize=5)
        ax2.annotate(f"-90 deg @ {result.f_90:.2f} Hz", (result.f_90, -90), xytext=(8, 10),
                     textcoords="offset points", fontsize=9, color="purple")
    ax2.grid(True, which="both", linestyle="--", alpha=0.5)

    t_rel = result.time - result.time[0]
    ax3.plot(t_rel, result.reference, color="steelblue", linewidth=0.8, label=result.ref_name)
    if show_raw and not np.allclose(result.response, result.response_raw):
        ax3.plot(t_rel, result.response_raw, color="lightgrey", linewidth=0.6, label=f"{result.resp_name} raw")
    ax3.plot(t_rel, result.response, color="darkorange", linewidth=0.8, label=result.resp_name)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel(result.units or "Signal")
    ax3.set_title("Time Series")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(fontsize=9)

    if result.chirp_frequency is not None:
        ax3b = ax3.twinx()
        ax3b.plot(t_rel, result.chirp_frequency, color="grey", linewidth=0.7, linestyle="--", label="chirp frequency")
        ax3b.set_ylabel("Chirp Frequency (Hz)", color="grey")
        ax3b.tick_params(axis="y", labelcolor="grey")
        ax3b.legend(loc="upper left", fontsize=9)

    return fig
