#!/usr/bin/env python3
"""
Inertia estimation analysis.

Loads a jipt_pdo.csv.gz from an inertia_estimation test run, detects the
forward- and reverse-torque phases, removes outliers via IQR, and estimates
rotor inertia from J = |torque| / |angular_acceleration|.

Velocity signal: main_tx_motor_velocity (motor-side, input-side encoder),
units mrev/s, converted to rad/s by multiplying by 2*pi/1000.
"""

from __future__ import annotations

import gzip
import pathlib
from typing import Tuple

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

VELOCITY_COL   = "main_tx_motor_velocity"   # mrev/s — motor-side input encoder
TORQUE_CMD_COL = "main_rx_torque_command"   # Nm  — used only for phase detection
IQ_ACTUAL_COL  = "main_tx_iq_actual"        # A   — measured q-axis current
TIMESTAMP_COL  = "stamp_ns"

_MREV_S_TO_RAD_S = 2.0 * np.pi / 1000.0   # 1 mrev/s → rad/s


# ── File helpers ─────────────────────────────────────────────────────────────

def _find_csv(folder: str | pathlib.Path) -> pathlib.Path:
    folder = pathlib.Path(folder)
    for name in ("jipt_pdo.csv.gz", "dyno_pdo.csv.gz"):
        candidate = folder / name
        if candidate.exists():
            return candidate
    for gz in sorted(folder.rglob("*.csv.gz"), reverse=True):
        return gz
    raise FileNotFoundError(f"No *.csv.gz found under {folder}")


def _latest_log(root: str | pathlib.Path = "test_data_log") -> pathlib.Path:
    root = pathlib.Path(root)
    candidates = sorted(
        root.rglob("*.csv.gz"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(f"No log files found under {root}")
    return candidates[0].parent


def _load_csv(csv_path: pathlib.Path) -> Tuple[np.ndarray, list]:
    open_fn = gzip.open if str(csv_path).endswith(".gz") else open
    with open_fn(csv_path, "rt") as fh:
        header = fh.readline().strip().split(",")
        rows = []
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append([float(v) for v in line.split(",")])
            except ValueError:
                continue
    if not rows:
        raise ValueError("CSV is empty or contains no numeric rows.")
    return np.array(rows, dtype=np.float64), header


# ── Analysis helpers ─────────────────────────────────────────────────────────

_CAPTURE_DELAY_S = 0.25   # skip first 0.25 s of each phase (current-loop settling)


def _phase_indices(
    time_s: np.ndarray,
    torque_mask: np.ndarray,
    duration_s: float | None,
) -> np.ndarray:
    """
    Return row indices belonging to a torque phase.

    The capture window is delayed by _CAPTURE_DELAY_S from the phase start to
    allow the current loop to settle.  If duration_s is given, the window is
    further limited to duration_s seconds after the delay:
        [t_start + delay, t_start + delay + duration_s]
    Without duration_s the window runs from the delay to the end of the phase.
    """
    indices = np.where(torque_mask)[0]
    if len(indices) == 0:
        return indices
    t_start  = time_s[indices[0]]
    t_offset = time_s[indices] - t_start
    after_delay = t_offset >= _CAPTURE_DELAY_S
    if duration_s is not None:
        after_delay &= t_offset <= _CAPTURE_DELAY_S + duration_s
    return indices[after_delay]


def _remove_outliers_iqr(values: np.ndarray, factor: float = 1.5) -> np.ndarray:
    """Remove values outside [Q1 - factor*IQR, Q3 + factor*IQR]."""
    if len(values) < 4:
        return values
    q1, q3 = np.percentile(values, [25, 75])
    iqr = q3 - q1
    if iqr == 0.0:
        return values
    keep = (values >= q1 - factor * iqr) & (values <= q3 + factor * iqr)
    return values[keep]


# ── Main analysis entry point ────────────────────────────────────────────────

def run_inertia_analysis(
    folder_or_csv: str | pathlib.Path,
    kt: float,
    duration_s: float | None = None,
    torque_threshold: float = 0.01,
    min_accel: float = 0.5,
    iqr_factor: float = 1.5,
) -> Tuple[float, plt.Figure, plt.Figure]:
    """
    Estimate rotor inertia from a JIPT inertia_estimation test log.

    Parameters
    ----------
    folder_or_csv :
        Run folder (or the .csv.gz file path directly).
    kt :
        Motor torque constant (Nm/A).  Actual torque is computed as
        iq_actual (main_tx_iq_actual) × kt.
    duration_s :
        Length of each torque phase to analyse (seconds).  When provided,
        only the first duration_s seconds of Phase 1 and Phase 3 are used
        (after the 0.25 s settling delay).  If None, all samples where the
        torque command is non-zero are used.
    torque_threshold :
        |torque command| below this value (Nm) marks the zero / pause phase.
        Only used for phase detection — not for the J calculation.
    min_accel :
        Minimum |acceleration| (rad/s²) to include; filters near-zero samples
        that would produce enormous J estimates.
    iqr_factor :
        Multiplier for the IQR outlier fence (default 1.5 = Tukey's rule).
        Larger values = fewer samples removed.

    Returns
    -------
    inertia_kgm2 : float
        Mean estimated rotor inertia after outlier removal (kg·m²).
    fig_time : matplotlib.figure.Figure
        Time-series figure: velocity, acceleration, per-sample J over time.
    fig_dist : matplotlib.figure.Figure
        Distribution figure: histogram of J estimates per phase.
    """
    path = pathlib.Path(folder_or_csv)
    csv_path = _find_csv(path) if path.is_dir() else path

    data, header = _load_csv(csv_path)

    vcol   = header.index(VELOCITY_COL)
    tcol   = header.index(TORQUE_CMD_COL)
    iq_col = header.index(IQ_ACTUAL_COL)
    tscol  = header.index(TIMESTAMP_COL)

    time_s    = (data[:, tscol] - data[0, tscol]) * 1e-9
    vel_rad_s = data[:, vcol] * _MREV_S_TO_RAD_S
    torque_cmd = data[:, tcol]                   # used only for phase detection
    torque_act = data[:, iq_col] * kt            # iq_actual × Kt  (Nm)

    accel = np.gradient(vel_rad_s, time_s)

    phase_defs = [
        ("Phase 1  (+torque)", torque_cmd >  torque_threshold),
        ("Phase 3  (−torque)", torque_cmd < -torque_threshold),
    ]

    estimates_clean = []
    phase_results   = []   # (label, t, vel, tor_act, acc, j_raw, j_clean)

    for label, torque_mask in phase_defs:
        idx = _phase_indices(time_s, torque_mask, duration_s)
        if len(idx) == 0:
            raise ValueError(
                f"{label} not detected — check log or lower torque_threshold."
            )

        ph_t   = time_s[idx]
        ph_vel = vel_rad_s[idx]
        ph_tor = torque_act[idx]          # iq_actual × Kt
        ph_acc = accel[idx]

        valid = np.abs(ph_acc) > min_accel
        if not valid.any():
            continue

        j_raw   = np.abs(ph_tor[valid]) / np.abs(ph_acc[valid])
        j_clean = _remove_outliers_iqr(j_raw, iqr_factor)

        if len(j_clean) == 0:
            continue

        mean_j = float(np.mean(j_clean))
        estimates_clean.append(mean_j)
        phase_results.append((label, ph_t, ph_vel, ph_tor, ph_acc,
                               j_raw, j_clean, valid))

    if not estimates_clean:
        raise ValueError(
            "No valid samples found after filtering. "
            "Try lowering min_accel or verify the drive moved during the test."
        )

    inertia_kgm2 = float(np.mean(estimates_clean))

    # ── Figure 1: Time series ────────────────────────────────────────────────
    fig_time, (ax_vel, ax_acc, ax_j) = plt.subplots(
        3, 1, figsize=(12, 8), sharex=True
    )

    ax_vel.plot(time_s, vel_rad_s, color="tab:blue", linewidth=0.8, label="velocity")
    for label, ph_t, ph_vel, *_ in phase_results:
        ax_vel.scatter(ph_t, ph_vel, s=4, alpha=0.5, label=label)
    ax_vel.set_ylabel("Velocity (rad/s)")
    ax_vel.legend(fontsize=8)
    ax_vel.grid(True, alpha=0.3)

    ax_acc.plot(time_s, accel, color="tab:orange", linewidth=0.8, label="acceleration")
    ax_acc.axhline(0, color="k", linewidth=0.5)
    ax_acc.set_ylabel("Acceleration (rad/s²)")
    ax_acc.legend(fontsize=8)
    ax_acc.grid(True, alpha=0.3)

    for label, ph_t, ph_vel, ph_tor, ph_acc, j_raw, j_clean, valid in phase_results:
        mean_j = float(np.mean(j_clean))
        # Plot all raw samples faint, clean samples bold
        ax_j.scatter(ph_t[valid], j_raw, s=3, alpha=0.25, color="grey")
        ax_j.scatter(ph_t[valid][np.isin(j_raw, j_clean)], j_clean,
                     s=5, alpha=0.7, label=f"{label}: J = {mean_j:.5f} kg·m²")
    ax_j.axhline(inertia_kgm2, color="red", linewidth=1.5, linestyle="--",
                 label=f"Mean J = {inertia_kgm2:.5f} kg·m²")
    ax_j.set_ylabel("Inertia estimate (kg·m²)")
    ax_j.set_xlabel("Time (s)")
    ax_j.legend(fontsize=8)
    ax_j.grid(True, alpha=0.3)

    fig_time.suptitle(
        f"Inertia Estimation — Time Series  |  Kt = {kt:.4f} Nm/A  |  J = {inertia_kgm2:.5f} kg·m²",
        fontsize=12,
    )
    fig_time.tight_layout()

    # ── Figure 2: Distribution ────────────────────────────────────────────────
    n_phases = len(phase_results)
    fig_dist, axes_dist = plt.subplots(
        1, n_phases, figsize=(6 * n_phases, 5), squeeze=False
    )

    colors = ["tab:blue", "tab:orange", "tab:green"]
    for i, (label, ph_t, ph_vel, ph_tor, ph_acc, j_raw, j_clean, valid) in \
            enumerate(phase_results):
        ax = axes_dist[0][i]
        mean_raw   = float(np.mean(j_raw))
        mean_clean = float(np.mean(j_clean))
        n_removed  = len(j_raw) - len(j_clean)

        ax.hist(j_raw,   bins=40, alpha=0.35, color="grey",         label="raw")
        ax.hist(j_clean, bins=40, alpha=0.75, color=colors[i % 3],  label="after IQR filter")
        ax.axvline(mean_raw,   color="grey",    linestyle="--", linewidth=1.2,
                   label=f"Raw mean: {mean_raw:.5f}")
        ax.axvline(mean_clean, color="red",     linestyle="-",  linewidth=1.5,
                   label=f"Clean mean: {mean_clean:.5f}")
        ax.set_title(f"{label}\n({n_removed} outliers removed)", fontsize=10)
        ax.set_xlabel("J estimate (kg·m²)")
        ax.set_ylabel("Count")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    fig_dist.suptitle(
        f"Inertia Distribution  |  Mean J = {inertia_kgm2:.5f} kg·m²",
        fontsize=12,
    )
    fig_dist.tight_layout()

    return inertia_kgm2, fig_time, fig_dist
