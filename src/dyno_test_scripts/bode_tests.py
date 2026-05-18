"""
Bode plot chirp test.

Runs a frequency-sweep (chirp) waveform through the drive's built-in function
generator to characterise the frequency response of the selected control loop.

Modes
-----
current  : FG drives q-axis current setpoint (iq_command_a).
torque   : FG drives torque setpoint (Nm). Response read from external sensor.
velocity : FG drives velocity setpoint (mrev/s).
position : FG drives position setpoint (encoder counts).

Chirp types
-----------
linear : frequency sweeps linearly from f_start_hz to f_end_hz.
exp    : frequency sweeps exponentially (log-spaced) from f_start_hz to f_end_hz.

After the chirp completes, a Bode analysis is automatically run in a daemon
thread and the plot is saved alongside the data log (or in
actuator_test_log/<serial>/<HHMMSS>_bode_plot/ when a serial number is set).

Framework contract (pre/post):
    Before run() — framework enables both drives and applies GUI modes.
    After  run() — framework zeros setpoints, disables drives, sets mode 0.
    This script overrides both explicitly for clarity and safety.
"""

import time

# Waveform type integers (must match WaveformType enum in function_generator.hpp)
_CHIRP_LINEAR      = 7
_CHIRP_EXPONENTIAL = 8

# FG control type integers (must match ControlType enum in function_generator.hpp)
_FG_CTRL_VELOCITY = 1
_FG_CTRL_POSITION = 2
_FG_CTRL_TORQUE   = 3
_FG_CTRL_CURRENT  = 4

# DS402 mode codes
_MODE_CSP     = 8    # Cyclic Synchronous Position
_MODE_CSV     = 9    # Cyclic Synchronous Velocity
_MODE_CST     = 10   # Cyclic Synchronous Torque
_MODE_CURRENT = -2   # Vendor direct-current mode

# mode name → (ds402_mode, fg_ctrl_type)
_MODE_CONFIG = {
    "current":  (_MODE_CURRENT, _FG_CTRL_CURRENT),
    "torque":   (_MODE_CST,     _FG_CTRL_TORQUE),
    "velocity": (_MODE_CSV,     _FG_CTRL_VELOCITY),
    "position": (_MODE_CSP,     _FG_CTRL_POSITION),
}

PARAMS = {
    "drive":         ["main", "dut"],
    "mode":          ["current", "torque", "velocity", "position"],
    "torque_sensor": ["ch1", "ch2"],   # used only when mode == "torque"
    "chirp_type":    ["linear", "exp"],
    "amplitude":     0.1,
    "offset":        0.0,
    "phase":         0.0,
    "f_start_hz":    0.1,
    "f_end_hz":      10.0,
    "duration_s":    30.0,
}


def _post_process(
    script_file: str,
    drive: str,
    mode: str,
    torque_sensor: str,
    chirp_type: str,
    f_start_hz: float,
    f_end_hz: float,
    duration_s: float,
) -> None:
    """Find this test's log folder, run Bode analysis, and save the plot."""
    import importlib
    import os as _os
    import re as _re
    import sys as _sys
    import time as _time
    from pathlib import Path

    _time.sleep(3)  # wait for bridge log rotation to close the CSV

    stem      = Path(script_file).stem
    repo_root = Path(script_file).resolve().parents[2]
    log_root  = repo_root / "test_data_log"
    now       = _time.time()
    candidates = [
        p for p in log_root.glob(f"*/*_{stem}")
        if p.is_dir() and now - p.stat().st_mtime < 600
    ]
    if not candidates:
        return
    log_folder = max(candidates, key=lambda p: p.stat().st_mtime)

    # Locate CSV (compressed or plain)
    csv_path = log_folder / "dyno_pdo.csv.gz"
    if not csv_path.exists():
        csv_path = log_folder / "dyno_pdo.csv"
    if not csv_path.exists():
        return

    # Determine bode preset or explicit ref/resp columns
    preset_name = None
    ref_col     = None
    resp_col    = None

    if mode == "current":
        preset_name = f"{drive}_current"
    elif mode == "velocity":
        preset_name = f"{drive}_velocity"
    elif mode == "position":
        preset_name = f"{drive}_position"
    elif mode == "torque":
        # Named presets only exist for main+ch1 and dut+ch2
        if drive == "main" and torque_sensor == "ch1":
            preset_name = "main_torque_ch1"
        elif drive == "dut" and torque_sensor == "ch2":
            preset_name = "dut_torque_ch2"
        else:
            ref_col  = f"{drive}_rx_torque_command"
            resp_col = f"torque_{torque_sensor}_nm"

    # Read serial number if available
    sn = None
    sn_file = log_folder / "actuator_serial_number.txt"
    if sn_file.exists():
        try:
            m = _re.search(r'=\s*"?([^"\n\r]+)"?', sn_file.read_text())
            if m:
                sn = m.group(1).strip()
        except OSError:
            pass

    # Resolve output directory
    if sn:
        hhmmss  = log_folder.name.split("_")[0]
        out_dir = repo_root / "actuator_test_log" / sn / f"{hhmmss}_bode_plot"
    else:
        out_dir = log_folder
    out_dir.mkdir(parents=True, exist_ok=True)

    # Add bode library to path
    bode_dir = _os.path.join(_os.path.dirname(script_file),
                             "../tools/post_processing/bode_plot")
    if bode_dir not in _sys.path:
        _sys.path.insert(0, bode_dir)

    _os.environ.setdefault("MPLCONFIGDIR", "/tmp/dyno_matplotlib")

    try:
        import dyno_bode as _bode
        importlib.reload(_bode)

        chirp_kind = "exponential" if chirp_type == "exp" else "linear"
        result = _bode.compute_bode(
            csv_path,
            preset_name=preset_name,
            reference=ref_col,
            response=resp_col,
            chirp_start_hz=f_start_hz,
            chirp_end_hz=f_end_hz,
            chirp_duration_s=duration_s,
            chirp_kind=chirp_kind,
        )

        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig = _bode.make_bode_figure(result)
        label    = preset_name or f"{drive}_{mode}_{torque_sensor}"
        out_path = out_dir / f"bode_{label}.png"
        fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
        plt.close(fig)
    except Exception:
        pass  # silent — test already finished, don't surface errors to user


def run(params: dict, commander, stop_event):
    drive         = str(params["drive"])
    mode          = str(params["mode"])
    torque_sensor = str(params["torque_sensor"])
    chirp_type    = str(params["chirp_type"])
    amplitude     = float(params["amplitude"])
    offset        = float(params["offset"])
    phase         = float(params["phase"])
    f_start_hz    = float(params["f_start_hz"])
    f_end_hz      = float(params["f_end_hz"])
    duration_s    = float(params["duration_s"])

    is_main    = (drive == "main")
    fg_prefix  = "main_fg" if is_main else "dut_fg"
    ds402_mode, fg_ctrl_type = _MODE_CONFIG[mode]
    waveform   = _CHIRP_LINEAR if chirp_type == "linear" else _CHIRP_EXPONENTIAL

    def _set(numeric: dict, *, enable: bool = True):
        commander.set_command(
            numeric     = numeric,
            main_enable = is_main if enable else False,
            dut_enable  = (not is_main) if enable else False,
            main_mode   = (ds402_mode if is_main else 0) if enable else 0,
            dut_mode    = (ds402_mode if not is_main else 0) if enable else 0,
        )

    # Enable drive and let it reach Operation Enabled
    _set({})
    time.sleep(0.3)

    if stop_event.is_set():
        _set({}, enable=False)
        return

    # Start chirp via function generator
    _set({
        f"{fg_prefix}_enable":       True,
        f"{fg_prefix}_waveform":     waveform,
        f"{fg_prefix}_control_type": fg_ctrl_type,
        f"{fg_prefix}_amplitude":    amplitude,
        f"{fg_prefix}_frequency":    1.0,
        f"{fg_prefix}_offset":       offset,
        f"{fg_prefix}_phase":        phase,
        f"{fg_prefix}_chirp_f_low":  f_start_hz,
        f"{fg_prefix}_chirp_f_high": f_end_hz,
        f"{fg_prefix}_chirp_dur":    duration_s,
    })

    t0 = time.monotonic()
    while not stop_event.is_set() and time.monotonic() - t0 < duration_s:
        time.sleep(0.05)

    # Stop FG and disable drive
    _set({f"{fg_prefix}_enable": False}, enable=False)

    if not stop_event.is_set():
        import threading
        threading.Thread(
            target=_post_process,
            args=(__file__, drive, mode, torque_sensor, chirp_type,
                  f_start_hz, f_end_hz, duration_s),
            daemon=True,
        ).start()
