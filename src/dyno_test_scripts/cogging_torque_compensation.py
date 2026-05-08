"""
Cogging torque compensation test.

Runs the selected drive at a constant input-side speed in the positive
direction for a calculated duration, then reverses for the same duration.

Speed conversion:
    velocity_rad_s = input_speed_rev_s × 2π

Duration per leg:
    leg_duration_s = target_input_revs / input_speed_rev_s

Parameters
----------
drive : str
    Which drive to command: "main" or "dut".
input_speed_rev_s : float
    Constant input-side speed (rev/s). Default 0.5.
target_input_revs : float
    Number of input-side revolutions per direction. Default 4.0.

Framework contract (pre/post):
    Before run() — framework enables both drives and applies GUI modes.
    After  run() — framework zeros setpoints, disables drives, sets mode 0.
    This script overrides both explicitly for clarity and safety.
"""

import math
import time

PARAMS = {
    "drive":             ["main", "dut"],
    "input_speed_rev_s": 0.5,
    "target_input_revs": 4.0,
}

CSV = 9   # DS402 Cyclic Synchronous Velocity


def _post_process(script_file: str, drive: str) -> None:
    """Find this test's log folder and run cogging analysis. Called in a daemon thread."""
    import time as _time, sys as _sys, os as _os
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

    analysis_dir = _os.path.join(_os.path.dirname(script_file),
                                 "../tools/post_processing/cogging_compensation_analysis")
    if analysis_dir not in _sys.path:
        _sys.path.insert(0, analysis_dir)
    import importlib
    import dyno_cogging_analysis as _mod
    importlib.reload(_mod)
    try:
        _mod.run_cogging_analysis(str(log_folder), drive=drive)
    except Exception:
        pass  # silent — test already finished, don't surface errors to user


def run(params: dict, commander, stop_event):
    drive       = params["drive"]
    is_main     = (drive == "main")
    speed_rev_s = float(params["input_speed_rev_s"])
    target_revs = float(params["target_input_revs"])
    vel_key     = "main_velocity" if is_main else "dut_velocity"

    vel_rad_s  = speed_rev_s * 2.0 * math.pi
    duration_s = target_revs / speed_rev_s

    def _send(vel: float):
        commander.set_command(
            numeric     = {vel_key: vel},
            main_enable = is_main,
            dut_enable  = not is_main,
            main_mode   = CSV if is_main     else 0,
            dut_mode    = CSV if not is_main else 0,
        )

    def _run_leg(vel: float):
        _send(vel)
        t0 = time.monotonic()
        while not stop_event.is_set():
            if time.monotonic() - t0 >= duration_s:
                break
            time.sleep(0.02)

    _run_leg(vel_rad_s)     # forward

    if not stop_event.is_set():
        _run_leg(-vel_rad_s)    # reverse

    # Zero and disable — framework epilogue also does this.
    commander.set_command(
        numeric     = {vel_key: 0.0},
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )

    if not stop_event.is_set():
        import threading
        threading.Thread(
            target=_post_process,
            args=(__file__, params["drive"]),
            daemon=True,
        ).start()
