"""
Torque ramp test script.

Puts the selected drive into mode 10 (Cyclic Synchronous Torque, CST) and
linearly ramps the torque command from start_torque_nm to end_torque_nm over
rise_time_s, holds at the peak, then ramps back down to 0 Nm over 1 second.

The ramp step size is derived from the commander's publish period so every
iteration corresponds to exactly one ROS2 command publish:

    step (Nm) = (end - start) / rise_time_s  ×  loop_dt

Safety
------
At every step the selected torque sensor channel is read.  If the absolute
torque reading reaches 97.5 % of the channel's full-scale range the script
immediately ramps the command torque to 0 Nm over 1 second and exits early.

Parameters
----------
start_torque_nm : float
    Torque setpoint at the beginning of the ramp (Nm).
end_torque_nm : float
    Peak torque setpoint (Nm). May be negative for reverse torque.
rise_time_s : float
    Duration of the linear ramp from start → end (s).
hold_time_s : float
    Duration to hold at the peak torque before ramping down (s).
drive : str
    Which drive to command: "main" or "dut".
torque_sensor : str
    ELM3002 channel used for the safety check: "ch1" or "ch2".

Framework contract (pre/post):
    Before run() — framework has enabled both drives and applied GUI modes.
    After  run() — framework zeros setpoints, disables drives, sets mode 0.
    This script overrides both explicitly for clarity and safety.
"""

import time

PARAMS = {
    "start_torque_nm": 0.0,   # torque at ramp start (Nm)
    "end_torque_nm":   5.0,   # peak torque (Nm)
    "rise_time_s":     2.0,   # ramp duration start → end (s)
    "hold_time_s":     3.0,   # hold duration at peak (s)
    "drive":           ["main", "dut"],   # dropdown
    "torque_sensor":   ["ch1", "ch2"],    # dropdown — channel used for safety check
}


def _post_process(script_file: str, drive: str, torque_sensor: str) -> None:
    """Find this test's log folder and run Kt analysis. Called in a daemon thread."""
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

    kt_dir = _os.path.join(_os.path.dirname(script_file),
                           "../tools/post_processing/kt_plot")
    if kt_dir not in _sys.path:
        _sys.path.insert(0, kt_dir)
    import importlib
    import dyno_kt_analysis as _mod
    importlib.reload(_mod)
    try:
        _mod.run_kt_analysis(str(log_folder), drive=drive, torque_sensor=torque_sensor)
    except Exception:
        pass  # silent — test already finished, don't surface errors to user


def run(params: dict, commander, stop_event):
    start_nm      = float(params["start_torque_nm"])
    end_nm        = float(params["end_torque_nm"])
    rise_t        = float(params["rise_time_s"])
    hold_t        = float(params["hold_time_s"])
    torque_sensor = params["torque_sensor"]   # "ch1" or "ch2"

    loop_dt    = commander.pub_period_s
    is_main    = (params["drive"] == "main")
    torque_key = "main_torque" if is_main else "dut_torque"
    CST        = 10   # DS402 mode 10: Cyclic Synchronous Torque

    # Safety threshold: 97.5 % of the selected channel's full-scale range.
    scale_nm     = commander.get_torque_scale(torque_sensor)
    safety_limit = 0.975 * scale_nm

    def _send(torque_nm: float):
        commander.set_command(
            numeric     = {torque_key: torque_nm},
            main_enable = is_main,
            dut_enable  = not is_main,
            main_mode   = CST if is_main     else 0,
            dut_mode    = CST if not is_main else 0,
        )

    def _over_limit() -> bool:
        """Return True when the sensor reading hits the safety threshold."""
        return abs(commander.get_torque(torque_sensor)) >= safety_limit

    def _ramp(from_nm: float, to_nm: float, duration_s: float,
              check_safety: bool = True) -> tuple[float, bool]:
        """
        Linearly ramp torque from from_nm to to_nm over duration_s.

        Returns (last_commanded_nm, safety_tripped).
        Exits early if stop_event is set or (when check_safety=True) if the
        sensor reading exceeds the safety threshold.
        """
        if duration_s <= 0 or from_nm == to_nm:
            _send(to_nm)
            return to_nm, False
        step   = (to_nm - from_nm) / duration_s * loop_dt
        torque = from_nm
        while not stop_event.is_set():
            torque += step
            if (step > 0 and torque >= to_nm) or (step < 0 and torque <= to_nm):
                _send(to_nm)
                return to_nm, False
            _send(torque)
            time.sleep(loop_dt)
            if check_safety and _over_limit():
                return torque, True   # safety trip
        return torque, False

    # ── Ramp up: start_nm → end_nm over rise_t ───────────────────────────────
    last_torque, safety = _ramp(start_nm, end_nm, rise_t)

    # ── Hold at end_nm ────────────────────────────────────────────────────────
    if not safety and not stop_event.is_set():
        t0 = time.monotonic()
        while not stop_event.is_set() and time.monotonic() - t0 < hold_t:
            if _over_limit():
                safety = True
                break
            time.sleep(0.05)

    # ── Ramp down: 1 second to 0 Nm (safety or normal) ───────────────────────
    # check_safety=False so the ramp-down always completes regardless of sensor.
    _ramp(last_torque if safety else end_nm, 0.0, 1.0, check_safety=False)

    # Zero torque and disable — framework epilogue also does this.
    commander.set_command(
        numeric     = {torque_key: 0.0},
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )

    if not stop_event.is_set():
        import threading
        threading.Thread(
            target=_post_process,
            args=(__file__, params["drive"], params["torque_sensor"]),
            daemon=True,
        ).start()
