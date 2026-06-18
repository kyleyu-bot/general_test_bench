"""
Impact torque test.

Sets the main drive into CST (Cyclic Synchronous Torque, mode 10) and runs
a three-phase torque sequence:

  Phase 1 — Forward torque:
      Commands torque_nm for duration_s.

  Phase 2 — Zero and pause:
      Commands 0 Nm for 1 second.

  Phase 3 — Reverse torque:
      Commands -torque_nm for duration_s, then ramps to 0 Nm.

Parameters
----------
torque_nm : float
    Torque to apply on the input side (Nm).  Positive = forward direction.
duration_s : float
    How long to hold the forward (and reverse) torque phase (s).

Framework contract (pre/post):
    Before run() — framework enables the drive and applies GUI modes.
    After  run() — framework zeros setpoints, disables drive, sets mode 0.
    This script sets enable/mode/torque explicitly for clarity and safety.
"""

import time

PARAMS = {
    "torque_nm":  0.1,   # torque to apply (Nm)
    "duration_s": 2.0,   # duration of each torque phase (s)
}

CST = 10  # DS402 mode 10: Cyclic Synchronous Torque


def run(params: dict, commander, stop_event):
    torque_nm  = float(params["torque_nm"])
    duration_s = float(params["duration_s"])

    def _send(torque: float):
        commander.set_command(
            numeric     = {"main_torque": torque},
            main_enable = True,
            dut_enable  = False,
            main_mode   = CST,
            dut_mode    = 0,
        )

    def _hold(torque: float, duration: float):
        _send(torque)
        t0 = time.monotonic()
        while not stop_event.is_set() and time.monotonic() - t0 < duration:
            time.sleep(0.005)

    # Phase 1: forward torque
    _hold(torque_nm, duration_s)

    if stop_event.is_set():
        _send(0.0)
        return

    # Phase 2: zero torque, pause 1 second
    _hold(0.0, 1.0)

    if stop_event.is_set():
        _send(0.0)
        return

    # Phase 3: reverse torque for the same duration
    _hold(-torque_nm, duration_s)

    # Zero torque and disable — framework epilogue also does this.
    commander.set_command(
        numeric     = {"main_torque": 0.0},
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )
