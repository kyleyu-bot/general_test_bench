"""
Encoder linearization test.

Runs the selected drive at a constant input-side speed in both directions and
monitors the output-side encoder position (post gear-ratio) via ROS2.  Each leg
stops once the output shaft has accumulated the target number of revolutions,
or when the safety timeout expires.

Speed conversion:
    velocity_rad_s = input_speed_rev_s × 2π

Output position is read from the drive telemetry field output_pos_rad,
which reflects the post-gear encoder in radians.  The test tracks the
absolute delta from the starting position so either rotation direction works.

Parameters
----------
drive : str
    Which drive to command: "main" or "dut".
input_speed_rev_s : float
    Input-side rotation speed magnitude (rev/s). Default 1.0.
target_output_revs : float
    Number of output-side revolutions to accumulate in each direction before
    stopping. Default 4.0.
timeout_s : float
    Per-direction safety timeout (s) — exits early if the target is not reached.
    Default 300.

Framework contract (pre/post):
    Before run() — framework enables both drives and applies GUI modes.
    After  run() — framework zeros setpoints, disables drives, sets mode 0.
    This script overrides both explicitly for clarity and safety.
"""

import math
import time

PARAMS = {
    "drive":               ["main", "dut"],
    "input_speed_rev_s":   1.0,
    "target_output_revs":  4.0,
    "timeout_s":           300.0,
}

CSV = 9   # DS402 Cyclic Synchronous Velocity


def run(params: dict, commander, stop_event):
    drive       = params["drive"]
    is_main     = (drive == "main")
    speed_rev_s = abs(float(params["input_speed_rev_s"]))
    target_revs = float(params["target_output_revs"])
    timeout_s   = float(params["timeout_s"])
    vel_key     = "main_velocity" if is_main else "dut_velocity"

    vel_rad_s  = speed_rev_s * 2.0 * math.pi
    target_rad = target_revs * 2.0 * math.pi

    def _send(vel: float):
        commander.set_command(
            numeric     = {vel_key: vel},
            main_enable = is_main,
            dut_enable  = not is_main,
            main_mode   = CSV if is_main     else 0,
            dut_mode    = CSV if not is_main else 0,
        )

    def _run_leg(vel: float) -> bool:
        start_pos_rad = commander.get_output_pos_rad(drive)
        _send(vel)

        t0 = time.monotonic()
        while not stop_event.is_set():
            if time.monotonic() - t0 >= timeout_s:
                return False
            delta = abs(commander.get_output_pos_rad(drive) - start_pos_rad)
            if delta >= target_rad:
                return True
            time.sleep(0.02)
        return False

    for direction in (1.0, -1.0):
        reached_target = _run_leg(direction * vel_rad_s)
        _send(0.0)
        time.sleep(0.25)
        if stop_event.is_set() or not reached_target:
            break

    # Zero and disable — framework epilogue also does this.
    commander.set_command(
        numeric     = {vel_key: 0.0},
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )
