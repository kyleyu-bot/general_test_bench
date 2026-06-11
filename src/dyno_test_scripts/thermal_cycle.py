"""
Thermal cycle test.

Both drives run simultaneously.  The main drive holds at 0 speed (CSV mode)
with user-defined positive and negative torque loop output limits, acting as a
controlled resistive load.  The DUT drive runs at the user-specified speed.

Each cycle consists of a run phase followed by a cooldown phase at zero speed.
The sequence repeats for num_cycles cycles.

Parameters
----------
main_torque_limit_pos : float
    Main drive torque loop max output (Nm, positive). Default 5.0.
main_torque_limit_neg : float
    Main drive torque loop min output (Nm, negative). Default -5.0.
dut_speed_rad_s : float
    DUT drive speed setpoint (rad/s, input side). Default 10.0.
num_cycles : int
    Number of run/cooldown cycles. Default 3.
cycle_run_s : float
    Duration of the run phase per cycle (s). Default 300.
cycle_cool_s : float
    Duration of the cooldown phase per cycle (s). Default 60.

Framework contract (pre/post):
    Before run() — framework enables both drives and applies GUI modes.
    After  run() — framework zeros setpoints, disables drives, sets mode 0.
    This script overrides both explicitly for clarity and safety.
"""

import time

PARAMS = {
    "main_torque_limit_pos": 5.0,
    "main_torque_limit_neg": -5.0,
    "dut_speed_rad_s":       10.0,
    "num_cycles":            3,
    "cycle_run_s":           300.0,
    "cycle_cool_s":          60.0,
}

CSV = 9   # DS402 Cyclic Synchronous Velocity


def run(params: dict, commander, stop_event):
    torque_max  = float(params["main_torque_limit_pos"])
    torque_min  = float(params["main_torque_limit_neg"])
    dut_speed   = float(params["dut_speed_rad_s"])
    num_cycles  = max(1, int(params["num_cycles"]))
    cycle_run   = float(params["cycle_run_s"])
    cycle_cool  = float(params["cycle_cool_s"])

    def _send(dut_vel: float):
        commander.set_command(
            numeric     = {
                "main_velocity":    0.0,
                "main_trq_loop_max_amps":  torque_max,
                "main_trq_loop_min_amps":  torque_min,
                "dut_velocity":     dut_vel,
            },
            main_enable = True,
            dut_enable  = True,
            main_mode   = CSV,
            dut_mode    = CSV,
        )

    def _hold(duration_s: float, dut_vel: float):
        _send(dut_vel)
        t0 = time.monotonic()
        while not stop_event.is_set() and time.monotonic() - t0 < duration_s:
            time.sleep(0.05)

    for _ in range(num_cycles):
        if stop_event.is_set():
            break
        _hold(cycle_run,  dut_speed)   # run phase
        if stop_event.is_set():
            break
        _hold(cycle_cool, 0.0)         # cooldown phase

    # Zero and disable — framework epilogue also does this.
    commander.set_command(
        numeric     = {
            "main_velocity":   0.0,
            "main_trq_loop_max_amps": torque_max,
            "main_trq_loop_min_amps": torque_min,
            "dut_velocity":    0.0,
        },
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )
