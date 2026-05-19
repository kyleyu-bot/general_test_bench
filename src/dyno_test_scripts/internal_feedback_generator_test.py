"""
Internal feedback generator test for Novanta drives.

Configures the drive's built-in waveform generator to produce a synthetic
commutation feedback signal (no physical encoder needed). Useful for
verifying commutation algorithm behaviour during motor bring-up.

SDO sequence
------------
1. COMMU_PHASING_MODE (0x2154) = 2  — disable phasing (fixed)
2. COMMU_ANGLE_SENSOR (0x2151) = 3  — switch to internal generator (fixed)
3. FBK_GEN_MODE        (0x2380)     — waveform: 0=constant, 1=sawtooth, 2=square
4. FBK_GEN_FREQ        (0x2381)     — frequency (Hz, FLOAT)
5. FBK_GEN_GAIN        (0x2382)     — amplitude scale (FLOAT)
6. FBK_GEN_OFFSET      (0x2383)     — DC offset (FLOAT)
7. FBK_GEN_CYCLES      (0x2384)     — cycle count (0 = continuous, UINT32)
8. <settle>
10. FBK_GEN_REARM      (0x2385) = 1 — arm/trigger the generator (WO)

Teardown: COMMU_ANGLE_SENSOR (0x2151) = 1  — restore normal sensor

Framework contract (pre/post):
    Before run() — framework enables both drives and applies GUI modes.
    After  run() — framework zeros setpoints, disables drives, sets mode 0.
"""

import struct
import time

PARAMS = {
    "drive":           ["main", "dut"],
    "fbk_gen_mode":    ["constant", "sawtooth", "square"],
    "fbk_gen_freq_hz": 1.0,
    "fbk_gen_gain":    1.0,
    "fbk_gen_offset":  0.0,
    "fbk_gen_cycles":  0,      # 0 = run continuously
    "control_mode":    ["current", "voltage"],  # current=-2, voltage=-1 → main_mode PDO
    "iq_command_a":    0.0,    # current command sent to main_current PDO during test
    "settle_time_s":   0.5,    # wait after setup SDOs before arming
}

_MODE_MAP    = {"constant": 0, "sawtooth": 1, "square": 2}
_CONTROL_MAP = {"current": -2, "voltage": -1}


def _f32_raw(value: float) -> int:
    return int.from_bytes(struct.pack("<f", value), "little")


def run(params: dict, commander, stop_event):
    drive      = str(params["drive"])
    gen_mode   = _MODE_MAP.get(str(params["fbk_gen_mode"]), 0)
    freq       = float(params["fbk_gen_freq_hz"])
    gain       = float(params["fbk_gen_gain"])
    offset     = float(params["fbk_gen_offset"])
    cycles       = int(params["fbk_gen_cycles"])
    control_mode = _CONTROL_MAP.get(str(params["control_mode"]), -2)
    iq_command   = float(params["iq_command_a"])
    settle       = float(params["settle_time_s"])

    def sdo(index, subindex, size, value):
        if stop_event.is_set():
            return
        commander.request_sdo(drive, "write", index, subindex, size, value)
        time.sleep(0.05)

    is_main = (drive == "main")

    # Disable drive before writing registers that reject writes while enabled.
    commander.set_command(
        numeric     = {},
        main_enable = False if is_main else True,
        dut_enable  = False if not is_main else True,
        main_mode   = 0,
        dut_mode    = 0,
    )
    time.sleep(0.3)  # wait for drive to reach Switch On Disabled

    if stop_event.is_set():
        return

    # Fixed setup SDOs (require drive disabled)
    sdo(0x2154, 0x00, 2, 2)                 # COMMU_PHASING_MODE = 2 (no phasing)
    sdo(0x2151, 0x00, 2, 3)                 # COMMU_ANGLE_SENSOR = 3 (internal gen)

    # Re-enable drive
    commander.set_command(
        numeric     = {},
        main_enable = True if is_main else False,
        dut_enable  = True if not is_main else False,
        main_mode   = control_mode,
        dut_mode    = 0,
    )
    time.sleep(0.3)  # wait for Operation Enabled

    if stop_event.is_set():
        return

    # User-configurable generator parameters
    sdo(0x2380, 0x00, 2, gen_mode)          # FBK_GEN_MODE
    # time.sleep(0.05)
    sdo(0x2381, 0x00, 4, _f32_raw(freq))    # FBK_GEN_FREQ (Hz, FLOAT)
    # time.sleep(0.05)
    sdo(0x2382, 0x00, 4, _f32_raw(gain))    # FBK_GEN_GAIN (FLOAT)
    # time.sleep(0.05)
    sdo(0x2383, 0x00, 4, _f32_raw(offset))  # FBK_GEN_OFFSET (FLOAT)
    # time.sleep(0.05)
    sdo(0x2384, 0x00, 4, cycles)                        # FBK_GEN_CYCLES (0 = continuous)
    # time.sleep(0.05)

    # Settle, then arm
    t0 = time.monotonic()
    while not stop_event.is_set() and time.monotonic() - t0 < settle:
        time.sleep(0.05)
    sdo(0x2385, 0x00, 2, 1)                 # FBK_GEN_REARM = 1

    # Hold for calculated duration (cycles / freq) or until aborted.
    # cycles == 0 means continuous — run until aborted.
    run_time_s = (cycles / freq) if (cycles > 0 and freq > 0) else float("inf")
    t_start = time.monotonic()
    while not stop_event.is_set() and time.monotonic() - t_start < run_time_s:
        commander.set_command(
            numeric     = {"main_current": iq_command},
            main_enable = True,
            dut_enable  = False,
            main_mode   = control_mode,
            dut_mode    = 0,
        )
        time.sleep(0.05)

    # Teardown: zero current and disable drive
    commander.set_command(
        numeric     = {"main_current": 0.0},
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )
    time.sleep(0.3)  # wait for Switch On Disabled before SDO write

    # Teardown: restore normal commutation sensor
    commander.request_sdo(drive, "write", 0x2151, 0x00, 2, 1)
    time.sleep(0.05)
