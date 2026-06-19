#!/usr/bin/env python3
"""
Dyno Qt GUI — drag-and-drop command slider assignment.

Usage (from repo root):
    bash src/interface_bridges/ros2/run_gui.sh [options]

Options:
    --bridge   <path>     Path to bridge_ros2 binary
                          (default: build/jipt_ros2_bridge/bridge_ros2)
    --topology <path>     Topology JSON passed to bridge_ros2
                          (default: config/ethercat_device_config/topology.singlejoint1.json)
    --pub-hz   <float>    ROS2 publish rate Hz (default: 200)
    --no-bridge           Don't launch bridge_ros2 (connect to already-running bridge)
    --fault-reset-s <s>   fault_reset_s passed to bridge_ros2 (default: 2.0)
    --debug               Pass debug:=1 to bridge_ros2

Controls:
    Drag a field from the left panel onto a slider slot to assign it.
    Right-click a slot to unassign.
    Main Enable / DUT Enable  — toggle drive enable
    Main Zero  / DUT Zero     — zero all four command types for that drive
    Fault Reset               — one-shot fault clear pulse
"""

import argparse
import importlib.util
import json
import os
import signal
import struct
import subprocess
import sys
import threading
import time
import traceback

# ── Qt ────────────────────────────────────────────────────────────────────────
try:
    from PyQt5.QtCore    import Qt, QTimer, QMimeData, QByteArray, QRegularExpression, QObject, pyqtSignal
    from PyQt5.QtGui     import QFont, QRegularExpressionValidator
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget,
        QVBoxLayout, QHBoxLayout, QSplitter,
        QSlider, QPushButton, QLabel, QGroupBox,
        QSpinBox, QDoubleSpinBox, QListWidget, QListWidgetItem,
        QMenu, QAction, QComboBox, QTextEdit, QFormLayout,
        QScrollArea, QSizePolicy, QLineEdit, QCheckBox,
    )
except ImportError:
    print("ERROR: PyQt5 not found.  pip install PyQt5", file=sys.stderr)
    sys.exit(1)

# ── MIDI (optional) ───────────────────────────────────────────────────────────
try:
    import mido
    _MIDO_AVAILABLE = True
except ImportError:
    _MIDO_AVAILABLE = False

# ── ROS2 ─────────────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node   import Node
    from std_msgs.msg import String as StringMsg, Float64 as Float64Msg
except ImportError:
    print("ERROR: rclpy not found.  source /opt/ros/humble/setup.bash", file=sys.stderr)
    sys.exit(1)

# ── Constants ─────────────────────────────────────────────────────────────────

DEFAULT_BRIDGE      = "build/jipt_ros2_bridge/bridge_ros2"
DEFAULT_TOPOLOGY    = "config/ethercat_device_config/topology.singlejoint1.json"
DEFAULT_PUB_HZ      = 200.0
TEST_SCRIPTS_DIR    = "src/general_test_scripts"  # scanned for *.py test scripts
DEFAULT_FAULT_S  = 2.0
STARTUP_WINDOW_WIDTH  = 2400
STARTUP_WINDOW_HEIGHT = 360

CMD_MIME  = "application/x-dyno-command-field"
NUM_SLOTS      = 9   # number of slider slots shown
NUM_SPIN_SLOTS  = 6   # spinbox slots per row
NUM_SPIN_ROWS   = 2   # number of spinbox rows below sliders
SDO_TIMEOUT_S   = 3.0 # seconds before an SDO request is declared timed out

# ── Novanta error code lookup ─────────────────────────────────────────────────

_ERROR_MAP_PATH = "config/novanta_error_code_mapping/error_mapping_sectioned.json"

def _load_error_map(path: str) -> dict:
    """Load all sections of the Novanta error code JSON into a flat int→str dict."""
    try:
        with open(path) as f:
            raw = json.load(f)
    except Exception:
        return {}
    result = {}
    for section in raw.values():
        for hex_str, desc in section.items():
            try:
                result[int(hex_str, 16)] = desc
            except ValueError:
                pass
    return result

_ERROR_MAP: dict = _load_error_map(_ERROR_MAP_PATH)

def _lookup_error(code: int) -> str:
    """Return a display string for a DS402 error code. Empty string when code is 0."""
    if code == 0:
        return ""
    desc = _ERROR_MAP.get(code)
    if desc:
        return f"0x{code:08X} — {desc}"
    return f"0x{code:08X} — Unknown error"

# ── Novanta register map ──────────────────────────────────────────────────────

_REGISTER_MAP_PATH = "config/novanta_register_list/novanta_ethercat_registers_list.json"

def _load_register_map(path: str) -> dict:
    """Load register list into {(index_int, subindex_int): entry} dict."""
    try:
        with open(path) as f:
            entries = json.load(f)
    except Exception:
        return {}
    result = {}
    for e in entries:
        try:
            idx = int(e["Index"], 16)
            sub = int(e["Sub Index"], 16)
            result[(idx, sub)] = e
        except Exception:
            pass
    return result

_REGISTER_MAP: dict = _load_register_map(_REGISTER_MAP_PATH)

def _dtype_size(dtype: str) -> int:
    """Return byte size for a register Data Type string."""
    dt = dtype.upper()
    if dt in ("INT8", "UINT8"):
        return 1
    if dt in ("INT16", "UINT16"):
        return 2
    if dt in ("INT32", "UINT32", "FLOAT", "FLOAT32"):
        return 4
    if dt in ("UINT64",):
        return 8
    return 4  # default

def _decode_sdo_value(raw_int: int, dtype: str) -> str:
    """Format a raw integer value read from SDO according to its data type."""
    dt = dtype.upper()
    size = _dtype_size(dtype)
    if dt in ("FLOAT", "FLOAT32"):
        raw_bytes = raw_int.to_bytes(4, "little", signed=False)
        (f,) = struct.unpack("<f", raw_bytes)
        return f"{f:.6g}"
    if dt in ("INT8", "INT16", "INT32"):
        signed_map = {1: "<b", 2: "<h", 4: "<i"}
        raw_bytes = raw_int.to_bytes(size, "little", signed=False)
        (v,) = struct.unpack(signed_map[size], raw_bytes)
        return f"{v}  (0x{raw_int:0{size*2}X})"
    # unsigned int / other
    return f"{raw_int}  (0x{raw_int:0{size*2}X})"

def _encode_sdo_value(text: str, dtype: str) -> tuple:
    """
    Parse user-entered text as the given data type and return (raw_uint, size).
    raw_uint is the integer whose bytes will be written via SDO.
    """
    dt = dtype.upper()
    size = _dtype_size(dtype)
    text = text.strip()
    if not text:
        return 0, size
    if dt in ("FLOAT", "FLOAT32"):
        f = float(text)
        raw_bytes = struct.pack("<f", f)
        raw_uint = int.from_bytes(raw_bytes, "little")
        return raw_uint, 4
    if dt in ("INT8", "INT16", "INT32"):
        signed_map = {1: "<b", 2: "<h", 4: "<i"}
        # accept decimal or hex
        v = int(text, 0)
        raw_bytes = struct.pack(signed_map[size], v)
        raw_uint = int.from_bytes(raw_bytes, "little")
        return raw_uint, size
    # unsigned / default
    v = int(text, 0)
    return v, size

# ── All drag-assignable command fields ────────────────────────────────────────
# All drag-assignable command fields: (json_key, display_label)
COMMAND_FIELDS = [
    ("main_velocity",   "Main Velocity"),
    ("main_position",   "Main Position"),
    ("main_torque",     "Main Torque"),
    ("main_current",    "Main Current"),
    ("dut_velocity",    "DUT Velocity"),
    ("dut_position",    "DUT Position"),
    ("dut_torque",      "DUT Torque"),
    ("dut_current",     "DUT Current"),
    # Control gains
    ("main_torque_kp",  "Main Torque Kp"),
    ("main_torque_max", "Main Torque Max"),
    ("main_torque_min", "Main Torque Min"),
    ("main_vel_kp",     "Main Vel Kp"),
    ("main_vel_ki",     "Main Vel Ki"),
    ("main_vel_kd",     "Main Vel Kd"),
    ("main_pos_kp",     "Main Pos Kp"),
    ("main_pos_ki",     "Main Pos Ki"),
    ("main_pos_kd",     "Main Pos Kd"),
    ("dut_torque_kp",   "DUT Torque Kp"),
    ("dut_torque_max",  "DUT Torque Max"),
    ("dut_torque_min",  "DUT Torque Min"),
    ("dut_vel_kp",      "DUT Vel Kp"),
    ("dut_vel_ki",      "DUT Vel Ki"),
    ("dut_vel_kd",      "DUT Vel Kd"),
    ("dut_pos_kp",      "DUT Pos Kp"),
    ("dut_pos_ki",      "DUT Pos Ki"),
    ("dut_pos_kd",      "DUT Pos Kd"),
]

# Fields that use a float spinbox instead of the integer slider
GAIN_FIELDS = {
    "main_torque_kp", "main_torque_max", "main_torque_min",
    "main_vel_kp",    "main_vel_ki",     "main_vel_kd",
    "main_pos_kp",    "main_pos_ki",     "main_pos_kd",
    "dut_torque_kp",  "dut_torque_max",  "dut_torque_min",
    "dut_vel_kp",     "dut_vel_ki",      "dut_vel_kd",
    "dut_pos_kp",     "dut_pos_ki",      "dut_pos_kd",
}

# Torque clamp fields are algorithm-driven; use scale=1 float mode so the
# display shows physical Nm directly rather than a 1e6-scaled slider position.
TORQUE_CLAMP_FIELDS = {
    "main_torque_max", "main_torque_min",
    "dut_torque_max",  "dut_torque_min",
}

# DS402 modes of operation: (display label, int value sent in JSON)
DS402_MODES = [
    ("Current (-2)",                -2),
    ("No Mode (0)",                  0),
    ("Profile Position (1)",         1),
    ("Profile Velocity (2)",         2),
    ("Profile Torque (4)",           4),
    ("Cyclic Sync Position (8)",     8),
    ("Cyclic Sync Velocity (9)",     9),
    ("Cyclic Sync Torque (10)",     10),
]
DS402_DEFAULT_MODE = 9   # Cyclic Sync Velocity

# Allowed torque sensor scale values — must match Elm3002Adapter::ALLOWED_TORQUE_SCALES
TORQUE_SCALE_OPTIONS = [20, 200, 500]   # Nm
CURRENT_COMMAND_FALLBACK_LIMIT_A = 20.0

MAIN_ZERO_FIELDS = ["main_velocity", "main_position", "main_torque", "main_current"]
DUT_ZERO_FIELDS  = ["dut_velocity",  "dut_position",  "dut_torque",  "dut_current"]
ALL_CMD_KEYS     = [k for k, _ in COMMAND_FIELDS]

# Non-gain setpoint fields zeroed during script preamble/epilogue
_ZERO_CMD = {k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS}


def _interruptible_sleep(secs: float, stop_event, granularity: float = 0.01) -> bool:
    """Sleep for ``secs`` seconds, waking every ``granularity`` seconds to check
    stop_event.  Returns True if interrupted (stop_event set), False if the full
    duration elapsed."""
    deadline = time.monotonic() + secs
    while time.monotonic() < deadline:
        if stop_event.is_set():
            return True
        time.sleep(min(granularity, deadline - time.monotonic()))
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Test script discovery and execution
# ─────────────────────────────────────────────────────────────────────────────

def _discover_scripts(scripts_dir: str) -> list[str]:
    """Return sorted list of *.py paths in scripts_dir, newest first."""
    if not os.path.isdir(scripts_dir):
        return []
    paths = [
        os.path.join(scripts_dir, f)
        for f in os.listdir(scripts_dir)
        if f.endswith(".py") and not f.startswith("_")
    ]
    return sorted(paths, key=lambda p: os.path.getmtime(p), reverse=True)


def _load_script_module(path: str):
    """
    Dynamically load a test script module.
    Scripts must define:
        PARAMS: dict[str, float | int | str]  — editable parameters with defaults
        run(params: dict, commander) -> None  — called in a background thread
    """
    spec = importlib.util.spec_from_file_location("_dyno_test_script", path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class ScriptRunner:
    """Manages loading and running one test script at a time in a background thread."""

    def __init__(self):
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock       = threading.Lock()

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def run(self, path: str, params: dict, commander, on_done, on_log,
            main_mode: int = 9, dut_mode: int = 9,
            pre_settle_s: float = 0.1, post_settle_s: float = 0.1):
        """
        Start script in a background thread with preamble and epilogue.

        Preamble (runs before mod.run()):
          zero+disable → sleep(pre_settle_s) → enable → sleep(pre_settle_s)
          → set mode → sleep(pre_settle_s) → mod.run()

        Epilogue (runs after mod.run() regardless of success/abort):
          zero+keep_enabled → sleep(post_settle_s) → disable+mode=0
          → sleep(post_settle_s) → on_done()

        on_done(success: bool, msg: str) — called once, after epilogue completes
        on_log(line: str)                — called for preamble/epilogue status lines
        """
        if self.is_running:
            return
        self._stop_event.clear()
        mod = _load_script_module(path)

        def _target():
            zero = _ZERO_CMD.copy()

            # ── PREAMBLE ─────────────────────────────────────────────────────
            # Step 0: caller already sent zero+disable; wait for it to reach drive.
            on_log("[preamble] waiting for zeros to propagate…")
            if _interruptible_sleep(pre_settle_s, self._stop_event):
                on_done(False, "Aborted during pre-zero settle.")
                return

            # Step 1: enable both drives, mode still 0
            on_log("[preamble] enabling drives…")
            commander.set_command(
                numeric=zero, main_enable=True, dut_enable=True,
                main_mode=0, dut_mode=0)
            if _interruptible_sleep(pre_settle_s, self._stop_event):
                commander.set_command(
                    numeric=zero, main_enable=False, dut_enable=False,
                    main_mode=0, dut_mode=0)
                on_done(False, "Aborted during pre-enable settle.")
                return

            # Step 2: set target mode
            on_log(f"[preamble] setting mode (main={main_mode}, dut={dut_mode})…")
            commander.set_command(
                numeric=zero, main_enable=True, dut_enable=True,
                main_mode=main_mode, dut_mode=dut_mode)
            if _interruptible_sleep(pre_settle_s, self._stop_event):
                commander.set_command(
                    numeric=zero, main_enable=False, dut_enable=False,
                    main_mode=0, dut_mode=0)
                on_done(False, "Aborted during pre-mode settle.")
                return

            # ── SCRIPT ───────────────────────────────────────────────────────
            on_log("[preamble] starting script…")
            try:
                mod.run(params, commander, self._stop_event)
                success, msg = True, "Completed successfully."
            except Exception:
                success, msg = False, traceback.format_exc()

            # ── EPILOGUE ─────────────────────────────────────────────────────
            # Zero setpoints; keep drives briefly enabled so motion can settle.
            on_log("[epilogue] zeroing commands…")
            commander.set_command(
                numeric=zero, main_enable=True, dut_enable=True,
                main_mode=0, dut_mode=0)
            time.sleep(post_settle_s)   # unconditional — always clean up

            # Disable drives and clear mode.
            on_log("[epilogue] disabling drives…")
            commander.set_command(
                numeric=zero, main_enable=False, dut_enable=False,
                main_mode=0, dut_mode=0)
            time.sleep(post_settle_s)

            on_done(success, msg)

        self._thread = threading.Thread(target=_target, daemon=True)
        self._thread.start()

    def abort(self):
        """Signal the running script to stop (sets stop_event)."""
        self._stop_event.set()


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 commander node (runs on a background thread)
# ─────────────────────────────────────────────────────────────────────────────

class DynoCommander(Node):
    """Publishes /dyno/command at a fixed rate from the current GUI state."""

    def __init__(self, pub_hz: float):
        super().__init__("dyno_gui_commander")
        self._pub = self.create_publisher(StringMsg, "/jipt/command", 10)

        self._lock         = threading.Lock()
        # Gain fields are intentionally excluded — they are only added to the
        # payload when a slider is explicitly assigned to them, so the bridge
        # keeps its SDO-seeded values by default.
        self._numeric      = {k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS}
        self._main_enable       = False
        self._dut_enable        = False
        self._fault_reset       = False
        self._hold_output1      = False
        self._zero_torque_ch1   = False
        self._zero_torque_ch2   = False
        self._save_log          = False
        self._script_name       = ""
        self._inertia           = 0.0
        self._hardstop_pos_upper = 0.0
        self._hardstop_pos_lower = 0.0
        self._margin             = 0.0
        self._main_mode    = DS402_DEFAULT_MODE
        self._dut_mode     = DS402_DEFAULT_MODE

        # Drive limits — updated from status topics, used to auto-set slider ranges.
        _empty_limits = {
            "max_velocity_abs": 0.0, "min_position": 0.0, "max_position": 0.0,
            "max_current_a": 0.0,
            "torque_kp": 0.0, "torque_max": 0.0, "torque_min": 0.0,
            "torque_abs_max": 0.0, "rt_torque_max": 0.0, "rt_torque_min": 0.0,
            "vel_kp": 0.0, "vel_ki": 0.0, "vel_kd": 0.0,
            "pos_kp": 0.0, "pos_ki": 0.0, "pos_kd": 0.0,
        }
        self._limits_lock     = threading.Lock()
        self._main_limits     = dict(_empty_limits)
        self._dut_limits      = dict(_empty_limits)
        self._main_error_code: int = 0
        self._dut_error_code:  int = 0
        self._main_al_state:   str = "?"
        self._dut_al_state:    str = "?"
        self._limits_callback = None   # callable(drive: str), set by DynoWindow

        # Torque sensor readings — updated from /dyno/torque/{ch1,ch2}.
        # Scales are mirrored from set_command() so scripts can query them.
        self._ch1_torque_nm: float = 0.0
        self._ch2_torque_nm: float = 0.0
        self._ch1_scale_nm:  float = 200.0   # default, matches Elm3002Adapter ch1
        self._ch2_scale_nm:  float = 20.0    # default, matches Elm3002Adapter ch2

        # Output-side encoder position (rad, post gear-ratio) from drive status JSON.
        self._main_output_pos_rad: float = 0.0
        self._dut_output_pos_rad:  float = 0.0

        # Input-side encoder position (raw counts, 0x204A) from drive status JSON.
        self._main_input_enc_pos: int = 0
        self._dut_input_enc_pos:  int = 0

        self.create_subscription(StringMsg, "/jipt/main_drive/status",
            lambda msg: self._on_status(msg, "main"), 10)
        self.create_subscription(StringMsg, "/jipt/dut/status",
            lambda msg: self._on_status(msg, "dut"), 10)
        self.create_subscription(Float64Msg, "/jipt/torque/ch1",
            lambda msg: self._on_torque(msg, "ch1"), 10)
        self.create_subscription(Float64Msg, "/jipt/torque/ch2",
            lambda msg: self._on_torque(msg, "ch2"), 10)

        # SDO request/response
        self._sdo_pub = self.create_publisher(StringMsg, "/jipt/sdo_request", 10)
        self._pending_sdo_response = None        # written by ROS thread, read by Qt timer
        self._sdo_one_shot_callbacks: list = []  # list of (index, subindex, callback)
        self.create_subscription(StringMsg, "/jipt/sdo_response",
            self._on_sdo_response, 10)

        # Bus status (all slaves' AL states)
        self._pending_bus_status = None     # written by ROS thread, read by Qt timer
        self.create_subscription(StringMsg, "/jipt/bus_status",
            self._on_bus_status, 10)

        self._pub_period_s = 1.0 / max(pub_hz, 1.0)
        self.create_timer(self._pub_period_s, self._publish)

    @property
    def pub_period_s(self) -> float:
        """Publish period in seconds (1 / pub_hz). Scripts use this as their loop dt."""
        return self._pub_period_s

    def set_command(self, numeric: dict,
                    main_enable: bool, dut_enable: bool,
                    fault_reset: bool = False, hold_output1: bool = False,
                    main_mode: int = DS402_DEFAULT_MODE,
                    dut_mode:  int = DS402_DEFAULT_MODE):
        with self._lock:
            self._numeric.update(numeric)
            self._main_enable  = main_enable
            self._dut_enable   = dut_enable
            self._fault_reset  = fault_reset
            self._hold_output1 = hold_output1
            self._main_mode    = main_mode
            self._dut_mode     = dut_mode
        # Mirror torque scales so scripts can read them back via get_torque_scale().
        with self._limits_lock:
            if "ch1_torque_scale" in numeric:
                self._ch1_scale_nm = float(numeric["ch1_torque_scale"])
            if "ch2_torque_scale" in numeric:
                self._ch2_scale_nm = float(numeric["ch2_torque_scale"])

    def set_inertia(self, val: float) -> None:
        with self._lock:
            self._inertia = float(val)

    def set_hardstop_pos_upper(self, val: float) -> None:
        with self._lock:
            self._hardstop_pos_upper = float(val)

    def set_hardstop_pos_lower(self, val: float) -> None:
        with self._lock:
            self._hardstop_pos_lower = float(val)

    def set_margin(self, val: float) -> None:
        with self._lock:
            self._margin = float(val)

    def _on_status(self, msg: StringMsg, drive: str) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        # Prefer natural-unit fields (_rad_s / _rad); fall back to raw for compat
        limits = {
            "max_velocity_abs": float(data.get("max_velocity_abs_rad_s",
                                      data.get("max_velocity_abs", 0.0))),
            "min_position":     float(data.get("min_position_rad",
                                      data.get("min_position", 0))),
            "max_position":     float(data.get("max_position_rad",
                                      data.get("max_position", 0))),
            "max_current_a":     float(data.get("max_current_a", 0.0)),
            "torque_kp":      float(data.get("torque_kp",      0.0)),
            "torque_max":     float(data.get("torque_max",     0.0)),
            "torque_min":     float(data.get("torque_min",     0.0)),
            "torque_abs_max": float(data.get("torque_abs_max", 0.0)),
            "rt_torque_max":  float(data.get("rt_torque_max",  0.0)),
            "rt_torque_min":  float(data.get("rt_torque_min",  0.0)),
            "vel_kp":     float(data.get("vel_kp",     0.0)),
            "vel_ki":     float(data.get("vel_ki",     0.0)),
            "vel_kd":     float(data.get("vel_kd",     0.0)),
            "pos_kp":     float(data.get("pos_kp",     0.0)),
            "pos_ki":     float(data.get("pos_ki",     0.0)),
            "pos_kd":     float(data.get("pos_kd",     0.0)),
        }
        error_code = int(data.get("err", 0))
        al_state   = str(data.get("al", "?"))
        output_pos  = float(data.get("output_pos_rad", 0.0))
        input_enc   = int(data.get("in_enc_pos", 0))
        with self._limits_lock:
            if drive == "main":
                self._main_limits         = limits
                self._main_error_code     = error_code
                self._main_al_state       = al_state
                self._main_output_pos_rad = output_pos
                self._main_input_enc_pos  = input_enc
            else:
                self._dut_limits          = limits
                self._dut_error_code      = error_code
                self._dut_al_state        = al_state
                self._dut_output_pos_rad  = output_pos
                self._dut_input_enc_pos   = input_enc
        if self._limits_callback:
            self._limits_callback(drive)

    def get_error_code(self, drive: str) -> int:
        with self._limits_lock:
            return self._main_error_code if drive == "main" else self._dut_error_code

    def get_al_state(self, drive: str) -> str:
        with self._limits_lock:
            return self._main_al_state if drive == "main" else self._dut_al_state

    def _on_bus_status(self, msg: StringMsg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self._limits_lock:
            self._pending_bus_status = data

    def pop_bus_status(self):
        with self._limits_lock:
            data = self._pending_bus_status
            self._pending_bus_status = None
            return data

    def _on_torque(self, msg: Float64Msg, channel: str) -> None:
        with self._limits_lock:
            if channel == "ch1":
                self._ch1_torque_nm = msg.data
            else:
                self._ch2_torque_nm = msg.data

    def get_torque(self, channel: str) -> float:
        """Return the latest torque reading for 'ch1' or 'ch2' (Nm)."""
        with self._limits_lock:
            return self._ch1_torque_nm if channel == "ch1" else self._ch2_torque_nm

    def get_torque_scale(self, channel: str) -> float:
        """Return the current full-scale range (Nm) for 'ch1' or 'ch2'."""
        with self._limits_lock:
            return self._ch1_scale_nm if channel == "ch1" else self._ch2_scale_nm

    def get_output_pos_rad(self, drive: str) -> float:
        """Return the latest output-side encoder position (rad) for 'main' or 'dut'."""
        with self._limits_lock:
            return self._main_output_pos_rad if drive == "main" else self._dut_output_pos_rad

    def get_input_enc_pos_raw(self, drive: str) -> int:
        """Return the latest input-side encoder position (raw counts, 0x204A) for 'main' or 'dut'."""
        with self._limits_lock:
            return self._main_input_enc_pos if drive == "main" else self._dut_input_enc_pos

    def set_limits_callback(self, cb) -> None:
        self._limits_callback = cb

    def get_limits(self, field_key: str):
        """Return (min, max, default) for a command field, or None if unavailable.
        Gain fields return floats; setpoints return floats in natural units."""
        if field_key.startswith("main_"):
            with self._limits_lock:
                limits = dict(self._main_limits)
            field_type = field_key[5:]   # strip "main_"
        elif field_key.startswith("dut_"):
            with self._limits_lock:
                limits = dict(self._dut_limits)
            field_type = field_key[4:]   # strip "dut_"
        else:
            return None

        if field_type == "velocity":
            max_vel = limits["max_velocity_abs"]
            if max_vel > 0:
                return (-max_vel, max_vel, 0.0)
        elif field_type == "position":
            lo = limits["min_position"]
            hi = limits["max_position"]
            if lo == 0.0 and hi == 0.0:
                return None
            _NO_LIMIT = 1000.0   # rad — ~159 revolutions, usable default
            if lo <= -1e9:
                lo = -_NO_LIMIT
            if hi >= 1e9:
                hi = _NO_LIMIT
            return (lo, hi, 0.0)
        elif field_type == "current":
            max_current = limits.get("max_current_a", 0.0)
            if max_current > 0.0:
                return (-max_current, max_current, 0.0)
            return (-CURRENT_COMMAND_FALLBACK_LIMIT_A,
                    CURRENT_COMMAND_FALLBACK_LIMIT_A, 0.0)
        elif field_type == "torque":
            max_torque = limits.get("torque_max", 0.0)
            if max_torque > 0.0:
                return (-max_torque, max_torque, 0.0)
        elif field_type == "torque_max":
            abs_max = limits.get("torque_abs_max", 0.0) or 100.0
            current = limits.get("rt_torque_max", 0.0)
            return (-abs_max, abs_max, current)
        elif field_type == "torque_min":
            abs_max = limits.get("torque_abs_max", 0.0) or 100.0
            current = limits.get("rt_torque_min", 0.0)
            return (-abs_max, abs_max, current)
        elif field_type in ("torque_kp",
                            "vel_kp", "vel_ki", "vel_kd",
                            "pos_kp", "pos_ki", "pos_kd"):
            current = limits.get(field_type, 0.0)
            return (0.0, 20.0, current)   # float range; default = value from drive
        return None

    def pulse_fault_reset(self):
        """Send fault_reset=true for one publish cycle."""
        with self._lock:
            self._fault_reset = True

    def pulse_torque_zero_ch1(self):
        """Send zero_torque_ch1=true for one publish cycle."""
        with self._lock:
            self._zero_torque_ch1 = True

    def pulse_torque_zero_ch2(self):
        """Send zero_torque_ch2=true for one publish cycle."""
        with self._lock:
            self._zero_torque_ch2 = True

    def pulse_save_log(self):
        """Send save_log=true for one publish cycle, triggering log rotation in the bridge."""
        with self._lock:
            self._save_log = True

    def set_script_name(self, name: str) -> None:
        with self._lock:
            self._script_name = name

    def request_sdo(self, drive: str, op: str,
                    index: int, subindex: int, size: int, value: int = 0) -> None:
        payload = {
            "drive":    drive,
            "op":       op,
            "index":    f"{index:04X}",
            "subindex": f"{subindex:02X}",
            "size":     size,
            "value":    value,
        }
        msg      = StringMsg()
        msg.data = json.dumps(payload)
        self._sdo_pub.publish(msg)

    def request_pre_op(self, enter: bool) -> None:
        op = "pre_op_all" if enter else "pre_op_off"
        self._sdo_pub.publish(StringMsg(data=json.dumps({"op": op})))

    def request_store_all(self, drive: str) -> None:
        self._sdo_pub.publish(StringMsg(data=json.dumps({"op": "store_all", "drive": drive})))

    def _on_sdo_response(self, msg: StringMsg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self._limits_lock:
            self._pending_sdo_response = data
            try:
                resp_index = int(data.get("index", "0"), 16)
            except (ValueError, TypeError):
                resp_index = -1
            resp_sub = int(data.get("subindex", -1))
            matched   = []
            remaining = []
            for idx, sub, cb in self._sdo_one_shot_callbacks:
                if idx == resp_index and sub == resp_sub:
                    matched.append(cb)
                else:
                    remaining.append((idx, sub, cb))
            self._sdo_one_shot_callbacks = remaining
        for cb in matched:
            cb(data)

    def pop_sdo_response(self):
        with self._limits_lock:
            data = self._pending_sdo_response
            self._pending_sdo_response = None
            return data

    def register_sdo_one_shot(self, index: int, subindex: int, callback) -> None:
        """Register a callback invoked when an SDO response for (index, subindex) arrives."""
        with self._limits_lock:
            self._sdo_one_shot_callbacks.append((index, subindex, callback))

    def _publish(self):
        with self._lock:
            payload = dict(self._numeric)
            payload["main_iqcommand"]    = payload.get("main_current", 0.0)
            payload["dut_iqcommand"]     = payload.get("dut_current", 0.0)
            payload["main_enable"]       = self._main_enable
            payload["dut_enable"]        = self._dut_enable
            payload["fault_reset"]       = self._fault_reset
            payload["hold_output1"]      = self._hold_output1
            payload["main_mode"]         = self._main_mode
            payload["dut_mode"]          = self._dut_mode
            payload["zero_torque_ch1"]   = self._zero_torque_ch1
            payload["zero_torque_ch2"]   = self._zero_torque_ch2
            payload["save_log"]          = self._save_log
            payload["script_name"]       = self._script_name
            payload["inertia"]           = self._inertia
            payload["hardstop_pos_upper"] = self._hardstop_pos_upper
            payload["hardstop_pos_lower"] = self._hardstop_pos_lower
            payload["margin"]             = self._margin
            self._fault_reset          = False   # one-shot pulses
            self._zero_torque_ch1      = False
            self._zero_torque_ch2      = False
            self._save_log             = False

        msg      = StringMsg()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Command field list (drag source)
# ─────────────────────────────────────────────────────────────────────────────

class CommandFieldList(QListWidget):
    """Static list of assignable command fields — drag onto a SliderSlot."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragEnabled(True)
        self.setDragDropMode(QListWidget.DragOnly)
        self.setMaximumWidth(150)
        self.setMinimumWidth(110)

        for key, label in COMMAND_FIELDS:
            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, key)
            self.addItem(item)

    def mimeData(self, items):
        mime = QMimeData()
        if items:
            key = items[0].data(Qt.UserRole) or ""
            mime.setData(CMD_MIME, QByteArray(key.encode()))
        return mime


# ─────────────────────────────────────────────────────────────────────────────
# Slider slot (drop target)
# ─────────────────────────────────────────────────────────────────────────────
# Behringer X-Touch Compact MIDI bridge
# ─────────────────────────────────────────────────────────────────────────────

class XTouchMidiBridge(QObject):
    """
    Auto-detects a Behringer X-Touch Compact and provides bidirectional fader
    sync with the GUI slider slots.

    Protocol auto-detected on first message:
      - 'pitchwheel'    (MCU mode, 14-bit, channels 0-8 per fader)
      - 'control_change' (standard MIDI mode, 7-bit, CC 1-9 on ch 0)

    Standard MIDI CC layout (confirmed by probe):
      CC 1-9   = fader 1-9 position (0-127)
      CC 101+  = fader touch sensors — ignored
    """

    DEVICE_PATTERN  = "X-TOUCH COMPACT"
    NUM_FADERS      = 9
    FADER_CC_BASE   = 1   # fader 1 → CC 1, fader 2 → CC 2, …, fader 9 → CC 9

    # Emitted from background MIDI thread — use Qt queued connection for safety.
    # (fader_idx: 0-based int, normalized: float 0.0-1.0)
    fader_moved        = pyqtSignal(int, float)
    connection_changed = pyqtSignal(bool)   # True = just connected

    def __init__(self, parent=None):
        super().__init__(parent)
        self._in_port:  object | None = None
        self._out_port: object | None = None
        self._connected = False
        self._midi_mode: str | None   = None  # 'cc' | 'pitchwheel'

        self._scan_timer = QTimer(self)
        self._scan_timer.timeout.connect(self._scan)
        self._scan_timer.start(2000)
        self._scan()   # try immediately on startup

    # ── port detection ────────────────────────────────────────────────────────

    def _find_port(self, port_list):
        for n in port_list:
            if self.DEVICE_PATTERN in n.upper():
                return n
        return None

    def _scan(self):
        if self._connected:
            if not self._find_port(mido.get_input_names()):
                self._disconnect()
        else:
            name = self._find_port(mido.get_input_names())
            if name:
                self._connect(name)

    def _connect(self, in_name):
        try:
            self._in_port = mido.open_input(in_name)   # polling, no callback thread
            out_name = self._find_port(mido.get_output_names())
            if out_name:
                self._out_port = mido.open_output(out_name)
                # Park all fader motors at bottom so the device is in a known state.
                for i in range(self.NUM_FADERS):
                    self._out_port.send(mido.Message(
                        'control_change', channel=0,
                        control=i + self.FADER_CC_BASE, value=0))
            self._midi_mode = 'cc'   # default; overridden by first incoming message
            self._connected = True
            self._poll_timer = QTimer(self)
            self._poll_timer.timeout.connect(self._poll)
            self._poll_timer.start(5)   # 200 Hz — matches test script's 5 ms sleep
            self.connection_changed.emit(True)
        except Exception as exc:
            print(f"[MIDI] Connect error: {exc}", file=sys.stderr)

    def _disconnect(self):
        if hasattr(self, '_poll_timer') and self._poll_timer is not None:
            self._poll_timer.stop()
            self._poll_timer = None
        for attr in ("_in_port", "_out_port"):
            port = getattr(self, attr)
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass
                setattr(self, attr, None)
        self._connected = False
        self.connection_changed.emit(False)

    # ── MIDI polling (main thread via QTimer) ─────────────────────────────────

    def _poll(self):
        if self._in_port is None:
            return
        try:
            for msg in self._in_port.iter_pending():
                self._process_msg(msg)
        except Exception:
            self._disconnect()

    def _process_msg(self, msg):
        if msg.type == 'pitchwheel':
            self._midi_mode = 'pitchwheel'
            if 0 <= msg.channel < self.NUM_FADERS:
                # Echo back so the motor tracks correctly.
                if self._out_port:
                    try:
                        self._out_port.send(msg.copy())
                    except Exception:
                        pass
                norm = (msg.pitch + 8192) / 16383.0
                self.fader_moved.emit(msg.channel, max(0.0, min(1.0, norm)))

        elif msg.type == 'control_change':
            cc_lo = self.FADER_CC_BASE
            cc_hi = self.FADER_CC_BASE + self.NUM_FADERS  # exclusive
            if msg.control < cc_lo or msg.control >= cc_hi:
                return  # touch sensors (CC 101+) and unrelated CCs — ignore
            self._midi_mode = 'cc'
            # Echo back — required for the motor to report smooth intermediate values.
            if self._out_port:
                try:
                    self._out_port.send(msg.copy())
                except Exception:
                    pass
            fader_idx = msg.control - self.FADER_CC_BASE
            norm = msg.value / 127.0
            self.fader_moved.emit(fader_idx, max(0.0, min(1.0, norm)))

    # ── MIDI output ───────────────────────────────────────────────────────────

    def send_fader(self, fader_idx: int, normalized: float):
        """Move motorized fader to position (0.0 = bottom, 1.0 = top)."""
        if not self._out_port or not self._connected or self._midi_mode is None:
            return
        norm = max(0.0, min(1.0, normalized))
        try:
            if self._midi_mode == 'pitchwheel':
                pitch = int(norm * 16383) - 8192
                self._out_port.send(
                    mido.Message('pitchwheel', channel=fader_idx, pitch=pitch))
            else:
                self._out_port.send(
                    mido.Message('control_change', channel=0,
                                 control=fader_idx + self.FADER_CC_BASE,
                                 value=int(norm * 127)))
        except Exception:
            pass

    # ── teardown ─────────────────────────────────────────────────────────────

    def close(self):
        self._scan_timer.stop()
        if hasattr(self, '_poll_timer') and self._poll_timer is not None:
            self._poll_timer.stop()
        self._disconnect()

    @property
    def is_connected(self):
        return self._connected


# ─────────────────────────────────────────────────────────────────────────────

class SliderSlot(QGroupBox):
    """
    Generic vertical slider, initially unassigned.
    Drop a command field from CommandFieldList to assign what it controls.
    Right-click to unassign.
    """

    _PLACEHOLDER = "— drop field here —"

    def __init__(self, on_drop=None, is_field_allowed=None,
                 on_assigned=None, on_unassigned=None, parent=None):
        super().__init__(SliderSlot._PLACEHOLDER, parent)
        self._field: str | None = None
        self._float_mode        = False
        self._scale: float      = 1.0
        self._on_drop          = on_drop           # (key) -> (min,max,default) | None
        self._is_field_allowed = is_field_allowed  # (key) -> bool
        self._on_assigned      = on_assigned       # (key) -> None
        self._on_unassigned    = on_unassigned     # (key) -> None
        self._midi_updating    = False             # suppress MIDI echo while setting from MIDI
        self._on_user_changed  = None              # (normalized: float) -> None — set by DynoWindow
        self.setAcceptDrops(True)
        self.setMinimumWidth(110)

        _SPIN_LO = -(2 ** 30)
        _SPIN_HI =  (2 ** 30) - 1

        # Max spinbox
        self._max_spin = QSpinBox()
        self._max_spin.setKeyboardTracking(False)
        self._max_spin.setRange(_SPIN_LO, _SPIN_HI)
        self._max_spin.setValue(1000)
        self._max_spin.setPrefix("Max: ")

        # Vertical slider
        self._slider = QSlider(Qt.Vertical)
        self._slider.setRange(-1000, 1000)
        self._slider.setValue(0)
        self._slider.setTickInterval(500)
        self._slider.setTickPosition(QSlider.TicksRight)
        self._slider.setMinimumHeight(90)

        # Min spinbox
        self._min_spin = QSpinBox()
        self._min_spin.setKeyboardTracking(False)
        self._min_spin.setRange(_SPIN_LO, _SPIN_HI)
        self._min_spin.setValue(-1000)
        self._min_spin.setPrefix("Min: ")

        # Exact entry spinbox (integer mode)
        self._exact_spin = QSpinBox()
        self._exact_spin.setKeyboardTracking(False)
        self._exact_spin.setRange(-1000, 1000)
        self._exact_spin.setValue(0)

        # Float spinbox (gain mode — replaces slider+exact_spin)
        self._float_spin = QDoubleSpinBox()
        self._float_spin.setKeyboardTracking(False)
        self._float_spin.setRange(0.0, 20.0)
        self._float_spin.setSingleStep(0.000001)
        self._float_spin.setDecimals(6)
        self._float_spin.setValue(0.0)
        self._float_spin.wheelEvent = lambda e: e.ignore()
        self._float_spin.hide()
        self._committed_float: float = 0.0

        # Scale display (shows ×1,000 or ×1,000,000 after a drop)
        self._scale_label = QLabel("")
        self._scale_label.setAlignment(Qt.AlignCenter)

        # Value display
        self._val_label = QLabel("0")
        self._val_label.setAlignment(Qt.AlignCenter)

        # Clear button
        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self._unassign)

        col = QVBoxLayout(self)
        col.addWidget(self._scale_label)
        col.addWidget(self._max_spin)
        col.addWidget(self._slider, 1, Qt.AlignHCenter)
        col.addWidget(self._float_spin)
        col.addWidget(self._min_spin)
        col.addWidget(QLabel("Exact:"))
        col.addWidget(self._exact_spin)
        col.addWidget(self._val_label)
        col.addWidget(self._clear_btn)

        # Signal wiring
        self._max_spin.valueChanged.connect(self._on_max_changed)
        self._min_spin.valueChanged.connect(self._on_min_changed)
        self._slider.valueChanged.connect(self._on_slider_moved)
        self._exact_spin.valueChanged.connect(self._on_exact_changed)
        self._float_spin.valueChanged.connect(self._on_float_changed)

        self._set_controls_enabled(False)

    # ── range changes ─────────────────────────────────────────────────────────

    def _on_max_changed(self, v: int):
        if v < self._min_spin.value():
            self._min_spin.blockSignals(True)
            self._min_spin.setValue(v)
            self._min_spin.blockSignals(False)
        self._slider.setMaximum(v)
        self._exact_spin.setMaximum(v)

    def _on_min_changed(self, v: int):
        if v > self._max_spin.value():
            self._max_spin.blockSignals(True)
            self._max_spin.setValue(v)
            self._max_spin.blockSignals(False)
        self._slider.setMinimum(v)
        self._exact_spin.setMinimum(v)

    # ── value sync ────────────────────────────────────────────────────────────

    def _on_slider_moved(self, v: int):
        self._val_label.setText(f"{v / self._scale:.3f}")
        self._exact_spin.blockSignals(True)
        self._exact_spin.setValue(v)
        self._exact_spin.blockSignals(False)
        if not self._midi_updating and self._on_user_changed and self._field is not None:
            lo, hi = self._slider.minimum(), self._slider.maximum()
            norm = (v - lo) / (hi - lo) if hi != lo else 0.0
            self._on_user_changed(max(0.0, min(1.0, norm)))

    def _on_exact_changed(self, v: int):
        self._slider.blockSignals(True)
        self._slider.setValue(v)
        self._slider.blockSignals(False)
        self._val_label.setText(f"{v / self._scale:.3f}")

    def _on_float_changed(self, v: float):
        self._committed_float = v
        self._val_label.setText(f"{v / self._scale:.6f}")

    # ── float mode ────────────────────────────────────────────────────────────

    def set_float_mode(self, lo: float, hi: float, default: float):
        """Switch to float spinbox mode (for gain fields)."""
        self._float_mode = True
        self._slider.hide()
        self._max_spin.hide()
        self._min_spin.hide()
        self._exact_spin.hide()
        self._float_spin.blockSignals(True)
        self._float_spin.setRange(lo, hi)
        self._float_spin.setValue(default)
        self._committed_float = default
        self._float_spin.blockSignals(False)
        self._val_label.setText(f"{default / self._scale:.6f}")
        self._float_spin.show()

    def clear_float_mode(self):
        """Switch back to integer slider mode."""
        self._float_mode = False
        self._float_spin.hide()
        self._slider.show()
        self._max_spin.show()
        self._min_spin.show()
        self._exact_spin.show()

    # ── drag / drop ───────────────────────────────────────────────────────────

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasFormat(CMD_MIME):
            key = bytes(ev.mimeData().data(CMD_MIME)).decode()
            # Allow if: this slot already has the field, OR it's not taken elsewhere.
            if key != self._field and self._is_field_allowed and \
                    not self._is_field_allowed(key):
                ev.ignore()
                return
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragMoveEvent(self, ev):
        ev.acceptProposedAction()

    def dropEvent(self, ev):
        key = bytes(ev.mimeData().data(CMD_MIME)).decode()
        self._assign(key)
        if self._on_drop:
            result = self._on_drop(key)
            if result is not None:
                lo, hi, default = result
                # Torque clamp fields use scale=1 000 (not 1 000 000) so the
                # label shows the value in physical units without micro-stepping.
                if key in TORQUE_CLAMP_FIELDS:
                    self._scale = 1_000.0
                elif key in GAIN_FIELDS:
                    self._scale = 1_000_000.0
                else:
                    self._scale = 1_000.0
                sc = self._scale
                self._scale_label.setText(f"×{int(sc):,}")
                lo_s, hi_s, def_s = lo * sc, hi * sc, default * sc
                self.clear_float_mode()
                for w in (self._min_spin, self._max_spin,
                          self._slider, self._exact_spin):
                    w.blockSignals(True)
                lo_i, hi_i, def_i = int(lo_s), int(hi_s), int(def_s)
                self._min_spin.setValue(lo_i)
                self._max_spin.setValue(hi_i)
                self._slider.setMinimum(lo_i)
                self._slider.setMaximum(hi_i)
                self._exact_spin.setMinimum(lo_i)
                self._exact_spin.setMaximum(hi_i)
                self._slider.setValue(def_i)
                self._exact_spin.setValue(def_i)
                self._val_label.setText(f"{def_i / sc:.3f}")
                for w in (self._min_spin, self._max_spin,
                          self._slider, self._exact_spin):
                    w.blockSignals(False)
                if self._on_user_changed and hi_i != lo_i:
                    norm = (def_i - lo_i) / (hi_i - lo_i)
                    self._on_user_changed(max(0.0, min(1.0, norm)))
        ev.acceptProposedAction()

    def contextMenuEvent(self, ev):
        if self._field is None:
            return
        menu = QMenu(self)
        act  = QAction("Unassign", self)
        act.triggered.connect(self._unassign)
        menu.addAction(act)
        menu.exec_(ev.globalPos())

    # ── assignment ────────────────────────────────────────────────────────────

    def _assign(self, key: str):
        if self._field == key:
            return  # No change — same field re-dropped
        if self._field is not None and self._on_unassigned:
            self._on_unassigned(self._field)   # Release previous field
        self._field = key
        label = next((l for k, l in COMMAND_FIELDS if k == key), key)
        self.setTitle(label)
        self._set_controls_enabled(True)
        if self._on_assigned:
            self._on_assigned(key)

    def _unassign(self):
        if self._on_user_changed and self._field is not None:
            self._on_user_changed(0.0)
        old = self._field
        self._field = None
        self._scale = 1.0
        self._scale_label.setText("")
        self.setTitle(SliderSlot._PLACEHOLDER)
        self.clear_float_mode()
        self._slider.setValue(0)
        self._set_controls_enabled(False)
        if old is not None and self._on_unassigned:
            self._on_unassigned(old)

    def _set_controls_enabled(self, enabled: bool):
        self._slider.setEnabled(enabled)
        self._max_spin.setEnabled(enabled)
        self._min_spin.setEnabled(enabled)
        self._exact_spin.setEnabled(enabled)
        self._float_spin.setEnabled(enabled)
        self._clear_btn.setEnabled(enabled)

    # ── public API ────────────────────────────────────────────────────────────

    @property
    def field(self) -> str | None:
        return self._field

    @property
    def value(self):
        if self._float_mode:
            return self._committed_float / self._scale
        return self._slider.value() / self._scale

    def set_from_midi(self, normalized: float):
        """Set slider position from a MIDI message without echoing back to MIDI."""
        if self._float_mode or self._field is None:
            return
        lo, hi = self._slider.minimum(), self._slider.maximum()
        v = max(lo, min(hi, int(lo + normalized * (hi - lo))))
        self._midi_updating = True
        self._slider.setValue(v)
        self._midi_updating = False

    @property
    def normalized_value(self) -> float:
        """Current slider position as 0.0-1.0 (bottom to top)."""
        lo, hi = self._slider.minimum(), self._slider.maximum()
        return (self._slider.value() - lo) / (hi - lo) if hi != lo else 0.0

    def zero(self):
        if self._float_mode:
            self._float_spin.setValue(0.0)
            self._committed_float = 0.0
        else:
            self._slider.setValue(0)

    def apply_limits(self, lo: float, hi: float):
        """Update slider range without resetting the current value (integer mode only)."""
        sc = self._scale
        if self._float_mode:
            self._float_spin.blockSignals(True)
            self._float_spin.setRange(lo * sc, hi * sc)
            self._float_spin.blockSignals(False)
            return
        lo_i, hi_i = int(lo * sc), int(hi * sc)
        for w in (self._min_spin, self._max_spin, self._slider, self._exact_spin):
            w.blockSignals(True)
        self._min_spin.setValue(lo_i)
        self._max_spin.setValue(hi_i)
        self._slider.setMinimum(lo_i)
        self._slider.setMaximum(hi_i)
        self._exact_spin.setMinimum(lo_i)
        self._exact_spin.setMaximum(hi_i)
        for w in (self._min_spin, self._max_spin, self._slider, self._exact_spin):
            w.blockSignals(False)

    def apply_value(self, val: float):
        """Update the displayed value from an external source (no user signal emitted)."""
        sc = self._scale
        if self._float_mode:
            self._committed_float = val * sc   # keep slot.value in sync
            self._float_spin.blockSignals(True)
            self._float_spin.setValue(val * sc)
            self._float_spin.blockSignals(False)
            self._val_label.setText(f"{val:.6f}")
            return
        val_i = int(val * sc)
        val_i = max(self._slider.minimum(), min(self._slider.maximum(), val_i))
        for w in (self._slider, self._exact_spin):
            w.blockSignals(True)
        self._slider.setValue(val_i)
        self._exact_spin.setValue(val_i)
        self._val_label.setText(f"{val_i / sc:.3f}")
        for w in (self._slider, self._exact_spin):
            w.blockSignals(False)


# ─────────────────────────────────────────────────────────────────────────────
# Spinbox slot (drop target)
# ─────────────────────────────────────────────────────────────────────────────

class SpinboxSlot(QGroupBox):
    """
    A single labelled QDoubleSpinBox drop target.
    Drop a command field from CommandFieldList to assign what it controls.
    Right-click to unassign.
    Values apply on Enter / focus-out (keyboardTracking=False + _committed).
    """

    _PLACEHOLDER = "— drop field —"

    def __init__(self, on_drop=None, is_field_allowed=None,
                 on_assigned=None, on_unassigned=None, parent=None):
        super().__init__(SpinboxSlot._PLACEHOLDER, parent)
        self._field: str | None = None
        self._on_drop          = on_drop
        self._is_field_allowed = is_field_allowed
        self._on_assigned      = on_assigned
        self._on_unassigned    = on_unassigned
        self._committed: float = 0.0
        self.setAcceptDrops(True)
        self.setMinimumWidth(90)

        self._spin = QDoubleSpinBox()
        self._spin.setRange(-1e9, 1e9)
        self._spin.setDecimals(4)
        self._spin.setSingleStep(1.0)
        self._spin.setValue(0.0)
        self._spin.setKeyboardTracking(False)
        self._spin.setEnabled(False)

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.setEnabled(False)
        self._clear_btn.clicked.connect(self._unassign)

        lay = QVBoxLayout(self)
        lay.addWidget(self._spin)
        lay.addWidget(self._clear_btn)

        self._spin.valueChanged.connect(self._on_value_changed)

    def _on_value_changed(self, v: float):
        self._committed = v

    # ── drag / drop ───────────────────────────────────────────────────────────

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasFormat(CMD_MIME):
            key = bytes(ev.mimeData().data(CMD_MIME)).decode()
            if key != self._field and self._is_field_allowed and \
                    not self._is_field_allowed(key):
                ev.ignore()
                return
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragMoveEvent(self, ev):
        ev.acceptProposedAction()

    def dropEvent(self, ev):
        key = bytes(ev.mimeData().data(CMD_MIME)).decode()
        self._assign(key)
        if self._on_drop:
            result = self._on_drop(key)
            if result is not None:
                lo, hi, default = result
                is_gain = key in GAIN_FIELDS
                self._spin.blockSignals(True)
                self._spin.setDecimals(6 if is_gain else 3)
                self._spin.setSingleStep(0.000001 if is_gain else 0.001)
                self._spin.setRange(lo, hi)
                self._spin.setValue(float(default))
                self._committed = float(default)
                self._spin.blockSignals(False)
        ev.acceptProposedAction()

    def contextMenuEvent(self, ev):
        if self._field is None:
            return
        menu = QMenu(self)
        act  = QAction("Unassign", self)
        act.triggered.connect(self._unassign)
        menu.addAction(act)
        menu.exec_(ev.globalPos())

    # ── assignment ────────────────────────────────────────────────────────────

    def _assign(self, key: str):
        if self._field == key:
            return
        if self._field is not None and self._on_unassigned:
            self._on_unassigned(self._field)
        self._field = key
        label = next((l for k, l in COMMAND_FIELDS if k == key), key)
        self.setTitle(label)
        self._spin.setEnabled(True)
        self._clear_btn.setEnabled(True)
        if self._on_assigned:
            self._on_assigned(key)

    def _unassign(self):
        old = self._field
        self._field = None
        self.setTitle(SpinboxSlot._PLACEHOLDER)
        self._spin.blockSignals(True)
        self._spin.setValue(0.0)
        self._committed = 0.0
        self._spin.blockSignals(False)
        self._spin.setEnabled(False)
        self._clear_btn.setEnabled(False)
        if old is not None and self._on_unassigned:
            self._on_unassigned(old)

    # ── public API ────────────────────────────────────────────────────────────

    @property
    def field(self) -> str | None:
        return self._field

    @property
    def value(self) -> float:
        return self._committed

    def zero(self):
        self._spin.blockSignals(True)
        self._spin.setValue(0.0)
        self._committed = 0.0
        self._spin.blockSignals(False)

    def apply_limits(self, lo: float, hi: float):
        self._spin.blockSignals(True)
        self._spin.setRange(lo, hi)
        self._spin.blockSignals(False)

    def apply_value(self, val: float):
        """Update displayed value without triggering committed callback."""
        self._spin.blockSignals(True)
        self._spin.setValue(val)
        self._committed = val
        self._spin.blockSignals(False)


# ─────────────────────────────────────────────────────────────────────────────
# Scripting panel
# ─────────────────────────────────────────────────────────────────────────────

class ScriptingPanel(QGroupBox):
    """
    Right-side panel for selecting and running Python test scripts.

    Script protocol — each script file must define:
        PARAMS = {"speed": 100, "duration_s": 5.0, ...}  # param name → default
        def run(params: dict, commander: DynoCommander, stop_event: threading.Event):
            ...  # check stop_event.is_set() regularly to support abort
    """

    def __init__(self, commander, scripts_dir: str,
                 on_script_active=None, get_modes=None, parent=None):
        super().__init__("Test Scripts", parent)
        self._commander        = commander
        self._scripts_dir      = scripts_dir
        self._runner           = ScriptRunner()
        self._param_widgets: dict[str, QDoubleSpinBox | QSpinBox | QComboBox] = {}
        self._current_mod      = None
        self._on_script_active = on_script_active  # callable(bool) | None
        self._get_modes        = get_modes          # callable() → (main_mode, dut_mode) | None
        self._sdo_auto_pending: list = []   # [(param_name, val)] written by ROS thread
        self._sdo_auto_read_ok: set = set() # params successfully populated

        # Drain _sdo_auto_pending in the Qt main thread
        self._sdo_drain_timer = QTimer(self)
        self._sdo_drain_timer.timeout.connect(self._drain_sdo_auto_pending)
        self._sdo_drain_timer.start(50)

        # Retry SDO auto-reads every 3 s until all params are populated
        self._sdo_retry_timer = QTimer(self)
        self._sdo_retry_timer.timeout.connect(self._retry_sdo_auto_reads)
        self._sdo_retry_timer.start(3000)

        # ── Script selector ───────────────────────────────────────────────────
        self._script_combo = QComboBox()
        self._script_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._script_combo.currentIndexChanged.connect(self._on_script_selected)

        refresh_btn = QPushButton("↻")
        refresh_btn.setFixedWidth(28)
        refresh_btn.setToolTip("Rescan scripts folder")
        refresh_btn.clicked.connect(self._scan_scripts)

        selector_row = QHBoxLayout()
        selector_row.addWidget(self._script_combo, 1)
        selector_row.addWidget(refresh_btn)

        # ── Parameters area ───────────────────────────────────────────────────
        self._params_form   = QFormLayout()
        self._params_form.setSpacing(4)
        params_widget = QWidget()
        params_widget.setLayout(self._params_form)

        self._params_scroll = QScrollArea()
        self._params_scroll.setWidget(params_widget)
        self._params_scroll.setWidgetResizable(True)
        self._params_scroll.setMinimumHeight(80)
        self._params_scroll.setMaximumHeight(200)

        # ── Timing ────────────────────────────────────────────────────────────
        self._pre_settle_spin = QDoubleSpinBox()
        self._pre_settle_spin.setRange(0, 5000)
        self._pre_settle_spin.setSingleStep(10)
        self._pre_settle_spin.setSuffix(" ms")
        self._pre_settle_spin.setValue(100)
        self._pre_settle_spin.setToolTip(
            "Wait applied after each preamble step:\n"
            "  zero→enable, enable→mode, mode→script start")

        self._post_settle_spin = QDoubleSpinBox()
        self._post_settle_spin.setRange(0, 5000)
        self._post_settle_spin.setSingleStep(10)
        self._post_settle_spin.setSuffix(" ms")
        self._post_settle_spin.setValue(100)
        self._post_settle_spin.setToolTip(
            "Wait applied after each epilogue step:\n"
            "  zero setpoints, then disable drives")

        timing_row = QHBoxLayout()
        timing_row.addWidget(QLabel("Pre:"))
        timing_row.addWidget(self._pre_settle_spin)
        timing_row.addSpacing(6)
        timing_row.addWidget(QLabel("Post:"))
        timing_row.addWidget(self._post_settle_spin)

        # ── Buttons ───────────────────────────────────────────────────────────
        self._run_btn = QPushButton("Run Test")
        self._run_btn.setStyleSheet("background-color: #44cc44; font-weight: bold;")
        self._run_btn.clicked.connect(self._run_script)

        self._abort_btn = QPushButton("Abort")
        self._abort_btn.setStyleSheet("background-color: #cc4444; font-weight: bold;")
        self._abort_btn.setEnabled(False)
        self._abort_btn.clicked.connect(self._abort_script)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self._run_btn)
        btn_row.addWidget(self._abort_btn)

        # ── Actuator serial number ────────────────────────────────────────────
        self._serial_number_edit = QLineEdit()
        self._serial_number_edit.setPlaceholderText("Enter serial number…")
        sn_row = QHBoxLayout()
        sn_row.addWidget(QLabel("Actuator S/N:"))
        sn_row.addWidget(self._serial_number_edit)

        # ── Output log ────────────────────────────────────────────────────────
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMinimumHeight(80)
        self._log.setFont(QFont("Monospace", 8))
        self._log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # ── Layout ────────────────────────────────────────────────────────────
        lay = QVBoxLayout(self)
        lay.addLayout(selector_row)
        lay.addWidget(QLabel("Parameters:"))
        lay.addWidget(self._params_scroll)
        lay.addLayout(timing_row)
        lay.addLayout(btn_row)
        lay.addLayout(sn_row)
        lay.addWidget(QLabel("Output:"))
        lay.addWidget(self._log, 1)

        self._scan_scripts()

    # ── Script discovery ──────────────────────────────────────────────────────

    def _scan_scripts(self):
        paths = _discover_scripts(self._scripts_dir)
        prev  = self._script_combo.currentData()
        self._script_combo.blockSignals(True)
        self._script_combo.clear()
        if not paths:
            self._script_combo.addItem(f"(no scripts in {self._scripts_dir})", None)
        for p in paths:
            self._script_combo.addItem(os.path.basename(p), p)
        # Restore selection if still present
        idx = next((i for i in range(self._script_combo.count())
                    if self._script_combo.itemData(i) == prev), 0)
        self._script_combo.setCurrentIndex(idx)
        self._script_combo.blockSignals(False)
        self._on_script_selected(self._script_combo.currentIndex())

    def _on_script_selected(self, idx: int):
        path = self._script_combo.itemData(idx)
        self._current_mod = None
        self._sdo_auto_read_ok.clear()
        self._clear_params()
        if not path:
            return
        try:
            mod = _load_script_module(path)
            self._current_mod = mod
            params = getattr(mod, "PARAMS", {})
            self._build_params(params)
            self._trigger_sdo_auto_reads()
            # Re-trigger SDO reads when the drive dropdown changes
            auto_reads = getattr(mod, "SDO_AUTO_READS", {})
            for spec in auto_reads.values():
                drive_param = spec.get("drive_param")
                if drive_param and drive_param in self._param_widgets:
                    w = self._param_widgets[drive_param]
                    if isinstance(w, QComboBox):
                        w.currentTextChanged.connect(
                            lambda _t, m=mod: self._trigger_sdo_auto_reads(m))
        except Exception as e:
            self._log_line(f"[load error] {e}")

    def _trigger_sdo_auto_reads(self, mod=None):
        if mod is None:
            mod = self._current_mod
        if mod is None:
            return
        auto_reads = getattr(mod, "SDO_AUTO_READS", {})
        for param_name, spec in auto_reads.items():
            if param_name not in self._param_widgets:
                continue
            drive = "main"
            drive_param = spec.get("drive_param")
            if drive_param and drive_param in self._param_widgets:
                w = self._param_widgets[drive_param]
                if isinstance(w, QComboBox):
                    drive = w.currentText()

            def _on_response(resp, pname=param_name, s=spec, m=mod):
                if self._current_mod is not m:
                    return
                if not (resp.get("op") == "read" and resp.get("success", False)):
                    return
                raw = int(resp.get("value", 0))
                transform = s.get("transform")
                val = int(transform(raw)) if transform else raw
                self._sdo_auto_pending.append((pname, val))  # GIL-safe append

            self._commander.register_sdo_one_shot(
                spec["index"], spec["subindex"], _on_response)
            self._commander.request_sdo(
                drive, "read", spec["index"], spec["subindex"], spec["size"])

    def _drain_sdo_auto_pending(self):
        while self._sdo_auto_pending:
            pname, val = self._sdo_auto_pending.pop(0)
            w = self._param_widgets.get(pname)
            if w and isinstance(w, QSpinBox):
                w.setValue(val)
                self._sdo_auto_read_ok.add(pname)

    def _retry_sdo_auto_reads(self):
        mod = self._current_mod
        if mod is None:
            return
        auto_reads = getattr(mod, "SDO_AUTO_READS", {})
        if not auto_reads:
            return
        # Only retry params that haven't been successfully populated yet
        pending = [p for p in auto_reads if p not in self._sdo_auto_read_ok]
        if pending:
            self._trigger_sdo_auto_reads(mod)

    # ── Parameter form ────────────────────────────────────────────────────────

    def _clear_params(self):
        while self._params_form.rowCount():
            self._params_form.removeRow(0)
        self._param_widgets.clear()

    def _build_params(self, params: dict):
        self._clear_params()
        for name, default in params.items():
            if isinstance(default, float):
                w = QDoubleSpinBox()
                w.setKeyboardTracking(False)
                w.setDecimals(4)
                w.setRange(-1e9, 1e9)
                w.setSingleStep(0.1)
                w.setValue(default)
            elif isinstance(default, list):
                w = QComboBox()
                for item in default:
                    w.addItem(str(item))
            else:
                w = QSpinBox()
                w.setKeyboardTracking(False)
                w.setRange(-2**30, 2**30 - 1)
                w.setValue(int(default))
            self._param_widgets[name] = w
            self._params_form.addRow(name + ":", w)

    def _collect_params(self) -> dict:
        result = {}
        for name, w in self._param_widgets.items():
            if isinstance(w, QComboBox):
                result[name] = w.currentText()
            else:
                result[name] = w.value()
        return result

    # ── Serial number file ────────────────────────────────────────────────────

    def _write_sn_file(self, script_stem: str, sn: str):
        """Write actuator_serial_number.txt into the test log directory.

        The bridge creates the directory asynchronously after pulse_save_log(),
        so we retry every 200 ms until it appears (up to ~3 s total).
        """
        if not sn:
            return
        import glob, datetime, re, pwd as _pwd
        repo_root  = os.path.abspath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
        date_str   = datetime.date.today().strftime("%Y-%m-%d")
        safe_stem  = re.sub(r"[^a-zA-Z0-9]+", "_", script_stem).rstrip("_")
        pattern    = os.path.join(repo_root, "test_data_log", date_str,
                                  f"*_{safe_stem}")
        attempts   = [0]

        def _try_write():
            dirs = sorted(glob.glob(pattern))
            if not dirs:
                attempts[0] += 1
                if attempts[0] < 15:   # give up after ~3 s
                    QTimer.singleShot(200, _try_write)
                return
            path = os.path.join(dirs[-1], "actuator_serial_number.txt")
            try:
                with open(path, "w") as f:
                    f.write(f'actuator_serial_number = "{sn}"\n')
                sudo_user = (os.environ.get("DYNO_ORIGINAL_USER")
                             or os.environ.get("SUDO_USER", ""))
                if sudo_user and sudo_user != "root":
                    pw = _pwd.getpwnam(sudo_user)
                    os.chown(path, pw.pw_uid, pw.pw_gid)
            except Exception as exc:
                print(f"[TestScriptPanel] serial number file write failed: {exc}")

        QTimer.singleShot(300, _try_write)

    # ── Run / abort ───────────────────────────────────────────────────────────

    def _run_script(self):
        if self._current_mod is None or self._runner.is_running:
            return

        # Zero drives and set mode_of_operation to 0 before handing off to script
        self._commander.set_command(
            numeric     = {k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS},
            main_enable = False,
            dut_enable  = False,
            main_mode   = 0,
            dut_mode    = 0,
        )
        # Pause GUI push — script now has exclusive control of set_command()
        if self._on_script_active:
            self._on_script_active(True)

        script_stem = os.path.splitext(self._script_combo.currentText())[0]
        self._commander.set_script_name(script_stem)
        self._commander.pulse_save_log()
        self._write_sn_file(script_stem, self._serial_number_edit.text().strip())

        params = self._collect_params()
        self._log.clear()
        self._log_line(f"[run] {self._script_combo.currentText()}")
        self._log_line(f"[params] {params}")

        self._run_btn.setEnabled(False)
        self._abort_btn.setEnabled(True)

        main_mode, dut_mode = self._get_modes() if self._get_modes else (9, 9)
        pre_s  = self._pre_settle_spin.value()  / 1000.0
        post_s = self._post_settle_spin.value() / 1000.0

        def _on_done(success: bool, msg: str):
            QTimer.singleShot(0, lambda: self._on_script_done(success, msg))

        try:
            self._runner.run(
                path          = self._script_combo.currentData(),
                params        = params,
                commander     = self._commander,
                on_done       = _on_done,
                on_log        = self._log_line,
                main_mode     = main_mode,
                dut_mode      = dut_mode,
                pre_settle_s  = pre_s,
                post_settle_s = post_s,
            )
        except Exception as exc:
            if self._on_script_active:
                self._on_script_active(False)
            self._run_btn.setEnabled(True)
            self._abort_btn.setEnabled(False)
            self._log_line(f"[error] Could not start script: {exc}")
            return

        # Thread started — poll for completion so UI is restored reliably
        # regardless of whether QTimer.singleShot from the background thread fires.
        QTimer.singleShot(100, self._poll_thread_done)

    def _abort_script(self):
        self._runner.abort()
        self._log_line("[abort] stop requested")
        # Immediately restore GUI control (Main Enable, sliders) without waiting
        # for the script thread to finish sleeping.
        # _poll_thread_done (started by _run_script) will re-enable Run Test
        # once the thread actually exits.
        if self._on_script_active:
            self._on_script_active(False)
        self._abort_btn.setEnabled(False)

    def _poll_thread_done(self) -> None:
        """Poll every 100 ms until the background script thread has exited.
        Authoritative UI restorer for both natural completion and abort."""
        if self._runner.is_running:
            QTimer.singleShot(100, self._poll_thread_done)
        else:
            # Idempotent — _on_script_active(False) may already be False (abort path).
            if self._on_script_active:
                self._on_script_active(False)
            self._run_btn.setEnabled(True)
            self._abort_btn.setEnabled(False)
            # End the named log window — runs here (not in _on_script_done) because
            # QTimer.singleShot from a background thread is not reliable.
            self._commander.set_script_name("")
            self._commander.pulse_save_log()
            threading.Thread(target=self._convert_recent_logs, daemon=True).start()

    def _convert_recent_logs(self) -> None:
        """Convert recently-closed .csv.gz logs to Parquet for fast column access."""
        import time, pwd as _pwd
        from pathlib import Path
        time.sleep(5)   # wait for bridge drain thread to close and flush the file
        try:
            import pandas as pd
            import pyarrow  # noqa: F401
        except ImportError as exc:
            self._log_line(f"[Parquet] Skipped (pip install pyarrow): {exc}")
            return
        repo_root = os.path.abspath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
        log_root  = Path(repo_root) / "test_data_log"
        cutoff    = time.time() - 120   # only files closed in the last 2 minutes
        sudo_user = (os.environ.get("DYNO_ORIGINAL_USER")
                     or os.environ.get("SUDO_USER", ""))
        for gz in sorted(log_root.glob("*/*/*.csv.gz")):
            pq_path = gz.parent / "dyno_pdo.parquet"
            try:
                mtime = gz.stat().st_mtime
            except OSError:
                continue
            if pq_path.exists() or mtime <= cutoff:
                continue
            try:
                self._log_line(f"[Parquet] Converting {gz.parent.name}…")
                df = pd.read_csv(str(gz))
                df.to_parquet(str(pq_path), index=False, compression="snappy")
                if sudo_user and sudo_user != "root":
                    try:
                        pw = _pwd.getpwnam(sudo_user)
                        os.chown(str(pq_path), pw.pw_uid, pw.pw_gid)
                    except Exception:
                        pass
                self._log_line("[Parquet] Done")
            except Exception as exc:
                self._log_line(f"[Parquet] Error: {exc}")

    def _on_script_done(self, success: bool, msg: str):
        """Log result message — UI state and log rotation handled by _poll_thread_done."""
        status = "OK" if success else "ERROR"
        self._log_line(f"[{status}] {msg.strip()}")

    # ── Log helper ────────────────────────────────────────────────────────────

    def _log_line(self, text: str):
        # Safe to call from any thread via QTimer.singleShot
        def _append():
            self._log.append(text)
            self._log.verticalScrollBar().setValue(
                self._log.verticalScrollBar().maximum())
        QTimer.singleShot(0, _append)


# ─────────────────────────────────────────────────────────────────────────────
# Qt main window
# ─────────────────────────────────────────────────────────────────────────────

class DynoWindow(QMainWindow):

    def __init__(self, commander: DynoCommander, scripts_dir: str = TEST_SCRIPTS_DIR,
                 bridge_proc=None, bridge_args=None):
        super().__init__()
        self._cmd            = commander
        self._scripts_dir    = scripts_dir
        self._bridge_proc    = bridge_proc
        self._bridge_args    = bridge_args
        self._main_enabled   = False
        self._dut_enabled    = False
        self._hold_output1   = False
        self._script_running = False
        self._ch1_scale: int = 200   # Nm — matches Elm3002Adapter ch1 default
        self._ch2_scale: int = 20    # Nm — matches Elm3002Adapter ch2 default

        def _fg_default():
            return {"enable": False, "waveform": 0, "control_type": 0,
                    "amplitude": 0.0, "frequency": 1.0, "offset": 0.0, "phase": 0.0,
                    "chirp_f_low": 0.1, "chirp_f_high": 10.0, "chirp_dur": 10.0}
        self._fg_state       = {"main": _fg_default(), "dut": _fg_default()}

        self.setWindowTitle("Dyno Control")
        self._build_ui()
        commander.set_limits_callback(self._on_limits_updated)

        self._midi_bridge: XTouchMidiBridge | None = None
        if _MIDO_AVAILABLE:
            self._midi_bridge = XTouchMidiBridge(self)
            self._midi_bridge.fader_moved.connect(self._on_fader_moved)
            self._midi_bridge.connection_changed.connect(self._on_midi_connection)

        self._fault_hist_state: dict | None = None  # active fault-history read state
        self._sdo_pending_time: float | None = None  # monotonic time of last SDO request
        self._ecat_pending: str | None = None        # "pre_op", "main_store", "dut_store"

        self._sdo_poll_timer = QTimer(self)
        self._sdo_poll_timer.timeout.connect(self._poll_sdo_response)
        self._sdo_poll_timer.start(50)   # 20 Hz

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._push_command)
        self._timer.start(5)   # 200 Hz
        self._torque_display_tick = 0

        self._error_timer = QTimer(self)
        self._error_timer.timeout.connect(self._refresh_all_errors)
        self._error_timer.start(500)   # 2 Hz

        self._bridge_poll_timer = QTimer(self)
        self._bridge_poll_timer.setInterval(500)
        self._bridge_poll_timer.timeout.connect(self._poll_bridge)
        self._bridge_poll_timer.start()
        self._update_bridge_status()

    def showEvent(self, event):
        super().showEvent(event)
        if not hasattr(self, '_initially_sized'):
            self._initially_sized = True
            QTimer.singleShot(50, self._apply_initial_window_size)

    def _target_startup_width(self) -> int:
        screen = self.screen() or QApplication.primaryScreen()
        if screen is None:
            return STARTUP_WINDOW_WIDTH
        # Keep a little horizontal headroom for the window manager decorations.
        return min(STARTUP_WINDOW_WIDTH, max(200, screen.availableGeometry().width() - 40))

    def _apply_initial_window_size(self):
        # Some window managers restore the last maximized state unless we
        # explicitly put the window back into normal mode before sizing it.
        if self.isMaximized():
            self.showNormal()
        self.resize(self._target_startup_width(), STARTUP_WINDOW_HEIGHT)
        self._fit_window_to_content()

    def _fit_window_to_content(self):
        """Resize height to content after first layout pass (actual sizes known)."""
        h = self.centralWidget().sizeHint().height()
        screen = self.screen() or QApplication.primaryScreen()
        if screen is not None:
            # Leave a little headroom for the window frame/title bar.
            h = min(h, max(200, screen.availableGeometry().height() - 360))
        # Do not let the one-time auto-fit grow the window taller than the
        # startup size; keeping it shorter lets the slider section settle to
        # the same compact layout you'd get after manually dragging the top edge.
        h = min(h, self.height())
        self.resize(self.width(), h)

    def _build_ui(self):
        # ── Left: command field list ───────────────────────────────────────────
        self._field_list  = CommandFieldList()
        self._field_search = QLineEdit()
        self._field_search.setPlaceholderText("Search fields…")
        self._field_search.setMaximumWidth(150)
        self._field_search.textChanged.connect(self._filter_command_fields)
        field_list_outer  = QWidget()
        field_list_outer_lay = QVBoxLayout(field_list_outer)
        field_list_outer_lay.setContentsMargins(0, 0, 0, 0)
        field_list_outer_lay.setSpacing(2)
        field_list_outer_lay.addWidget(self._field_search)
        field_list_outer_lay.addWidget(self._field_list)
        field_list_outer_lay.addStretch(1)

        # ── Centre: slider slots ───────────────────────────────────────────────
        self._assigned_fields: set[str] = set()
        self._slots = [
            SliderSlot(
                on_drop          = self._cmd.get_limits,
                is_field_allowed = lambda key: key not in self._assigned_fields,
                on_assigned      = lambda key: self._assigned_fields.add(key),
                on_unassigned    = lambda key: self._assigned_fields.discard(key),
            )
            for _ in range(NUM_SLOTS)
        ]
        for _idx, _slot in enumerate(self._slots):
            def _make_user_cb(_i):
                def _cb(norm):
                    if self._midi_bridge:
                        self._midi_bridge.send_fader(_i, norm)
                return _cb
            _slot._on_user_changed = _make_user_cb(_idx)

        slots_w   = QWidget()
        slots_lay = QHBoxLayout(slots_w)
        slots_lay.setSpacing(6)
        for slot in self._slots:
            slots_lay.addWidget(slot)
        self._slots_w = slots_w

        slots_outer     = QWidget()
        slots_outer_lay = QVBoxLayout(slots_outer)
        slots_outer_lay.setContentsMargins(0, 0, 0, 0)
        slots_outer_lay.setSpacing(0)
        slots_outer_lay.setAlignment(Qt.AlignTop)
        slots_outer_lay.addWidget(slots_w)

        # ── Spinbox rows (below sliders, full-width) ──────────────────────────
        self._spin_slots = [
            SpinboxSlot(
                on_drop          = self._cmd.get_limits,
                is_field_allowed = lambda key: key not in self._assigned_fields,
                on_assigned      = lambda key: self._assigned_fields.add(key),
                on_unassigned    = lambda key: self._assigned_fields.discard(key),
            )
            for _ in range(NUM_SPIN_SLOTS * NUM_SPIN_ROWS)
        ]
        spin_area     = QWidget()
        spin_area_lay = QVBoxLayout(spin_area)
        spin_area_lay.setContentsMargins(0, 0, 0, 0)
        spin_area_lay.setSpacing(4)
        for row in range(NUM_SPIN_ROWS):
            row_w   = QWidget()
            row_lay = QHBoxLayout(row_w)
            row_lay.setSpacing(6)
            row_lay.setContentsMargins(0, 0, 0, 0)
            for slot in self._spin_slots[row * NUM_SPIN_SLOTS:(row + 1) * NUM_SPIN_SLOTS]:
                row_lay.addWidget(slot)
            row_lay.addStretch()
            spin_area_lay.addWidget(row_w)

        self._spin_area = spin_area

        # ── Right panel: buttons + scripting side by side ─────────────────────
        right_w     = QWidget()
        right_outer = QVBoxLayout(right_w)
        right_outer.setContentsMargins(0, 0, 0, 0)
        right_outer.setSpacing(0)
        right_inner = QWidget()
        right_lay   = QHBoxLayout(right_inner)
        right_lay.setSpacing(6)
        right_lay.setContentsMargins(0, 0, 0, 0)

        # ── Buttons column ────────────────────────────────────────────────────
        btn_w   = QWidget()
        btn_lay = QVBoxLayout(btn_w)
        btn_lay.setSpacing(8)
        btn_w.setFixedWidth(200)

        self._main_enable_btn = QPushButton("Main Enable")
        self._main_enable_btn.setCheckable(True)
        self._main_enable_btn.setStyleSheet(
            "QPushButton:checked { background-color: #44cc44; font-weight: bold; }")
        self._main_enable_btn.clicked.connect(self._toggle_main)

        main_zero_btn = QPushButton("Main Zero")
        main_zero_btn.clicked.connect(self._main_zero)

        self._main_mode_combo = QComboBox()
        for label, _ in DS402_MODES:
            self._main_mode_combo.addItem(label)
        self._main_mode_combo.setCurrentIndex(
            next(i for i, (_, v) in enumerate(DS402_MODES) if v == DS402_DEFAULT_MODE))

        self._dut_enable_btn = QPushButton("DUT Enable")
        self._dut_enable_btn.setCheckable(True)
        self._dut_enable_btn.setStyleSheet(
            "QPushButton:checked { background-color: #44cc44; font-weight: bold; }")
        self._dut_enable_btn.clicked.connect(self._toggle_dut)

        dut_zero_btn = QPushButton("DUT Zero")
        dut_zero_btn.clicked.connect(self._dut_zero)

        self._dut_mode_combo = QComboBox()
        for label, _ in DS402_MODES:
            self._dut_mode_combo.addItem(label)
        self._dut_mode_combo.setCurrentIndex(
            next(i for i, (_, v) in enumerate(DS402_MODES) if v == DS402_DEFAULT_MODE))

        self._output1_btn = QPushButton("Hold Output 1")
        self._output1_btn.setCheckable(True)
        self._output1_btn.setStyleSheet(
            "QPushButton:checked { background-color: #44aaff; font-weight: bold; }")
        self._output1_btn.clicked.connect(self._toggle_output1)

        fault_btn = QPushButton("Fault Reset")
        fault_btn.setStyleSheet("background-color: #f0a000; font-weight: bold;")
        fault_btn.clicked.connect(self._fault_reset_pressed)

        btn_lay.addWidget(self._main_enable_btn)
        btn_lay.addWidget(main_zero_btn)
        btn_lay.addWidget(QLabel("Main Mode:"))
        btn_lay.addWidget(self._main_mode_combo)
        btn_lay.addSpacing(12)
        btn_lay.addWidget(self._dut_enable_btn)
        btn_lay.addWidget(dut_zero_btn)
        btn_lay.addWidget(QLabel("DUT Mode:"))
        btn_lay.addWidget(self._dut_mode_combo)
        btn_lay.addSpacing(12)
        btn_lay.addWidget(self._output1_btn)
        btn_lay.addWidget(fault_btn)
        btn_lay.addSpacing(12)

        btn_lay.addWidget(QLabel("Torque Ch1 Scale:"))
        self._ch1_scale_combo = QComboBox()
        for v in TORQUE_SCALE_OPTIONS:
            self._ch1_scale_combo.addItem(f"{v} Nm", v)
        self._ch1_scale_combo.setCurrentIndex(TORQUE_SCALE_OPTIONS.index(200))
        self._ch1_scale_combo.currentIndexChanged.connect(self._on_ch1_scale_changed)
        btn_lay.addWidget(self._ch1_scale_combo)

        zero_ch1_btn = QPushButton("Zero Ch1")
        zero_ch1_btn.setToolTip("Capture current Ch1 reading as zero offset")
        zero_ch1_btn.clicked.connect(self._cmd.pulse_torque_zero_ch1)
        btn_lay.addWidget(zero_ch1_btn)

        btn_lay.addWidget(QLabel("Torque Ch2 Scale:"))
        self._ch2_scale_combo = QComboBox()
        for v in TORQUE_SCALE_OPTIONS:
            self._ch2_scale_combo.addItem(f"{v} Nm", v)
        self._ch2_scale_combo.setCurrentIndex(TORQUE_SCALE_OPTIONS.index(20))
        self._ch2_scale_combo.currentIndexChanged.connect(self._on_ch2_scale_changed)
        btn_lay.addWidget(self._ch2_scale_combo)

        zero_ch2_btn = QPushButton("Zero Ch2")
        zero_ch2_btn.setToolTip("Capture current Ch2 reading as zero offset")
        zero_ch2_btn.clicked.connect(self._cmd.pulse_torque_zero_ch2)
        btn_lay.addWidget(zero_ch2_btn)

        btn_lay.addSpacing(12)

        save_log_btn = QPushButton("Save Log")
        save_log_btn.setToolTip(
            "Close the current CSV log and start a new file with a fresh timestamp")
        save_log_btn.clicked.connect(self._cmd.pulse_save_log)
        btn_lay.addWidget(save_log_btn)

        btn_lay.addSpacing(12)

        # ── Main drive fault section ──────────────────────────────────────────
        main_fault_group = QGroupBox("Main Drive Faults")
        main_fault_lay   = QVBoxLayout(main_fault_group)
        main_fault_lay.setSpacing(4)
        main_fault_lay.setContentsMargins(6, 6, 6, 6)

        main_al_row = QHBoxLayout()
        main_al_row.addWidget(QLabel("AL:"))
        self._main_al_label = QLabel("?")
        _mono = QFont("Monospace"); _mono.setPointSize(8)
        self._main_al_label.setFont(_mono)
        main_al_row.addWidget(self._main_al_label)
        main_al_row.addStretch()
        main_fault_lay.addLayout(main_al_row)

        main_fault_lay.addWidget(QLabel("Last Error:"))
        self._main_last_error = QTextEdit()
        self._main_last_error.setReadOnly(True)
        self._main_last_error.setLineWrapMode(QTextEdit.NoWrap)
        self._main_last_error.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._main_last_error.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._main_last_error.setFixedHeight(44)
        self._main_last_error.setPlaceholderText("—")
        main_fault_lay.addWidget(self._main_last_error)

        main_fault_history_btn = QPushButton("Read Fault History")
        main_fault_history_btn.clicked.connect(lambda: self._read_fault_history("main"))
        main_fault_lay.addWidget(main_fault_history_btn)

        self._main_fault_history = QTextEdit()
        self._main_fault_history.setReadOnly(True)
        self._main_fault_history.setFixedHeight(80)
        self._main_fault_history.setPlaceholderText("(no history)")
        main_fault_lay.addWidget(self._main_fault_history)

        btn_lay.addWidget(main_fault_group)
        btn_lay.addSpacing(8)

        # ── DUT fault section ─────────────────────────────────────────────────
        dut_fault_group = QGroupBox("DUT Faults")
        dut_fault_lay   = QVBoxLayout(dut_fault_group)
        dut_fault_lay.setSpacing(4)
        dut_fault_lay.setContentsMargins(6, 6, 6, 6)

        dut_al_row = QHBoxLayout()
        dut_al_row.addWidget(QLabel("AL:"))
        self._dut_al_label = QLabel("?")
        _mono2 = QFont("Monospace"); _mono2.setPointSize(8)
        self._dut_al_label.setFont(_mono2)
        dut_al_row.addWidget(self._dut_al_label)
        dut_al_row.addStretch()
        dut_fault_lay.addLayout(dut_al_row)

        dut_fault_lay.addWidget(QLabel("Last Error:"))
        self._dut_last_error = QTextEdit()
        self._dut_last_error.setReadOnly(True)
        self._dut_last_error.setLineWrapMode(QTextEdit.NoWrap)
        self._dut_last_error.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._dut_last_error.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._dut_last_error.setFixedHeight(44)
        self._dut_last_error.setPlaceholderText("—")
        dut_fault_lay.addWidget(self._dut_last_error)

        dut_fault_history_btn = QPushButton("Read Fault History")
        dut_fault_history_btn.clicked.connect(lambda: self._read_fault_history("dut"))
        dut_fault_lay.addWidget(dut_fault_history_btn)

        self._dut_fault_history = QTextEdit()
        self._dut_fault_history.setReadOnly(True)
        self._dut_fault_history.setFixedHeight(80)
        self._dut_fault_history.setPlaceholderText("(no history)")
        dut_fault_lay.addWidget(self._dut_fault_history)

        btn_lay.addWidget(dut_fault_group)
        btn_lay.addSpacing(8)

        # ── Inertia ───────────────────────────────────────────────────────────
        inertia_group = QGroupBox("Inertia")
        inertia_lay   = QVBoxLayout(inertia_group)
        inertia_lay.setSpacing(4)
        inertia_lay.setContentsMargins(6, 6, 6, 6)
        inertia_lay.addWidget(QLabel("Estimated (kg·m²):"))
        self._inertia_spin = QDoubleSpinBox()
        self._inertia_spin.setRange(0.0, 100.0)
        self._inertia_spin.setDecimals(6)
        self._inertia_spin.setSingleStep(0.001)
        self._inertia_spin.setKeyboardTracking(False)
        self._inertia_spin.setValue(0.000135)
        self._inertia_spin.valueChanged.connect(self._cmd.set_inertia)
        inertia_lay.addWidget(self._inertia_spin)
        btn_lay.addWidget(inertia_group)
        btn_lay.addSpacing(8)

        # ── JIP Parameters ────────────────────────────────────────────────────
        jip_group = QGroupBox("JIP Parameters")
        jip_lay   = QVBoxLayout(jip_group)
        jip_lay.setSpacing(4)
        jip_lay.setContentsMargins(6, 6, 6, 6)

        jip_lay.addWidget(QLabel("Hardstop Upper (rad):"))
        self._hardstop_upper_spin = QDoubleSpinBox()
        self._hardstop_upper_spin.setRange(-100.0, 100.0)
        self._hardstop_upper_spin.setDecimals(4)
        self._hardstop_upper_spin.setSingleStep(0.01)
        self._hardstop_upper_spin.setKeyboardTracking(False)
        self._hardstop_upper_spin.setValue(3.0)
        self._hardstop_upper_spin.valueChanged.connect(self._cmd.set_hardstop_pos_upper)
        jip_lay.addWidget(self._hardstop_upper_spin)

        jip_lay.addWidget(QLabel("Hardstop Lower (rad):"))
        self._hardstop_lower_spin = QDoubleSpinBox()
        self._hardstop_lower_spin.setRange(-100.0, 100.0)
        self._hardstop_lower_spin.setDecimals(4)
        self._hardstop_lower_spin.setSingleStep(0.01)
        self._hardstop_lower_spin.setKeyboardTracking(False)
        self._hardstop_lower_spin.setValue(-3.0)
        self._hardstop_lower_spin.valueChanged.connect(self._cmd.set_hardstop_pos_lower)
        jip_lay.addWidget(self._hardstop_lower_spin)

        jip_lay.addWidget(QLabel("Margin (rad):"))
        self._margin_spin = QDoubleSpinBox()
        self._margin_spin.setRange(0.0, 100.0)
        self._margin_spin.setDecimals(4)
        self._margin_spin.setSingleStep(0.01)
        self._margin_spin.setKeyboardTracking(False)
        self._margin_spin.setValue(0.1)
        self._margin_spin.valueChanged.connect(self._cmd.set_margin)
        jip_lay.addWidget(self._margin_spin)

        jip_lay.addSpacing(4)
        jip_lay.addWidget(QLabel("Torque Abs Max (SDO):"))
        self._torque_abs_max_label = QLabel("—")
        self._torque_abs_max_label.setStyleSheet("font-weight: bold;")
        jip_lay.addWidget(self._torque_abs_max_label)

        btn_lay.addWidget(jip_group)

        btn_lay.addStretch(1)

        # ── Scripting panel ───────────────────────────────────────────────────
        self._script_panel = ScriptingPanel(
            self._cmd, self._scripts_dir,
            on_script_active = self._on_script_active,
            get_modes        = lambda: (
                DS402_MODES[self._main_mode_combo.currentIndex()][1],
                DS402_MODES[self._dut_mode_combo.currentIndex()][1],
            ),
        )
        self._script_panel.setMinimumWidth(280)

        # ── SDO access panel ──────────────────────────────────────────────────
        _hex_validator = QRegularExpressionValidator(
            QRegularExpression("[0-9A-Fa-f]{0,8}"))

        sdo_group = QGroupBox("SDO Access")
        sdo_lay   = QVBoxLayout(sdo_group)
        sdo_lay.setSpacing(4)
        sdo_group.setFixedWidth(200)

        self._sdo_drive = QComboBox()
        self._sdo_drive.addItems(["main", "dut"])

        self._sdo_index = QLineEdit()
        self._sdo_index.setValidator(_hex_validator)
        self._sdo_index.setPlaceholderText("6040")
        self._sdo_index.setMaxLength(4)

        self._sdo_sub = QLineEdit()
        self._sdo_sub.setValidator(_hex_validator)
        self._sdo_sub.setPlaceholderText("00")
        self._sdo_sub.setMaxLength(2)

        self._sdo_size = QComboBox()
        self._sdo_size.addItems(["1 byte", "2 bytes", "4 bytes"])
        self._sdo_size.setCurrentIndex(1)  # default 2 bytes

        self._sdo_value = QLineEdit()
        self._sdo_value.setPlaceholderText("value")

        self._sdo_reg_info = QLabel("")
        self._sdo_reg_info.setFont(QFont("Monospace", 7))
        self._sdo_reg_info.setWordWrap(True)
        self._sdo_reg_info.setStyleSheet("color: #888;")

        self._sdo_index.textChanged.connect(self._sdo_lookup_register)
        self._sdo_sub.textChanged.connect(self._sdo_lookup_register)

        sdo_read_btn  = QPushButton("Read SDO")
        sdo_write_btn = QPushButton("Write SDO")
        sdo_read_btn.clicked.connect(self._sdo_read)
        sdo_write_btn.clicked.connect(self._sdo_write)
        sdo_btn_row = QHBoxLayout()
        sdo_btn_row.addWidget(sdo_read_btn)
        sdo_btn_row.addWidget(sdo_write_btn)

        self._sdo_feedback = QTextEdit()
        self._sdo_feedback.setReadOnly(True)
        self._sdo_feedback.setFont(QFont("Monospace", 8))
        self._sdo_feedback.setMinimumHeight(50)
        self._sdo_feedback.setPlaceholderText("(result)")

        def _sdo_hex_row(label: str, widget: QLineEdit) -> QHBoxLayout:
            row = QHBoxLayout()
            row.addWidget(QLabel(label))
            row.addWidget(QLabel("0x"))
            row.addWidget(widget, 1)
            return row

        sdo_lay.addWidget(QLabel("Drive:"))
        sdo_lay.addWidget(self._sdo_drive)
        sdo_lay.addLayout(_sdo_hex_row("Index:", self._sdo_index))
        sdo_lay.addLayout(_sdo_hex_row("Sub:",   self._sdo_sub))
        sdo_lay.addWidget(self._sdo_reg_info)
        sdo_lay.addWidget(QLabel("Size:"))
        sdo_lay.addWidget(self._sdo_size)
        sdo_lay.addWidget(QLabel("Value:"))
        sdo_lay.addWidget(self._sdo_value)
        sdo_lay.addLayout(sdo_btn_row)
        sdo_lay.addWidget(QLabel("Response:"))
        sdo_lay.addWidget(self._sdo_feedback, 1)

        # ── EtherCAT Control panel ────────────────────────────────────────────
        def _resp_label() -> QLabel:
            lbl = QLabel("")
            lbl.setFont(QFont("Monospace", 7))
            lbl.setWordWrap(True)
            lbl.setStyleSheet("color: #888;")
            return lbl

        ecat_group = QGroupBox("EtherCAT Control")
        ecat_lay   = QVBoxLayout(ecat_group)
        ecat_lay.setSpacing(3)
        ecat_lay.setContentsMargins(6, 6, 6, 6)
        ecat_group.setFixedWidth(220)

        self._preop_chk = QCheckBox("All → Pre-OP")
        self._preop_chk.toggled.connect(self._on_preop_toggled)
        self._preop_resp = _resp_label()

        main_store_btn = QPushButton("Main Store All")
        main_store_btn.clicked.connect(lambda: self._store_all("main"))
        self._main_store_resp = _resp_label()

        dut_store_btn = QPushButton("DUT Store All")
        dut_store_btn.clicked.connect(lambda: self._store_all("dut"))
        self._dut_store_resp = _resp_label()

        ecat_lay.addWidget(self._preop_chk)
        ecat_lay.addWidget(self._preop_resp)
        ecat_lay.addWidget(main_store_btn)
        ecat_lay.addWidget(self._main_store_resp)
        ecat_lay.addWidget(dut_store_btn)
        ecat_lay.addWidget(self._dut_store_resp)

        ecat_lay.addWidget(QLabel("Bus AL Status:"))
        self._bus_status_panel = QTextEdit()
        self._bus_status_panel.setReadOnly(True)
        _bus_font = QFont("Monospace"); _bus_font.setPointSize(8)
        self._bus_status_panel.setFont(_bus_font)
        self._bus_status_panel.setFixedHeight(200)
        self._bus_status_panel.setLineWrapMode(QTextEdit.NoWrap)
        self._bus_status_panel.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._bus_status_panel.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._bus_status_panel.setPlaceholderText("(waiting for bridge)")
        ecat_lay.addWidget(self._bus_status_panel)

        # ── Function Generator ────────────────────────────────────────────────
        fg_group = QGroupBox("Function Generator")
        fg_lay   = QVBoxLayout(fg_group)
        fg_lay.setSpacing(3)
        fg_lay.setContentsMargins(6, 6, 6, 6)

        # Drive selector + Enable checkbox
        fg_head = QHBoxLayout()
        self._fg_drive_combo = QComboBox()
        self._fg_drive_combo.addItem("Main", "main")
        self._fg_drive_combo.addItem("DUT",  "dut")
        fg_head.addWidget(self._fg_drive_combo)
        self._fg_enable_chk = QCheckBox("Enable FG")
        fg_head.addWidget(self._fg_enable_chk)
        fg_lay.addLayout(fg_head)

        # Control type
        fg_ct = QHBoxLayout()
        fg_ct.addWidget(QLabel("Control:"))
        self._fg_ctrl_combo = QComboBox()
        for _name, _val in [("None", 0), ("Velocity", 1), ("Position", 2),
                             ("Torque", 3), ("Current", 4)]:
            self._fg_ctrl_combo.addItem(_name, _val)
        fg_ct.addWidget(self._fg_ctrl_combo)
        fg_lay.addLayout(fg_ct)

        # Changing control type → auto-update the DS402 mode combo for the
        # selected drive so the JSON main_mode always agrees with apply_fg.
        _FG_CT_TO_MODE_COMBO = {
            1: next(i for i, (_, v) in enumerate(DS402_MODES) if v == 9),   # Velocity → CSV
            2: next(i for i, (_, v) in enumerate(DS402_MODES) if v == 8),   # Position → CSP
            3: next(i for i, (_, v) in enumerate(DS402_MODES) if v == 10),  # Torque   → CST
            4: next(i for i, (_, v) in enumerate(DS402_MODES) if v == -2),  # Current  → current mode
        }

        def _fg_on_ctrl_type(idx):
            if idx not in _FG_CT_TO_MODE_COMBO:
                return  # NONE (0) — leave mode combo unchanged
            mode_idx = _FG_CT_TO_MODE_COMBO[idx]
            drive = self._fg_drive_combo.currentData()
            combo = self._main_mode_combo if drive == "main" else self._dut_mode_combo
            combo.setCurrentIndex(mode_idx)

        def _fg_on_enable(state):
            if state:  # on check — sync mode combo to current control type
                _fg_on_ctrl_type(self._fg_ctrl_combo.currentIndex())

        self._fg_ctrl_combo.currentIndexChanged.connect(_fg_on_ctrl_type)
        self._fg_enable_chk.stateChanged.connect(_fg_on_enable)

        # Waveform
        fg_wf = QHBoxLayout()
        fg_wf.addWidget(QLabel("Waveform:"))
        self._fg_wf_combo = QComboBox()
        for _name in ["OFF", "DC", "Sine", "Square", "Triangle",
                      "Sawtooth", "Noise", "Chirp", "Exp Chirp"]:
            self._fg_wf_combo.addItem(_name)
        fg_wf.addWidget(self._fg_wf_combo)
        fg_lay.addLayout(fg_wf)

        # Parameter rows — shown/hidden based on waveform
        def _fg_param_row(label, lo, hi, decimals, step, suffix=""):
            w   = QWidget()
            row = QHBoxLayout(w)
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(4)
            lbl = QLabel(label)
            lbl.setFixedWidth(62)
            sb  = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setDecimals(decimals)
            sb.setSingleStep(step)
            sb.setKeyboardTracking(False)
            if suffix:
                sb.setSuffix(suffix)
            row.addWidget(lbl)
            row.addWidget(sb)
            return w, sb

        self._fg_amp_row,          self._fg_amp_spin          = _fg_param_row("Amplitude:", 0,      1e6,    3, 1.0)
        self._fg_freq_row,         self._fg_freq_spin         = _fg_param_row("Frequency:", 0,      1e4,    3, 0.1,  " Hz")
        self._fg_off_row,          self._fg_off_spin          = _fg_param_row("Offset:",    -1e6,   1e6,    3, 0.1)
        self._fg_phase_row,        self._fg_phase_spin        = _fg_param_row("Phase:",     -6.2832, 6.2832, 4, 0.01, " rad")
        self._fg_chirp_f_low_row,  self._fg_chirp_f_low_spin  = _fg_param_row("f start:",  0.001,  1e4,    3, 0.1,  " Hz")
        self._fg_chirp_f_high_row, self._fg_chirp_f_high_spin = _fg_param_row("f end:",    0.001,  1e4,    3, 0.1,  " Hz")
        self._fg_chirp_dur_row,    self._fg_chirp_dur_spin    = _fg_param_row("Duration:", 0.1,    1e4,    1, 1.0,  " s")

        for _w in (self._fg_amp_row, self._fg_freq_row, self._fg_off_row,
                   self._fg_phase_row, self._fg_chirp_f_low_row,
                   self._fg_chirp_f_high_row, self._fg_chirp_dur_row):
            fg_lay.addWidget(_w)

        _FG_PARAMS_VISIBLE = {
            0: [],
            1: ["offset"],
            2: ["amplitude", "frequency", "offset", "phase"],
            3: ["amplitude", "frequency", "offset", "phase"],
            4: ["amplitude", "frequency", "offset", "phase"],
            5: ["amplitude", "frequency", "offset", "phase"],
            6: ["amplitude", "offset"],
            7: ["amplitude", "chirp_f_low", "chirp_f_high", "chirp_dur", "offset", "phase"],
            8: ["amplitude", "chirp_f_low", "chirp_f_high", "chirp_dur", "offset", "phase"],
        }
        _fg_param_map = {
            "amplitude":   self._fg_amp_row,
            "frequency":   self._fg_freq_row,
            "offset":      self._fg_off_row,
            "phase":       self._fg_phase_row,
            "chirp_f_low": self._fg_chirp_f_low_row,
            "chirp_f_high":self._fg_chirp_f_high_row,
            "chirp_dur":   self._fg_chirp_dur_row,
        }

        def _fg_update_params(wf_idx):
            visible = _FG_PARAMS_VISIBLE.get(wf_idx, [])
            for _k, _w in _fg_param_map.items():
                _w.setVisible(_k in visible)

        self._fg_wf_combo.currentIndexChanged.connect(_fg_update_params)
        _fg_update_params(0)

        # Drive switch — save current drive state, load new drive state
        def _fg_save_current():
            drive = self._fg_drive_combo.currentData()
            st = self._fg_state[drive]
            st["enable"]       = self._fg_enable_chk.isChecked()
            st["waveform"]     = self._fg_wf_combo.currentIndex()
            st["control_type"] = self._fg_ctrl_combo.currentIndex()
            st["amplitude"]    = self._fg_amp_spin.value()
            st["frequency"]    = self._fg_freq_spin.value()
            st["offset"]       = self._fg_off_spin.value()
            st["phase"]        = self._fg_phase_spin.value()
            st["chirp_f_low"]  = self._fg_chirp_f_low_spin.value()
            st["chirp_f_high"] = self._fg_chirp_f_high_spin.value()
            st["chirp_dur"]    = self._fg_chirp_dur_spin.value()

        def _fg_load(drive):
            st = self._fg_state[drive]
            for _w in (self._fg_enable_chk, self._fg_wf_combo, self._fg_ctrl_combo):
                _w.blockSignals(True)
            self._fg_enable_chk.setChecked(st["enable"])
            self._fg_wf_combo.setCurrentIndex(st["waveform"])
            self._fg_ctrl_combo.setCurrentIndex(st["control_type"])
            self._fg_amp_spin.setValue(st["amplitude"])
            self._fg_freq_spin.setValue(st["frequency"])
            self._fg_off_spin.setValue(st["offset"])
            self._fg_phase_spin.setValue(st["phase"])
            self._fg_chirp_f_low_spin.setValue(st["chirp_f_low"])
            self._fg_chirp_f_high_spin.setValue(st["chirp_f_high"])
            self._fg_chirp_dur_spin.setValue(st["chirp_dur"])
            for _w in (self._fg_enable_chk, self._fg_wf_combo, self._fg_ctrl_combo):
                _w.blockSignals(False)
            _fg_update_params(st["waveform"])

        self._fg_save_current = _fg_save_current
        self._fg_load         = _fg_load

        self._fg_drive_combo.currentIndexChanged.connect(
            lambda _: (_fg_save_current(), _fg_load(self._fg_drive_combo.currentData())))

        ecat_lay.addWidget(fg_group)
        ecat_lay.addStretch(1)

        right_lay.addWidget(btn_w)
        right_lay.addWidget(self._script_panel, 1)
        right_lay.addWidget(sdo_group)
        right_lay.addWidget(ecat_group)
        right_outer.addWidget(right_inner)
        right_outer.addStretch(1)

        # Match the top splitter row so the left panel, slider "Clear" buttons,
        # and right-side panels share the same bottom edge.
        _content_h = right_inner.sizeHint().height()
        slots_w.setMinimumHeight(_content_h)
        self._field_list.setMinimumHeight(max(200, _content_h - 15))

        # ── Splitter ───────────────────────────────────────────────────────────
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(field_list_outer)
        splitter.addWidget(slots_outer)
        splitter.addWidget(right_w)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setFixedHeight(max(200, _content_h - 15 + 32))

        # ── Bottom status / bridge row ────────────────────────────────────────
        status_row = QWidget()
        status_lay = QHBoxLayout(status_row)
        status_lay.setContentsMargins(6, 2, 6, 2)
        status_lay.setSpacing(8)

        self._status_label = QLabel("bridge_ros2 starting…")
        self._status_label.setAlignment(Qt.AlignCenter)
        status_lay.addWidget(self._status_label, 1)

        self._bridge_status_lbl = QLabel("Bridge: ○ Stopped")
        self._bridge_status_lbl.setFont(QFont("Monospace", 8))
        status_lay.addWidget(self._bridge_status_lbl)

        if _MIDO_AVAILABLE:
            self._midi_status_lbl = QLabel("X-Touch: scanning…")
            self._midi_status_lbl.setFont(QFont("Monospace", 8))
            self._midi_status_lbl.setStyleSheet("color: gray; font-style: italic;")
            status_lay.addWidget(self._midi_status_lbl)
        else:
            self._midi_status_lbl = None

        self._bridge_start_btn   = QPushButton("Start")
        self._bridge_stop_btn    = QPushButton("Stop")
        self._bridge_restart_btn = QPushButton("Restart")
        status_lay.addWidget(self._bridge_start_btn)
        status_lay.addWidget(self._bridge_stop_btn)
        status_lay.addWidget(self._bridge_restart_btn)

        self._bridge_start_btn.clicked.connect(self._on_bridge_start)
        self._bridge_stop_btn.clicked.connect(self._on_bridge_stop)
        self._bridge_restart_btn.clicked.connect(self._on_bridge_restart)

        if self._bridge_args is None:
            for _b in (self._bridge_start_btn, self._bridge_stop_btn,
                       self._bridge_restart_btn):
                _b.setEnabled(False)
                _b.setToolTip("Bridge not managed by GUI")

        # ── Post-processing buttons (left-aligned with _script_panel) ─────────
        def _launch(cmd):
            subprocess.Popen(cmd, start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        def _launch_as_user(cmd):
            sudo_user = os.environ.get("SUDO_USER")
            if sudo_user:
                cmd = ["sudo", "-u", sudo_user, "-H",
                       "--preserve-env=DISPLAY,XAUTHORITY"] + cmd
            subprocess.Popen(cmd, start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        def _launch_as_user_ros2(script_path):
            """Launch a Python script that requires ROS2, sourcing the ROS2 env first."""
            bash_cmd = f'source /opt/ros/humble/setup.bash && python3 "{script_path}"'
            sudo_user = os.environ.get("SUDO_USER")
            if sudo_user:
                cmd = ["sudo", "-u", sudo_user, "-H",
                       "--preserve-env=DISPLAY,XAUTHORITY",
                       "bash", "-c", bash_cmd]
            else:
                cmd = ["bash", "-c", bash_cmd]
            subprocess.Popen(cmd, start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))

        btn_plot = QPushButton("Live Plot")
        btn_plot.clicked.connect(lambda: _launch(
            ["bash", os.path.join(repo, "src/interface_bridges/ros2/joint_impact_prevention_testbench_interface/run_plot.sh")]))

        btn_encoder = QPushButton("Encoder Linearization")
        btn_encoder.clicked.connect(lambda: _launch_as_user(
            ["python3", os.path.join(repo,
             "src/tools/post_processing/encoder_linearization/dyno_encoder_linearization_gui.py")]))

        btn_enc_comp = QPushButton("Enc Comp Write")
        btn_enc_comp.clicked.connect(lambda: _launch_as_user_ros2(
            os.path.join(repo,
             "src/tools/post_processing/encoder_linearization/dyno_encoder_comp_write_gui.py")))

        plot_btn_w = max(btn_plot.sizeHint().width(), btn_encoder.sizeHint().width())
        btn_plot.setFixedWidth(plot_btn_w)
        btn_encoder.setFixedWidth(plot_btn_w)

        btn_log = QPushButton("Log Viewer")
        btn_log.clicked.connect(lambda: _launch(
            ["python3", os.path.join(repo, "src/tools/dyno_log_viewer.py")]))

        btn_bode = QPushButton("Bode Plot")
        btn_bode.clicked.connect(lambda: _launch_as_user(
            ["python3", os.path.join(repo,
             "src/tools/post_processing/bode_plot/dyno_bode_gui.py")]))

        btn_kt = QPushButton("Kt Plot")
        btn_kt.clicked.connect(lambda: _launch_as_user(
            ["python3", os.path.join(repo,
             "src/tools/post_processing/kt_plot/dyno_kt_gui.py")]))

        btn_cogging = QPushButton("Cogging")
        btn_cogging.clicked.connect(lambda: _launch_as_user(
            ["python3", os.path.join(repo,
             "src/tools/post_processing/cogging_compensation_analysis/dyno_cogging_gui.py")]))

        # ── Google Drive sync ─────────────────────────────────────────────────
        _upload_proc = [None]   # mutable box: subprocess.Popen or None

        _sync_label = QLabel("Not synced")
        _sync_label.setStyleSheet("color: grey; font-size: 10px;")

        _poll_timer = QTimer()

        def _poll_upload():
            proc = _upload_proc[0]
            if proc is None or proc.poll() is not None:
                _poll_timer.stop()
                if proc is not None:
                    from datetime import datetime
                    ts = datetime.now().strftime("%H:%M")
                    if proc.returncode == 0:
                        _sync_label.setText(f"Synced {ts}")
                        _sync_label.setStyleSheet("color: grey; font-size: 10px;")
                    else:
                        _sync_label.setText(f"Sync failed {ts}")
                        _sync_label.setStyleSheet("color: red; font-size: 10px;")

        _poll_timer.timeout.connect(_poll_upload)

        def _sync_to_drive():
            if _upload_proc[0] is not None and _upload_proc[0].poll() is None:
                return   # already running
            src = os.path.join(repo, "actuator_test_log")
            if not os.path.isdir(src):
                _sync_label.setText("No actuator_test_log")
                _sync_label.setStyleSheet("color: red; font-size: 10px;")
                return
            sudo_user = (os.environ.get("SUDO_USER")
                         or os.environ.get("DYNO_ORIGINAL_USER"))
            # rclone copy: upload new/changed files only, never delete destination
            cmd = ["rclone", "copy", src, "foundation_gdrive_hw_actuator:actuator_test_log",
                   "--transfers=4", "--checkers=8"]
            if sudo_user:
                cmd = ["sudo", "-u", sudo_user, "-H"] + cmd
            try:
                _upload_proc[0] = subprocess.Popen(
                    cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    start_new_session=True)
                _sync_label.setText("Syncing…")
                _sync_label.setStyleSheet("color: orange; font-size: 10px;")
                _poll_timer.start(2000)
            except OSError:
                _sync_label.setText("rclone not found")
                _sync_label.setStyleSheet("color: red; font-size: 10px;")

        _auto_check = QCheckBox("Auto")
        _auto_check.setToolTip("Automatically sync to Google Drive on a timer")

        _auto_spin = QSpinBox()
        _auto_spin.setRange(1, 24)
        _auto_spin.setValue(2)
        _auto_spin.setSuffix(" h")
        _auto_spin.setFixedWidth(58)
        _auto_spin.setToolTip("Auto-sync interval in hours")

        _auto_timer = QTimer()
        _auto_timer.timeout.connect(_sync_to_drive)

        def _update_auto_timer(checked):
            if checked:
                _sync_to_drive()   # immediate sync on enable
                _auto_timer.start(_auto_spin.value() * 3_600_000)
            else:
                _auto_timer.stop()

        _auto_check.toggled.connect(_update_auto_timer)
        _auto_spin.valueChanged.connect(
            lambda v: _auto_timer.start(v * 3_600_000) if _auto_check.isChecked() else None)

        btn_sync = QPushButton("Sync to Drive")
        btn_sync.clicked.connect(_sync_to_drive)

        # Two-row post-processing area.  The top row aligns Live Plot with the
        # other tool buttons; Encoder Linearization sits directly underneath.
        # The left spacer matches btn_w (200 px) + right_lay spacing (6 px).
        pp_row     = QWidget()
        pp_row_lay = QVBoxLayout(pp_row)
        pp_row_lay.setContentsMargins(0, 14, 0, 2)
        pp_row_lay.setSpacing(4)

        pp_top     = QWidget()
        pp_top_lay = QHBoxLayout(pp_top)
        pp_top_lay.setContentsMargins(0, 0, 0, 0)
        pp_top_lay.setSpacing(6)
        pp_spacer_top = QWidget()
        pp_spacer_top.setFixedWidth(206)
        pp_top_lay.addWidget(pp_spacer_top)
        pp_top_lay.addWidget(btn_plot)
        for btn in (btn_log, btn_bode, btn_kt, btn_cogging, btn_sync):
            pp_top_lay.addWidget(btn)
        pp_top_lay.addWidget(_sync_label)
        pp_top_lay.addWidget(_auto_check)
        pp_top_lay.addWidget(_auto_spin)
        pp_top_lay.addStretch(1)

        pp_bottom     = QWidget()
        pp_bottom_lay = QHBoxLayout(pp_bottom)
        pp_bottom_lay.setContentsMargins(0, 0, 0, 0)
        pp_bottom_lay.setSpacing(6)
        pp_spacer_bottom = QWidget()
        pp_spacer_bottom.setFixedWidth(206)
        pp_bottom_lay.addWidget(pp_spacer_bottom)
        btn_inertia = QPushButton("Inertia Analysis")
        btn_inertia.clicked.connect(lambda: _launch_as_user(
            ["python3", os.path.join(repo,
             "src/tools/post_processing/inertia_estimation/dyno_inertia_gui.py")]))

        pp_bottom_lay.addWidget(btn_encoder)
        pp_bottom_lay.addWidget(btn_enc_comp)
        pp_bottom_lay.addWidget(btn_inertia)
        pp_bottom_lay.addStretch(1)

        pp_row_lay.addWidget(pp_top)
        pp_row_lay.addWidget(pp_bottom)
        right_outer.insertWidget(1, pp_row)  # between right_inner and stretch

        # ── Central widget ─────────────────────────────────────────────────────
        central = QWidget()
        vlay    = QVBoxLayout(central)
        vlay.setContentsMargins(0, 0, 0, 0)
        vlay.setSpacing(0)
        vlay.addWidget(splitter)
        vlay.addWidget(spin_area)
        vlay.addWidget(status_row)
        self.setCentralWidget(central)

        # Rough initial width; height corrected after first layout pass.
        self.resize(STARTUP_WINDOW_WIDTH, STARTUP_WINDOW_HEIGHT)

    # ── button callbacks ──────────────────────────────────────────────────────

    def _toggle_main(self):
        self._main_enabled = self._main_enable_btn.isChecked()
        self._main_enable_btn.setText(
            "Main Enabled" if self._main_enabled else "Main Enable")

    def _toggle_dut(self):
        self._dut_enabled = self._dut_enable_btn.isChecked()
        self._dut_enable_btn.setText(
            "DUT Enabled" if self._dut_enabled else "DUT Enable")

    def _toggle_output1(self):
        self._hold_output1 = self._output1_btn.isChecked()
        self._output1_btn.setText(
            "Output 1 ON" if self._hold_output1 else "Hold Output 1")

    def _filter_command_fields(self, text: str) -> None:
        q = text.strip().lower()
        for i in range(self._field_list.count()):
            item = self._field_list.item(i)
            label = item.text().lower()
            key   = (item.data(Qt.UserRole) or "").lower()
            item.setHidden(bool(q) and q not in label and q not in key)

    def _on_ch1_scale_changed(self, idx: int) -> None:
        self._ch1_scale = self._ch1_scale_combo.itemData(idx)

    def _on_ch2_scale_changed(self, idx: int) -> None:
        self._ch2_scale = self._ch2_scale_combo.itemData(idx)

    def _main_zero(self):
        for slot in self._slots:
            if slot.field in MAIN_ZERO_FIELDS:
                slot.zero()
        for slot in self._spin_slots:
            if slot.field in MAIN_ZERO_FIELDS:
                slot.zero()

    def _dut_zero(self):
        for slot in self._slots:
            if slot.field in DUT_ZERO_FIELDS:
                slot.zero()
        for slot in self._spin_slots:
            if slot.field in DUT_ZERO_FIELDS:
                slot.zero()

    # ── script handoff ────────────────────────────────────────────────────────

    def _on_script_active(self, active: bool) -> None:
        """Called by ScriptingPanel when a script starts (True) or finishes (False)."""
        self._script_running = active
        # Always reset mode combos to "No Mode (0)":
        #   • on start  — ensures _get_modes() returns (0,0) so the preamble doesn't
        #                 briefly re-apply whatever manual mode the GUI combo showed.
        #   • on finish — reflects the epilogue's mode=0 disable state.
        _none_idx = next(i for i, (_, v) in enumerate(DS402_MODES) if v == 0)
        self._main_mode_combo.setCurrentIndex(_none_idx)
        self._dut_mode_combo.setCurrentIndex(_none_idx)
        if not active:
            # Uncheck both enable buttons so the drive is not automatically
            # re-enabled when GUI push resumes after the script.
            self._main_enabled = False
            self._dut_enabled  = False
            self._main_enable_btn.setChecked(False)
            self._main_enable_btn.setText("Main Enable")
            self._dut_enable_btn.setChecked(False)
            self._dut_enable_btn.setText("DUT Enable")
            # Push the now-disabled state once so the drive receives it immediately.
            self._push_command()

    def _fault_reset_pressed(self) -> None:
        """Fault Reset button handler — blocked while a script is running."""
        if self._script_running:
            return   # must abort script before using fault reset
        self._cmd.pulse_fault_reset()

    # ── MIDI bridge handlers ──────────────────────────────────────────────────

    def _on_fader_moved(self, fader_idx: int, normalized: float):
        if 0 <= fader_idx < len(self._slots):
            self._slots[fader_idx].set_from_midi(normalized)

    def _on_midi_connection(self, connected: bool):
        if self._midi_status_lbl is None:
            return
        if connected:
            self._midi_status_lbl.setText("X-Touch: ● Connected")
            self._midi_status_lbl.setStyleSheet("color: green;")
            for i, slot in enumerate(self._slots):
                if slot.field is not None and self._midi_bridge:
                    self._midi_bridge.send_fader(i, slot.normalized_value)
        else:
            self._midi_status_lbl.setText("X-Touch: ○ Disconnected")
            self._midi_status_lbl.setStyleSheet("color: orange;")

    # ── publish ───────────────────────────────────────────────────────────────

    def _push_command(self):
        if self._script_running:
            return
        # Torque clamp slots are read-only displays of the algorithm's RT output.
        # Update at 20 Hz (every 10 ticks) to avoid visual flicker — the braking
        # algorithm may toggle these rapidly on each RT cycle.
        self._torque_display_tick = (self._torque_display_tick + 1) % 10
        if self._torque_display_tick == 0:
            for _slot in self._slots + self._spin_slots:
                if _slot.field in TORQUE_CLAMP_FIELDS:
                    _res = self._cmd.get_limits(_slot.field)
                    if _res is not None:
                        _slot.apply_value(_res[2])
            # Always show the SDO abs-max regardless of whether a slot is assigned.
            _abs_res = self._cmd.get_limits("main_torque_max")
            if _abs_res is not None:
                self._torque_abs_max_label.setText(f"{_abs_res[1]:.3f}")
        # Setpoint fields default to 0; gain fields are omitted unless a slider
        # is actively assigned to them (so the bridge keeps its SDO-seeded values).
        # TORQUE_CLAMP_FIELDS are display-only — never send them as commands or the
        # braking algorithm's output gets fed back into CommandState, zeroing the
        # torque limit between braking cycles and breaking the algorithm's reset path.
        numeric = {k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS}
        for slot in self._slots:
            if slot.field is not None and slot.field not in TORQUE_CLAMP_FIELDS:
                numeric[slot.field] = slot.value
        for slot in self._spin_slots:
            if slot.field is not None and slot.field not in TORQUE_CLAMP_FIELDS:
                numeric[slot.field] = slot.value
        numeric["ch1_torque_scale"] = self._ch1_scale
        numeric["ch2_torque_scale"] = self._ch2_scale

        # Sync current widget values into fg_state for the selected drive, then
        # include both drives' FG fields so the bridge always gets a full update.
        self._fg_save_current()
        for _drive in ("main", "dut"):
            _st    = self._fg_state[_drive]
            _pfx   = f"{_drive}_fg"
            numeric[f"{_pfx}_enable"]       = _st["enable"]
            numeric[f"{_pfx}_waveform"]     = _st["waveform"]
            numeric[f"{_pfx}_control_type"] = _st["control_type"]
            numeric[f"{_pfx}_amplitude"]    = _st["amplitude"]
            numeric[f"{_pfx}_frequency"]    = _st["frequency"]
            numeric[f"{_pfx}_offset"]       = _st["offset"]
            numeric[f"{_pfx}_phase"]        = _st["phase"]
            numeric[f"{_pfx}_chirp_f_low"]  = _st["chirp_f_low"]
            numeric[f"{_pfx}_chirp_f_high"] = _st["chirp_f_high"]
            numeric[f"{_pfx}_chirp_dur"]    = _st["chirp_dur"]

        # Mode comes directly from the DS402 mode combos, which are auto-synced
        # by _fg_on_ctrl_type when the FG control type changes.  The RT
        # callback's apply_fg is the per-cycle enforcer; NO_MODE on FG disable
        # is handled there as well.
        main_mode_val = DS402_MODES[self._main_mode_combo.currentIndex()][1]
        dut_mode_val  = DS402_MODES[self._dut_mode_combo.currentIndex()][1]

        self._cmd.set_command(
            numeric      = numeric,
            main_enable  = self._main_enabled,
            dut_enable   = self._dut_enabled,
            hold_output1 = self._hold_output1,
            main_mode    = main_mode_val,
            dut_mode     = dut_mode_val,
        )

    # ── SDO handlers ──────────────────────────────────────────────────────────

    def _poll_sdo_response(self):
        now = time.monotonic()

        # Timeout: fault history sequence waiting too long for next response
        if self._fault_hist_state is not None:
            if now - self._fault_hist_state["request_time"] > SDO_TIMEOUT_S:
                drive  = self._fault_hist_state["drive"]
                sub    = self._fault_hist_state["next_sub"]
                widget = self._main_fault_history if drive == "main" else self._dut_fault_history
                widget.setPlainText(f"Timeout on sub 0x{sub:02X}")
                self._fault_hist_state = None
                return

        # Timeout: regular SDO read/write or ecat op waiting too long
        if self._fault_hist_state is None and self._sdo_pending_time is not None:
            if now - self._sdo_pending_time > SDO_TIMEOUT_S:
                if self._ecat_pending is not None:
                    label = (self._main_store_resp if "main" in self._ecat_pending
                             else self._dut_store_resp if "dut" in self._ecat_pending
                             else self._preop_resp)
                    label.setText("Timeout — no response")
                    self._ecat_pending = None
                else:
                    self._sdo_feedback.setPlainText("Timeout — no response from bridge")
                self._sdo_pending_time = None

        bus_data = self._cmd.pop_bus_status()
        if bus_data is not None:
            self._update_bus_status(bus_data)

        data = self._cmd.pop_sdo_response()
        if data is None:
            return
        if self._fault_hist_state is not None:
            self._handle_fault_hist_response(data)
        elif self._ecat_pending is not None:
            self._sdo_pending_time = None
            self._handle_ecat_response(data)
        else:
            self._sdo_pending_time = None
            self._on_sdo_response(data)

    def _read_fault_history(self, drive: str):
        widget = self._main_fault_history if drive == "main" else self._dut_fault_history
        widget.setPlainText("…reading…")
        self._fault_hist_state = {
            "drive": drive, "next_sub": 0, "results": [],
            "request_time": time.monotonic(),
        }
        # sub 0x00 is UINT8 (error count), subs 1–4 are UINT32
        self._cmd.request_sdo(drive, "read", 0x1003, 0x00, 1)

    def _handle_fault_hist_response(self, data: dict):
        state  = self._fault_hist_state
        drive  = state["drive"]
        widget = self._main_fault_history if drive == "main" else self._dut_fault_history
        state["results"].append(data)
        # Abort on any failure — avoids queuing further SDO requests to a
        # slave that isn't responding (each would pause the RT PDO loop).
        if not data.get("success"):
            self._fault_hist_state = None
            self._render_fault_history(state["results"], widget)
            return
        next_sub = state["next_sub"] + 1
        state["next_sub"] = next_sub
        if next_sub <= 4:
            state["request_time"] = time.monotonic()
            self._cmd.request_sdo(drive, "read", 0x1003, next_sub, 4)
        else:
            self._fault_hist_state = None
            self._render_fault_history(state["results"], widget)

    def _render_fault_history(self, results: list, widget) -> None:
        if not results or not results[0].get("success"):
            widget.setPlainText("Read failed")
            return
        num_errors = results[0].get("value", 0)
        if num_errors == 0:
            widget.setPlainText("No faults stored")
            return
        lines = [f"{num_errors} fault(s) stored:"]
        for i, entry in enumerate(results[1:], 1):
            if i > num_errors:
                break
            if entry.get("success"):
                code = entry.get("value", 0)
                desc = _lookup_error(code)
                lines.append(f"[{i}] {desc}" if desc else f"[{i}] 0x{code:08X}")
            else:
                lines.append(f"[{i}] read failed")
        widget.setPlainText("\n".join(lines))

    def _sdo_lookup_register(self):
        """Update reg_info label and auto-set size when index/sub change."""
        try:
            idx = int(self._sdo_index.text() or "0", 16)
            sub = int(self._sdo_sub.text()   or "0", 16)
        except ValueError:
            self._sdo_reg_info.setText("")
            return
        entry = _REGISTER_MAP.get((idx, sub))
        if entry:
            name  = entry.get("Name", "")
            dtype = entry.get("Data Type", "")
            self._sdo_reg_info.setText(f"{name}\n[{dtype}]")
            size_map = {1: 0, 2: 1, 4: 2, 8: 2}
            combo_idx = size_map.get(_dtype_size(dtype), 2)
            self._sdo_size.setCurrentIndex(combo_idx)
        else:
            self._sdo_reg_info.setText("")

    def _sdo_read(self):
        drive    = self._sdo_drive.currentText()
        index    = int(self._sdo_index.text() or "0", 16)
        subindex = int(self._sdo_sub.text()   or "0", 16)
        size     = [1, 2, 4][self._sdo_size.currentIndex()]
        self._sdo_feedback.setPlainText("…waiting…")
        self._sdo_pending_time = time.monotonic()
        self._cmd.request_sdo(drive, "read", index, subindex, size)

    def _sdo_write(self):
        drive    = self._sdo_drive.currentText()
        index    = int(self._sdo_index.text() or "0", 16)
        subindex = int(self._sdo_sub.text()   or "0", 16)
        size     = [1, 2, 4][self._sdo_size.currentIndex()]
        entry = _REGISTER_MAP.get((index, subindex))
        dtype = entry.get("Data Type", "") if entry else ""
        text  = self._sdo_value.text().strip()
        try:
            if dtype:
                raw_uint, size = _encode_sdo_value(text, dtype)
            else:
                raw_uint = int(text or "0", 0)
        except Exception as e:
            self._sdo_feedback.setPlainText(f"Parse error: {e}")
            return
        self._sdo_feedback.setPlainText("…waiting…")
        self._sdo_pending_time = time.monotonic()
        self._cmd.request_sdo(drive, "write", index, subindex, size, raw_uint)

    def _on_sdo_response(self, data: dict):
        if data.get("success"):
            op      = data.get("op", "?").upper()
            idx_str = data.get("index", "?")
            sub_raw = data.get("subindex", 0)
            raw_val = data.get("value", 0)
            try:
                idx_int = int(idx_str.replace("0x", "").replace("0X", ""), 16)
                sub_int = int(sub_raw) if isinstance(sub_raw, int) else int(str(sub_raw), 0)
            except Exception:
                idx_int, sub_int = 0, 0
            entry = _REGISTER_MAP.get((idx_int, sub_int))
            dtype = entry.get("Data Type", "") if entry else ""
            if dtype:
                val_str = _decode_sdo_value(raw_val, dtype)
            else:
                val_str = data.get("value_hex", hex(raw_val))
            name_line = f"Name:     {entry['Name']}\n" if entry else ""
            type_line = f"Type:     {dtype}\n"         if dtype  else ""
            self._sdo_feedback.setPlainText(
                f"{op} OK\n"
                f"Index:    {idx_str}\n"
                f"Sub:      0x{sub_int:02X}\n"
                f"{name_line}{type_line}"
                f"Value:    {val_str}"
            )
        else:
            self._sdo_feedback.setPlainText(
                f"ERROR\n{data.get('error', 'unknown')}")

    def _handle_ecat_response(self, data: dict) -> None:
        op      = data.get("op", "")
        success = data.get("success", False)
        err     = data.get("error", "unknown error")
        pending = self._ecat_pending
        self._ecat_pending = None

        if op in ("pre_op_all", "pre_op_off"):
            self._preop_resp.setText("OK" if success else f"ERR: {err}")
            # Sync checkbox to actual state without re-triggering the handler
            self._preop_chk.blockSignals(True)
            self._preop_chk.setChecked(op == "pre_op_all" and success)
            self._preop_chk.blockSignals(False)
        elif op == "store_all":
            label = self._main_store_resp if pending == "main_store" else self._dut_store_resp
            label.setText("OK — stored" if success else f"ERR: {err}")
            # Uncheck Pre-OP checkbox — loop has been restarted
            self._preop_chk.blockSignals(True)
            self._preop_chk.setChecked(False)
            self._preop_chk.blockSignals(False)
            self._preop_resp.setText("")

    def _update_bus_status(self, slaves: list) -> None:
        lines = []
        for s in slaves:
            idx  = s.get("idx", "?")
            name = s.get("name", "?")[:14].ljust(14)
            al   = s.get("al", "?")
            lines.append(f"[{idx:2}] {name}  {al}")
        new_text = "\n".join(lines)
        if self._bus_status_panel.toPlainText() == new_text:
            return
        sb  = self._bus_status_panel.verticalScrollBar()
        pos = sb.value()
        self._bus_status_panel.setPlainText(new_text)
        sb.setValue(pos)

    def _on_preop_toggled(self, checked: bool) -> None:
        self._preop_resp.setText("…waiting…")
        self._ecat_pending = "pre_op"
        self._sdo_pending_time = time.monotonic()
        self._cmd.request_pre_op(checked)

    def _store_all(self, drive: str) -> None:
        label = self._main_store_resp if drive == "main" else self._dut_store_resp
        label.setText("…waiting…")
        self._preop_chk.blockSignals(True)
        self._preop_chk.setChecked(True)
        self._preop_chk.blockSignals(False)
        self._preop_resp.setText("(store in progress)")
        self._ecat_pending = f"{drive}_store"
        self._sdo_pending_time = time.monotonic()
        self._cmd.request_store_all(drive)

    def _on_limits_updated(self, drive: str) -> None:
        """Called from the ROS spin thread — schedule GUI update on the Qt thread."""
        QTimer.singleShot(0, lambda: self._refresh_slot_limits(drive))
        QTimer.singleShot(0, lambda: self._refresh_error_display(drive))

    def _refresh_all_errors(self) -> None:
        for drive in ("main", "dut"):
            self._refresh_error_display(drive)

    def _refresh_error_display(self, drive: str) -> None:
        code = self._cmd.get_error_code(drive)
        text = _lookup_error(code)
        widget = self._main_last_error if drive == "main" else self._dut_last_error
        if widget.toPlainText() != text:
            widget.setPlainText(text)
        al = self._cmd.get_al_state(drive)
        lbl = self._main_al_label if drive == "main" else self._dut_al_label
        lbl.setText(al)

    def _refresh_slot_limits(self, drive: str) -> None:
        prefix = "main_" if drive == "main" else "dut_"
        for slot in self._slots + self._spin_slots:
            if slot.field and slot.field.startswith(prefix):
                result = self._cmd.get_limits(slot.field)
                if result is not None:
                    lo, hi, current = result
                    slot.apply_limits(lo, hi)

    def set_status(self, text: str):
        self._status_label.setText(text)

    def closeEvent(self, event):
        """Abort any running script before closing so the cleanup zero command wins."""
        if self._script_running:
            self._script_panel._abort_script()
            # Give the script thread a moment to see stop_event and stop sending
            # enable=True before main() sends its zero command.
            time.sleep(0.15)
        if self._midi_bridge:
            self._midi_bridge.close()
        event.accept()

    # ── Bridge management ─────────────────────────────────────────────────────

    def _stop_bridge(self):
        """Send SIGINT to bridge, wait up to 5 s, then kill. No-op if not running."""
        proc = self._bridge_proc
        if proc is None or proc.poll() is not None:
            return
        print(f"[dyno_gui] Sending SIGINT to bridge_ros2 (PID {proc.pid})…")
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        print("[dyno_gui] bridge_ros2 stopped.")
        self._bridge_proc = None

    def _on_bridge_start(self):
        if self._bridge_proc and self._bridge_proc.poll() is None:
            return
        a = self._bridge_args
        self._bridge_proc = launch_bridge(
            a.bridge, a.topology, a.fault_reset_s, a.pub_hz, a.debug)
        self._update_bridge_status()

    def _on_bridge_stop(self):
        self._stop_bridge()
        self._update_bridge_status()

    def _on_bridge_restart(self):
        self._stop_bridge()
        time.sleep(0.5)
        self._on_bridge_start()

    def _update_bridge_status(self):
        if self._bridge_args is None:
            return
        proc = self._bridge_proc
        if proc is None or proc.poll() is not None:
            self._bridge_status_lbl.setText("Bridge: ○ Stopped")
            self._bridge_start_btn.setEnabled(True)
            self._bridge_stop_btn.setEnabled(False)
            self._bridge_restart_btn.setEnabled(False)
        else:
            self._bridge_status_lbl.setText(f"Bridge: ● PID {proc.pid}")
            self._bridge_start_btn.setEnabled(False)
            self._bridge_stop_btn.setEnabled(True)
            self._bridge_restart_btn.setEnabled(True)

    def _poll_bridge(self):
        if self._bridge_args is None:
            return
        proc = self._bridge_proc
        if proc is not None and proc.poll() is not None:
            self._bridge_proc = None
            self._bridge_status_lbl.setText(
                f"Bridge: ✕ Crashed (exit {proc.returncode})")
            self.set_status(f"bridge_ros2 exited unexpectedly (code {proc.returncode})")
            self._bridge_start_btn.setEnabled(True)
            self._bridge_stop_btn.setEnabled(False)
            self._bridge_restart_btn.setEnabled(False)


# ─────────────────────────────────────────────────────────────────────────────
# Bridge subprocess management
# ─────────────────────────────────────────────────────────────────────────────

def launch_bridge(bridge_path: str, topology: str, fault_reset_s: float,
                  pub_hz: float, debug: bool = False):
    """Launch bridge_ros2 as a subprocess with sudo -E."""
    cmd = [
        "sudo", "-E",
        bridge_path,
        "--ros-args",
        "-p", f"topology:={topology}",
        "-p", f"fault_reset_s:={fault_reset_s}",
        "-p", f"pub_hz:={pub_hz}",
    ]
    if debug:
        cmd += ["-p", "debug:=1"]
    print(f"[dyno_gui] Launching: {' '.join(cmd)}")
    # sudo -E overwrites SUDO_USER with the invoking user (root here, since the
    # GUI itself runs as root via sudo).  Preserve the original user in a custom
    # variable that sudo does not touch, so chown_to_sudo_user() in the bridge
    # can find the real unprivileged user.
    env = os.environ.copy()
    if "SUDO_USER" in env:
        env.setdefault("DYNO_ORIGINAL_USER", env["SUDO_USER"])
    proc = subprocess.Popen(cmd, env=env)
    return proc


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Dyno Qt GUI + ROS2 command publisher")
    p.add_argument("--bridge",        default=DEFAULT_BRIDGE,
                   help=f"Path to bridge_ros2 binary (default: {DEFAULT_BRIDGE})")
    p.add_argument("--topology",      default=DEFAULT_TOPOLOGY,
                   help=f"Topology JSON (default: {DEFAULT_TOPOLOGY})")
    p.add_argument("--pub-hz",        type=float, default=DEFAULT_PUB_HZ,
                   help=f"Command publish rate Hz (default: {DEFAULT_PUB_HZ})")
    p.add_argument("--fault-reset-s", type=float, default=DEFAULT_FAULT_S,
                   help=f"fault_reset_s for bridge (default: {DEFAULT_FAULT_S})")
    p.add_argument("--no-bridge",     action="store_true",
                   help="Don't launch bridge_ros2 (assume it's already running)")
    p.add_argument("--debug",         action="store_true",
                   help="Pass debug:=1 to bridge_ros2")
    return p.parse_args()


def main():
    args = parse_args()

    # ── Launch bridge subprocess ──────────────────────────────────────────────
    bridge_proc = None
    if not args.no_bridge:
        if not os.path.isfile(args.bridge):
            print(f"ERROR: bridge binary not found at '{args.bridge}'\n"
                  f"       Build it first:  bash src/interface_bridges/ros2/build.sh",
                  file=sys.stderr)
            sys.exit(1)
        bridge_proc = launch_bridge(
            args.bridge, args.topology, args.fault_reset_s, args.pub_hz, args.debug)

    # ── ROS2 init ─────────────────────────────────────────────────────────────
    rclpy.init()
    commander = DynoCommander(pub_hz=args.pub_hz)

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(commander,), daemon=True)
    ros_thread.start()

    # ── Qt GUI ────────────────────────────────────────────────────────────────
    app    = QApplication(sys.argv)
    window = DynoWindow(commander, bridge_proc=bridge_proc, bridge_args=args)
    window.resize(STARTUP_WINDOW_WIDTH, STARTUP_WINDOW_HEIGHT)

    if bridge_proc is not None:
        window.set_status(f"bridge_ros2 PID {bridge_proc.pid}")
    else:
        window.set_status("--no-bridge: connecting to existing bridge")

    window.show()

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    exit_code = app.exec_()

    # ── Cleanup ───────────────────────────────────────────────────────────────
    print("[dyno_gui] Window closed — shutting down.")
    commander.set_command(
        numeric={k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS},
        main_enable=False, dut_enable=False, hold_output1=False)
    time.sleep(0.1)

    rclpy.shutdown()

    window._stop_bridge()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
