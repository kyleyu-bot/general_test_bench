#!/usr/bin/env python3
"""Encoder compensation LUT writer — loads a linearization CSV and uploads it
to a drive via SDO writes over ROS2, then triggers store-all.
Also reads back the LUT from the drive for verification."""

from __future__ import annotations

import csv
import json
import math
import os
import struct  # used by _enc_counts_from_raw for readback reinterpretation
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

os.environ.setdefault("MPLCONFIGDIR", "/tmp/dyno_matplotlib")
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

# ── SDO register addresses ────────────────────────────────────────────────────
_INPUT_EN   = 0x2300   # write 1 to enable input-side compensation
_INPUT_IDX  = 0x2301   # LUT entry index   (UINT32, written before each value)
_INPUT_VAL  = 0x2302   # LUT entry value

_OUTPUT_EN  = 0x2303   # write 1 to enable output-side compensation
_OUTPUT_IDX = 0x2304   # LUT entry index   (UINT32, written before each value)
_OUTPUT_VAL = 0x2305   # LUT entry value

_SUBINDEX    = 0x00
_SIZE_BYTES  = 4        # uint32 is 4 bytes

_LUT_SIZE    = 2048
_SDO_TIMEOUT = 0.5      # seconds to wait for any single SDO ack / response

# ── Repo layout helpers ───────────────────────────────────────────────────────
_REPO_ROOT = Path(__file__).resolve().parents[4]   # …/dyno_testbench_cpp


def _search_roots() -> list[Path]:
    return [
        _REPO_ROOT / "actuator_test_log",
        _REPO_ROOT / "test_data_log",
    ]


def _find_lut_folders() -> list[Path]:
    """Return all folders that contain an encoder_linearization_lut_*.csv, newest first."""
    seen: set[Path] = set()
    hits: list[Path] = []
    for root in _search_roots():
        if not root.exists():
            continue
        for csv_path in root.rglob("encoder_linearization_lut_*.csv"):
            parent = csv_path.parent
            if parent not in seen:
                seen.add(parent)
                hits.append(parent)
    hits.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return hits


def _find_lut_csv(folder: Path, drive: str) -> Path | None:
    """Return the best-matching LUT CSV in folder for the given drive."""
    specific = sorted(folder.glob(f"encoder_linearization_lut_{drive}_*.csv"))
    if specific:
        return specific[0]
    fallback = sorted(folder.glob("encoder_linearization_lut_*.csv"))
    return fallback[0] if fallback else None


def _load_lut_csv(path: Path) -> list[tuple]:
    """Load LUT CSV. Returns list of (index, phase_rad, input_lut_rad, output_lut_rad)."""
    rows: list[tuple] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((
                int(row["index"]),
                float(row["phase_rad"]),
                float(row["input_lut_rad"]),
                float(row["output_lut_rad"]),
            ))
    return rows


_RAD_TO_ENC = (1 << 20) / (2.0 * math.pi)   # 2^20 counts per revolution


def _rad_to_enc_counts(val_rad: float) -> int:
    """Convert radians to signed 20-bit encoder counts."""
    return int(round(val_rad * _RAD_TO_ENC))


def _enc_counts_from_raw(raw_uint32: int) -> int:
    """Reinterpret uint32 from the bridge as signed int32 encoder counts."""
    return struct.unpack("<i", struct.pack("<I", raw_uint32 & 0xFFFFFFFF))[0]


# ── ROS2 SDO writer / reader ──────────────────────────────────────────────────
try:
    import rclpy
    from std_msgs.msg import String as StringMsg
    _ROS_OK = True
except ImportError:
    _ROS_OK = False


class _SdoWriter:
    """ROS2 node wrapper: synchronous SDO writes (waits for ack) and blocking reads.

    The bridge has a single pending-request slot — publishing a second request
    before the first is consumed overwrites it.  write_sync() and read() both
    wait for the bridge's response before returning, so requests are serialised
    and the slot is never clobbered.
    """

    def __init__(self) -> None:
        if not _ROS_OK:
            raise RuntimeError(
                "rclpy not available — source /opt/ros/humble/setup.bash"
            )
        try:
            rclpy.init()
        except Exception:
            pass  # already initialized
        self._node = rclpy.create_node("encoder_comp_writer")
        self._pub  = self._node.create_publisher(StringMsg, "/dyno/sdo_request", 10)
        self._node.create_subscription(
            StringMsg, "/dyno/sdo_response", self._on_response, 10)
        self._lock    = threading.Lock()
        self._pending: dict[tuple, tuple] = {}  # (idx, sub) → (event, result_box)
        threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True).start()
        time.sleep(1.0)   # allow ROS2 discovery to complete

    def _on_response(self, msg: StringMsg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        try:
            idx = int(data.get("index", "0"), 16)
            sub = int(str(data.get("subindex", 0)), 0)
        except Exception:
            return
        with self._lock:
            entry = self._pending.pop((idx, sub), None)
        if entry is not None:
            event, result_box = entry
            if data.get("success"):
                result_box[0] = data.get("value")
            event.set()

    def _request(self, payload: dict, index: int, subindex: int,
                 timeout: float) -> int | None:
        """Publish payload and block until the bridge responds for (index, subindex)."""
        event      = threading.Event()
        result_box = [None]
        key        = (index, subindex)
        with self._lock:
            self._pending[key] = (event, result_box)
        self._pub.publish(StringMsg(data=json.dumps(payload)))
        event.wait(timeout)
        with self._lock:
            self._pending.pop(key, None)
        return result_box[0]

    def write_sync(self, drive: str, index: int, subindex: int,
                   size: int, value: int, timeout: float = _SDO_TIMEOUT) -> bool:
        """SDO write — blocks until the bridge sends its write acknowledgment."""
        payload = {
            "drive":    drive,
            "op":       "write",
            "index":    f"{index:04X}",
            "subindex": f"{subindex:02X}",
            "size":     size,
            "value":    value,
        }
        return self._request(payload, index, subindex, timeout) is not None

    def read(self, drive: str, index: int, subindex: int,
             size: int, timeout: float = _SDO_TIMEOUT) -> int | None:
        """SDO read — blocks until the bridge responds with the value."""
        payload = {
            "drive":    drive,
            "op":       "read",
            "index":    f"{index:04X}",
            "subindex": f"{subindex:02X}",
            "size":     size,
        }
        return self._request(payload, index, subindex, timeout)

    def store_all(self, drive: str) -> None:
        self._pub.publish(
            StringMsg(data=json.dumps({"op": "store_all", "drive": drive}))
        )


# Module-level singleton — rclpy.init() must only be called once per process.
_sdo_singleton: _SdoWriter | None = None
_sdo_singleton_lock = threading.Lock()


def _get_sdo() -> _SdoWriter:
    global _sdo_singleton
    with _sdo_singleton_lock:
        if _sdo_singleton is None:
            _sdo_singleton = _SdoWriter()
        return _sdo_singleton


# ── Main application ──────────────────────────────────────────────────────────
class EncoderCompWriterApp(tk.Tk):

    def __init__(self) -> None:
        super().__init__()
        self.title("Encoder Compensation Writer")
        self.geometry("740x720")
        self.resizable(True, True)
        self._build_ui()
        self._refresh_folders()

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        # Data source
        src = ttk.LabelFrame(self, text="Data Source", padding=10)
        src.pack(fill="x", padx=10, pady=(10, 0))
        src.columnconfigure(1, weight=1)

        ttk.Label(src, text="Folder:").grid(row=0, column=0, sticky="w")
        self._folder_var = tk.StringVar()
        self._folder_cb  = ttk.Combobox(src, textvariable=self._folder_var,
                                         state="readonly", width=62)
        self._folder_cb.grid(row=0, column=1, sticky="ew", padx=6)
        self._folder_cb.bind("<<ComboboxSelected>>", lambda _: self._on_folder_change())
        ttk.Button(src, text="Browse",  command=self._browse).grid(row=0, column=2)
        ttk.Button(src, text="Refresh", command=self._refresh_folders).grid(
            row=0, column=3, padx=(4, 0))

        ttk.Label(src, text="CSV:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self._csv_var = tk.StringVar(value="—")
        ttk.Label(src, textvariable=self._csv_var, foreground="grey",
                  wraplength=540, anchor="w").grid(
            row=1, column=1, columnspan=3, sticky="w", pady=(6, 0))

        # Target
        tgt = ttk.LabelFrame(self, text="Target", padding=10)
        tgt.pack(fill="x", padx=10, pady=(10, 0))

        ttk.Label(tgt, text="Drive:").grid(row=0, column=0, sticky="w")
        self._drive_var = tk.StringVar(value="main")
        ttk.Combobox(tgt, textvariable=self._drive_var, values=["main", "dut"],
                     state="readonly", width=10).grid(
            row=0, column=1, sticky="w", padx=6)
        self._drive_var.trace_add("write", lambda *_: self._on_folder_change())

        ttk.Label(tgt, text="Side:").grid(row=0, column=2, sticky="w", padx=(24, 0))
        self._side_var = tk.StringVar(value="input")
        ttk.Combobox(tgt, textvariable=self._side_var, values=["input", "output"],
                     state="readonly", width=10).grid(
            row=0, column=3, sticky="w", padx=6)

        # Progress
        prg = ttk.LabelFrame(self, text="Progress", padding=10)
        prg.pack(fill="x", padx=10, pady=(10, 0))
        prg.columnconfigure(0, weight=1)

        self._progress_var = tk.DoubleVar(value=0.0)
        ttk.Progressbar(prg, variable=self._progress_var, maximum=100.0).grid(
            row=0, column=0, sticky="ew")
        self._progress_lbl = ttk.Label(prg, text="")
        self._progress_lbl.grid(row=1, column=0, sticky="w", pady=(4, 0))

        # Bottom bar
        bot = ttk.Frame(self)
        bot.pack(fill="x", padx=10, pady=(8, 0))
        self._write_btn = ttk.Button(bot, text="Write", command=self._start_write)
        self._write_btn.pack(side="left")
        self._readback_btn = ttk.Button(bot, text="Read Back",
                                        command=self._start_readback)
        self._readback_btn.pack(side="left", padx=(8, 0))
        self._status_var = tk.StringVar(value="Ready.")
        ttk.Label(bot, textvariable=self._status_var, foreground="grey").pack(
            side="left", padx=(12, 0))

        # Readback plot
        plot_frame = ttk.LabelFrame(self, text="Readback Plot", padding=6)
        plot_frame.pack(fill="both", expand=True, padx=10, pady=(8, 10))

        self._fig = Figure(figsize=(7, 3.2), dpi=96)
        self._ax  = self._fig.add_subplot(111)
        self._ax.set_xlabel("LUT index")
        self._ax.set_ylabel("Compensation (counts)")
        self._ax.set_title("Press 'Read Back' to read the LUT from the drive")
        self._ax.grid(True, alpha=0.3)
        self._fig.tight_layout()

        self._canvas = FigureCanvasTkAgg(self._fig, master=plot_frame)
        self._canvas.draw()
        self._canvas.get_tk_widget().pack(fill="both", expand=True)

    # ── Folder helpers ────────────────────────────────────────────────────────
    def _refresh_folders(self) -> None:
        folders  = _find_lut_folders()
        display  = [str(p) for p in folders]
        self._folder_cb["values"] = display
        if display and not self._folder_var.get():
            self._folder_var.set(display[0])
        self._on_folder_change()

    def _browse(self) -> None:
        path = filedialog.askdirectory(title="Select folder with encoder LUT CSV")
        if not path:
            return
        vals = list(self._folder_cb["values"])
        if path not in vals:
            vals.insert(0, path)
            self._folder_cb["values"] = vals
        self._folder_var.set(path)
        self._on_folder_change()

    def _on_folder_change(self) -> None:
        folder = self._folder_var.get()
        if not folder:
            self._csv_var.set("—")
            return
        drive    = self._drive_var.get()
        csv_path = _find_lut_csv(Path(folder), drive)
        self._csv_var.set(str(csv_path) if csv_path else
                          "No encoder_linearization_lut_*.csv found in this folder")

    def _set_buttons(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        self._write_btn.configure(state=state)
        self._readback_btn.configure(state=state)

    # ── Write sequence ────────────────────────────────────────────────────────
    def _start_write(self) -> None:
        folder = self._folder_var.get()
        if not folder:
            messagebox.showerror("No folder", "Select a folder first.")
            return

        drive    = self._drive_var.get()
        side     = self._side_var.get()
        csv_path = _find_lut_csv(Path(folder), drive)
        if csv_path is None:
            messagebox.showerror("No CSV",
                                 "No encoder_linearization_lut_*.csv found in the selected folder.")
            return

        try:
            rows = _load_lut_csv(csv_path)
        except Exception as exc:
            messagebox.showerror("CSV load failed", str(exc))
            return

        if not rows:
            messagebox.showerror("Empty CSV", "The LUT CSV contains no data rows.")
            return

        self._set_buttons(False)
        self._progress_var.set(0.0)
        self._progress_lbl.configure(text="")
        self._status_var.set("Connecting…")

        threading.Thread(
            target=self._write_worker,
            args=(rows, drive, side),
            daemon=True,
        ).start()

    def _write_worker(self, rows: list[tuple], drive: str, side: str) -> None:
        try:
            sdo = _get_sdo()
        except Exception as exc:
            self.after(0, lambda e=str(exc): messagebox.showerror("ROS2 unavailable", e))
            self.after(0, lambda: self._set_buttons(True))
            self.after(0, lambda: self._status_var.set("Ready."))
            return

        if side == "input":
            idx_obj = _INPUT_IDX
            val_obj = _INPUT_VAL
            en_obj  = _INPUT_EN
            val_col = 2
        else:
            idx_obj = _OUTPUT_IDX
            val_obj = _OUTPUT_VAL
            en_obj  = _OUTPUT_EN
            val_col = 3

        n            = len(rows)
        entries_done = 0

        self.after(0, lambda: self._status_var.set("Writing…"))

        def _update(msg: str = "") -> None:
            pct = 100.0 * entries_done / (n + 1)
            self.after(0, lambda p=pct: self._progress_var.set(p))
            self.after(0, lambda d=entries_done, t=n:
                       self._progress_lbl.configure(text=f"{d} / {t} entries written"))
            if msg:
                self.after(0, lambda m=msg: self._status_var.set(m))

        for row in rows:
            lut_index = row[0]
            lut_value = row[val_col]

            # Synchronous writes: wait for each ack before proceeding.
            # The bridge has a single pending-request slot — back-to-back
            # publishes would overwrite each other before being processed.
            sdo.write_sync(drive, idx_obj, _SUBINDEX, _SIZE_BYTES, lut_index)
            sdo.write_sync(drive, val_obj, _SUBINDEX, _SIZE_BYTES,
                           _rad_to_enc_counts(lut_value))
            entries_done += 1
            _update()

        _update(f"Enabling {side}-side compensation…")
        sdo.write_sync(drive, en_obj, _SUBINDEX, _SIZE_BYTES, 1)
        entries_done += 1
        _update()

        time.sleep(1.0)
        _update("Storing to non-volatile memory…")
        sdo.store_all(drive)
        time.sleep(2.0)

        self.after(0, lambda: (
            self._set_buttons(True),
            self._progress_var.set(100.0),
            self._status_var.set("Done — LUT written and stored."),
        ))

    # ── Readback sequence ─────────────────────────────────────────────────────
    def _start_readback(self) -> None:
        drive = self._drive_var.get()
        side  = self._side_var.get()

        self._set_buttons(False)
        self._progress_var.set(0.0)
        self._progress_lbl.configure(text="")
        self._status_var.set("Connecting…")

        threading.Thread(
            target=self._readback_worker,
            args=(drive, side),
            daemon=True,
        ).start()

    def _readback_worker(self, drive: str, side: str) -> None:
        try:
            sdo = _get_sdo()
        except Exception as exc:
            self.after(0, lambda e=str(exc): messagebox.showerror("ROS2 unavailable", e))
            self.after(0, lambda: self._set_buttons(True))
            self.after(0, lambda: self._status_var.set("Ready."))
            return

        if side == "input":
            idx_obj = _INPUT_IDX
            val_obj = _INPUT_VAL
        else:
            idx_obj = _OUTPUT_IDX
            val_obj = _OUTPUT_VAL

        self.after(0, lambda: self._status_var.set("Reading back…"))

        values_counts: list[int | float] = []
        timeouts = 0

        for i in range(_LUT_SIZE):
            # Synchronous: set the index first, then read the value.
            sdo.write_sync(drive, idx_obj, _SUBINDEX, _SIZE_BYTES, i)
            raw = sdo.read(drive, val_obj, _SUBINDEX, _SIZE_BYTES)
            if raw is None:
                values_counts.append(float("nan"))
                timeouts += 1
            else:
                values_counts.append(_enc_counts_from_raw(raw))

            pct = 100.0 * (i + 1) / _LUT_SIZE
            self.after(0, lambda p=pct, d=i + 1: (
                self._progress_var.set(p),
                self._progress_lbl.configure(
                    text=f"{d} / {_LUT_SIZE} entries read"),
            ))

        timeout_note = f"  ({timeouts} timeouts)" if timeouts else ""
        self.after(0, lambda v=values_counts[:], s=side, tn=timeout_note:
                   self._finish_readback(v, s, tn))

    def _finish_readback(self, values_counts: list, side: str,
                         timeout_note: str) -> None:
        self._plot_readback(values_counts, side)
        self._set_buttons(True)
        self._progress_var.set(100.0)
        self._status_var.set(f"Read back complete{timeout_note}.")

    def _plot_readback(self, values_counts: list, side: str) -> None:
        ax = self._ax
        ax.clear()

        indices = np.arange(len(values_counts))
        counts  = np.array(values_counts, dtype=float)

        ax.plot(indices, counts, linewidth=0.9, color="steelblue")
        ax.set_xlabel("LUT index")
        ax.set_ylabel("Compensation (counts)")
        ax.set_title(f"{side.capitalize()}-side encoder compensation LUT (readback from drive)")
        ax.grid(True, alpha=0.3)
        self._fig.tight_layout()
        self._canvas.draw()


if __name__ == "__main__":
    EncoderCompWriterApp().mainloop()
