#!/usr/bin/env python3
"""Kt (torque constant) plot GUI — loads a dyno_pdo.csv from a torque-ramp test,
auto-detects the ramp segment, and fits Iq vs torque to extract Kt."""

from __future__ import annotations

import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

os.environ.setdefault("MPLCONFIGDIR", "/tmp/dyno_matplotlib")

# Ensure the analysis module next to this file is importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from dyno_kt_analysis import (
    _DRIVE_COLS,
    _find_csv, _latest_log, _load_csv,
    _validate_kt_data, _is_2way_test,
    _detect_ramp_segment, _detect_2way_segments,
    _detect_torque_inversion,
    _linear_fit, make_kt_figure, save_kt_subplots,
)


class DynoKtApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dyno Kt Plot")
        self.geometry("1200x800")
        self._fig       = None
        self._canvas    = None
        self._toolbar   = None
        self._csv_path  = None
        self._axes_info: list[tuple] = []
        self._build_controls()
        self._plot_frame = ttk.LabelFrame(self, text="Preview", padding=4)
        self._plot_frame.pack(fill="both", expand=True, padx=10, pady=10)

    def _build_controls(self):
        ctrl = ttk.LabelFrame(self, text="Inputs", padding=10)
        ctrl.pack(fill="x", padx=10, pady=(10, 0))
        ctrl.columnconfigure(1, weight=1)

        self.path_var = tk.StringVar(value="test_data_log")
        ttk.Label(ctrl, text="Run folder:").grid(row=0, column=0, sticky="w")
        ttk.Entry(ctrl, textvariable=self.path_var, width=60).grid(
            row=0, column=1, sticky="ew", padx=6)
        ttk.Button(ctrl, text="Browse", command=self._browse).grid(row=0, column=2)
        ttk.Button(ctrl, text="Latest",  command=self._latest).grid(
            row=0, column=3, padx=(6, 0))

        self.drive_var  = tk.StringVar(value="main")
        self.sensor_var = tk.StringVar(value="ch1")
        self.invert_var = tk.BooleanVar(value=False)
        ttk.Label(ctrl, text="Drive:").grid(row=1, column=0, sticky="w", pady=6)
        ttk.Combobox(ctrl, textvariable=self.drive_var,
                     values=["main", "dut"], state="readonly", width=8).grid(
            row=1, column=1, sticky="w", padx=6)
        ttk.Label(ctrl, text="Torque sensor:").grid(row=1, column=2, sticky="w")
        ttk.Combobox(ctrl, textvariable=self.sensor_var,
                     values=["ch1", "ch2"], state="readonly", width=8).grid(
            row=1, column=3, sticky="w", padx=6)
        ttk.Checkbutton(ctrl, text="Invert torque (×−1)",
                        variable=self.invert_var).grid(
            row=1, column=4, sticky="w", padx=(12, 0))

        btn_row = ttk.Frame(ctrl)
        btn_row.grid(row=2, column=0, columnspan=4, sticky="e", pady=(4, 0))
        ttk.Button(btn_row, text="Generate", command=self._generate).pack(side="left")
        self._save_btn = ttk.Button(btn_row, text="Save", command=self._save,
                                    state="disabled")
        self._save_btn.pack(side="left", padx=(6, 0))

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(ctrl, textvariable=self.status_var, foreground="grey").grid(
            row=3, column=0, columnspan=4, sticky="w")

    def _browse(self):
        path = filedialog.askdirectory(title="Select run folder (contains dyno_pdo.csv)")
        if path:
            self.path_var.set(path)

    def _latest(self):
        try:
            folder = _latest_log(self.path_var.get().strip() or "test_data_log")
            self.path_var.set(str(folder))
            self.status_var.set(f"Loaded: {folder}")
        except Exception as exc:
            messagebox.showerror("Latest log failed", str(exc))

    def _generate(self):
        folder     = self.path_var.get().strip() or "test_data_log"
        drive      = self.drive_var.get()
        sensor_col = f"torque_{self.sensor_var.get()}_nm"

        try:
            csv_path = _find_csv(folder)
        except FileNotFoundError as exc:
            messagebox.showerror("Wrong folder", str(exc))
            return

        try:
            data, header = _load_csv(csv_path)
            _validate_kt_data(data, header, drive)
        except Exception as exc:
            messagebox.showerror("Invalid data", str(exc))
            return

        if sensor_col not in header:
            messagebox.showerror("Column missing", f"Column '{sensor_col}' not in CSV.")
            return

        two_way = _is_2way_test(csv_path)

        try:
            if two_way:
                seg1, seg2, seg3, gear_ratio = _detect_2way_segments(
                    data, header, drive, sensor_col)
            else:
                iq, torque, gear_ratio = _detect_ramp_segment(
                    data, header, drive, sensor_col)
                seg1 = (iq, torque)
        except Exception as exc:
            messagebox.showerror("Segment detection failed", str(exc))
            return

        invert = _detect_torque_inversion(data, header, drive, sensor_col)
        self.invert_var.set(invert)

        def _fits(iq, torque):
            if invert:
                torque = -torque
            return torque, _linear_fit(iq, torque), _linear_fit(iq, torque / gear_ratio)

        tor1, fit1_out, fit1_mot = _fits(*seg1)
        extra_legs = None
        if two_way:
            tor2, fit2_out, fit2_mot = _fits(*seg2)
            tor3, fit3_out, fit3_mot = _fits(*seg3)
            extra_legs = [
                (seg2[0], tor2, fit2_out, fit2_mot),
                (seg3[0], tor3, fit3_out, fit3_mot),
            ]

        _, iq_col, _ = _DRIVE_COLS[drive]
        t_s         = (data[:, header.index("stamp_ns")] - data[0, header.index("stamp_ns")]) * 1e-9
        full_iq     = data[:, header.index(iq_col)]
        full_torque = data[:, header.index(sensor_col)]
        if invert:
            full_torque = -full_torque

        self._csv_path = csv_path
        fig, axes_info = make_kt_figure(
            seg1, tor1, gear_ratio,
            fit1_out, fit1_mot, sensor_col, extra_legs,
            t_s, full_iq, full_torque,
        )
        self._axes_info = axes_info
        self._embed(fig)

        self.status_var.set(
            f"Seg 1 — Kt_output = {fit1_out[0]:.4f} Nm/A  |  "
            f"Kt_motor = {fit1_mot[0]:.4f} Nm/A  (gear ratio: {gear_ratio:.4f})"
            + ("  |  torque inverted" if invert else "")
            + (f"  |  Seg 2 Kt = {fit2_out[0]:.4f}  |  Seg 3 Kt = {fit3_out[0]:.4f}"
               if two_way else "")
        )

    def _embed(self, fig):
        if self._canvas is not None:
            self._canvas.get_tk_widget().destroy()
        if self._toolbar is not None:
            self._toolbar.destroy()
        self._fig    = fig
        self._canvas = FigureCanvasTkAgg(fig, master=self._plot_frame)
        self._canvas.draw()
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._toolbar = NavigationToolbar2Tk(self._canvas, self._plot_frame)
        self._toolbar.update()
        self._save_btn.configure(state="normal")

        def _on_scroll(event):
            if event.inaxes is None:
                return
            factor = 1.15 if event.button == "down" else (1 / 1.15)
            xdata, ydata = event.xdata, event.ydata
            ax = event.inaxes
            ax.set_xlim([xdata + (x - xdata) * factor for x in ax.get_xlim()])
            ax.set_ylim([ydata + (y - ydata) * factor for y in ax.get_ylim()])
            self._canvas.draw_idle()

        self._canvas.mpl_connect("scroll_event", _on_scroll)

    def _save(self):
        if self._fig is None or not self._axes_info:
            return
        default_dir = str(self._csv_path.parent) if self._csv_path else "."
        save_dir = filedialog.askdirectory(
            title="Select folder to save plots", initialdir=default_dir)
        if not save_dir:
            return
        try:
            saved_paths = save_kt_subplots(self._fig, self._axes_info, save_dir)
            self.status_var.set(f"Saved {len(saved_paths)} plots to {save_dir}")
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))


if __name__ == "__main__":
    DynoKtApp().mainloop()
