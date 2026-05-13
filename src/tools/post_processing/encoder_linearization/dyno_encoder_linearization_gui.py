#!/usr/bin/env python3
"""Encoder linearization GUI - loads a dyno_pdo.csv from an encoder test,
builds input/output encoder error plots, and exports correction LUTs."""

from __future__ import annotations

import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

os.environ.setdefault("MPLCONFIGDIR", "/tmp/dyno_matplotlib")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from dyno_encoder_linearization_analysis import (
    DEFAULT_LUT_SIZE,
    LUT_SIZE_CHOICES,
    _DRIVE_COLS,
    _latest_log,
    analyze_encoder_linearization,
    make_encoder_linearization_figure,
    save_encoder_subplots,
    save_lut_csv,
)


class DynoEncoderLinearizationApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dyno Encoder Linearization")
        self.geometry("1400x900")
        self._fig = None
        self._canvas = None
        self._toolbar = None
        self._axes_info: list[tuple] = []
        self._result = None
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
        ttk.Button(ctrl, text="Latest", command=self._latest).grid(
            row=0, column=3, padx=(6, 0))

        self.drive_var = tk.StringVar(value="auto")
        self.lut_size_var = tk.StringVar(value=str(DEFAULT_LUT_SIZE))
        ttk.Label(ctrl, text="Drive:").grid(row=1, column=0, sticky="w", pady=6)
        ttk.Combobox(ctrl, textvariable=self.drive_var,
                     values=["auto", *sorted(_DRIVE_COLS)], state="readonly", width=8).grid(
            row=1, column=1, sticky="w", padx=6)
        ttk.Label(ctrl, text="LUT size:").grid(row=1, column=2, sticky="w")
        ttk.Combobox(ctrl, textvariable=self.lut_size_var,
                     values=[str(v) for v in LUT_SIZE_CHOICES], width=10).grid(
            row=1, column=3, sticky="w", padx=6)

        btn_row = ttk.Frame(ctrl)
        btn_row.grid(row=2, column=0, columnspan=4, sticky="e", pady=(4, 0))
        ttk.Button(btn_row, text="Generate", command=self._generate).pack(side="left")
        self._save_btn = ttk.Button(btn_row, text="Save LUT/Plots",
                                    command=self._save, state="disabled")
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

    def _lut_size(self) -> int:
        text = self.lut_size_var.get().strip()
        try:
            return int(text)
        except ValueError as exc:
            raise ValueError("LUT size must be an integer.") from exc

    def _generate(self):
        folder = self.path_var.get().strip() or "test_data_log"
        drive = self.drive_var.get()
        try:
            result = analyze_encoder_linearization(
                folder,
                drive=drive,
                lut_size=self._lut_size(),
            )
            fig, axes_info = make_encoder_linearization_figure(result)
        except Exception as exc:
            messagebox.showerror("Encoder linearization failed", str(exc))
            return

        self._result = result
        self._axes_info = axes_info
        self._embed(fig)
        self.status_var.set(
            f"Drive {result.drive}: input rms {result.input_side.rms_rad:.6g} rad, "
            f"output rms {result.output_side.rms_rad:.6g} rad, "
            f"LUT {result.lut_size}, input revs {result.input_revolution_count}, "
            f"samples {result.sample_count}"
        )

    def _embed(self, fig):
        if self._canvas is not None:
            self._canvas.get_tk_widget().destroy()
        if self._toolbar is not None:
            self._toolbar.destroy()
        self._fig = fig
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
        if self._fig is None or self._result is None or not self._axes_info:
            return
        default_dir = str(self._result.csv_path.parent)
        save_dir = filedialog.askdirectory(
            title="Select folder to save LUT and plots",
            initialdir=default_dir,
        )
        if not save_dir:
            return
        try:
            plots = save_encoder_subplots(self._fig, self._axes_info, save_dir)
            lut_path = save_lut_csv(self._result, save_dir)
            self.status_var.set(
                f"Saved LUT to {lut_path} and {len(plots)} plots to {save_dir}")
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))


if __name__ == "__main__":
    DynoEncoderLinearizationApp().mainloop()
