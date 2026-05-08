#!/usr/bin/env python3
"""Cogging torque analysis GUI — loads a dyno_pdo.csv from a cogging test,
detects forward/reverse segments, resamples to encoder space, and runs FFT."""

from __future__ import annotations

import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

os.environ.setdefault("MPLCONFIGDIR", "/tmp/dyno_matplotlib")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from dyno_cogging_analysis import (
    _DRIVE_COLS,
    _find_csv, _latest_log, _load_csv,
    _detect_segments, _resample_to_enc_space, _compute_fft,
    make_cogging_figure, make_cogging_avg_fft_figure,
)


class DynoCoggingApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dyno Cogging Analysis")
        self.geometry("1400x900")
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

        self.drive_var = tk.StringVar(value="main")
        ttk.Label(ctrl, text="Drive:").grid(row=1, column=0, sticky="w", pady=6)
        ttk.Combobox(ctrl, textvariable=self.drive_var,
                     values=["main", "dut"], state="readonly", width=8).grid(
            row=1, column=1, sticky="w", padx=6)

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
        folder = self.path_var.get().strip() or "test_data_log"
        drive  = self.drive_var.get()

        try:
            csv_path = _find_csv(folder)
        except FileNotFoundError as exc:
            messagebox.showerror("Wrong folder", str(exc))
            return

        try:
            data, header = _load_csv(csv_path)
        except Exception as exc:
            messagebox.showerror("Load failed", str(exc))
            return

        vel_col, iq_col, enc_col = _DRIVE_COLS[drive]
        for col in (vel_col, iq_col, enc_col):
            if col not in header:
                messagebox.showerror(
                    "Column missing",
                    f"Column '{col}' not found. Is this a cogging test log?")
                return

        try:
            t_start, t_mid, t_end = _detect_segments(data, header, drive)
        except Exception as exc:
            messagebox.showerror("Segment detection failed", str(exc))
            return

        iq_full  = data[:, header.index(iq_col)]
        vel_full = data[:, header.index(vel_col)]
        enc_full = data[:, header.index(enc_col)]
        t_s      = (data[:, header.index("stamp_ns")]
                    - data[0, header.index("stamp_ns")]) * 1e-9

        seg1_enc = enc_full[t_start : t_mid + 1]
        seg1_iq  = iq_full[t_start  : t_mid + 1]
        seg2_enc = enc_full[t_mid   : t_end + 1]
        seg2_iq  = iq_full[t_mid    : t_end + 1]

        common_enc, iq1 = _resample_to_enc_space(seg1_enc, seg1_iq)
        _,          iq2 = _resample_to_enc_space(seg2_enc, seg2_iq)
        iq_avg          = (iq1 + iq2) / 2.0

        harmonics1, fft_seg1 = _compute_fft(iq1)
        harmonics2, fft_seg2 = _compute_fft(iq2)
        harmonics_avg, fft_avg = _compute_fft(iq_avg)

        dom1 = int(np.argmax(fft_seg1[1:]) + 1)
        dom2 = int(np.argmax(fft_seg2[1:]) + 1)
        doma = int(np.argmax(fft_avg[1:])  + 1)

        self._csv_path = csv_path
        self._avg_axes: list[tuple] = []

        fig, axes_info = make_cogging_figure(
            common_enc, iq1, iq2, iq_avg,
            fft_seg1, fft_seg2, fft_avg,
            harmonics1, t_s, iq_full, vel_full, drive,
        )
        fig_avg, axes_avg = make_cogging_avg_fft_figure(
            common_enc, iq_avg, fft_avg, harmonics_avg,
        )
        self._fig_avg  = fig_avg
        self._avg_axes = axes_avg
        self._embed(fig)
        self._axes_info = axes_info + axes_avg

        self.status_var.set(
            f"Forward dominant harmonic: {dom1}  |  "
            f"Reverse dominant harmonic: {dom2}  |  "
            f"Averaged dominant harmonic: {doma}"
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
            for fig, axes_info in [
                (self._fig,     [a for a in self._axes_info if a not in self._avg_axes]),
                (self._fig_avg, self._avg_axes),
            ]:
                for ax, stem in axes_info:
                    path = os.path.join(save_dir, f"cogging_{stem}.png")
                    fig.savefig(path, dpi=150, bbox_inches="tight")
            self.status_var.set(
                f"Saved {len(self._axes_info)} plots to {save_dir}")
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))


if __name__ == "__main__":
    DynoCoggingApp().mainloop()
