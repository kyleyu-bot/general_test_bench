#!/usr/bin/env python3
"""Inertia estimation GUI — loads a JIPT inertia_estimation test log and
computes rotor inertia (kg·m²) from the forward/reverse torque phases."""

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

from dyno_inertia_analysis import _latest_log, run_inertia_analysis


class _PlotTab(ttk.Frame):
    """A notebook tab that holds a single matplotlib figure."""

    def __init__(self, parent):
        super().__init__(parent)
        self._canvas  = None
        self._toolbar = None

    def embed(self, fig):
        if self._canvas is not None:
            self._canvas.get_tk_widget().destroy()
        if self._toolbar is not None:
            self._toolbar.destroy()

        self._canvas = FigureCanvasTkAgg(fig, master=self)
        self._canvas.draw()
        self._canvas.get_tk_widget().pack(fill="both", expand=True)

        toolbar_frame = ttk.Frame(self)
        toolbar_frame.pack(fill="x")
        self._toolbar = NavigationToolbar2Tk(self._canvas, toolbar_frame)
        self._toolbar.update()

        def _on_scroll(event):
            if event.inaxes is None:
                return
            factor = 1.15 if event.button == "down" else (1 / 1.15)
            ax = event.inaxes
            xdata, ydata = event.xdata, event.ydata
            ax.set_xlim([xdata + (x - xdata) * factor for x in ax.get_xlim()])
            ax.set_ylim([ydata + (y - ydata) * factor for y in ax.get_ylim()])
            self._canvas.draw_idle()

        self._canvas.mpl_connect("scroll_event", _on_scroll)


class DynoInertiaApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Inertia Estimation")
        self.geometry("1300x820")
        self._build_controls()
        self._build_notebook()

    def _build_controls(self):
        ctrl = ttk.LabelFrame(self, text="Inputs", padding=10)
        ctrl.pack(fill="x", padx=10, pady=(10, 0))
        ctrl.columnconfigure(1, weight=1)

        # Row 0 — folder
        self.path_var = tk.StringVar(value="test_data_log")
        ttk.Label(ctrl, text="Run folder:").grid(row=0, column=0, sticky="w")
        ttk.Entry(ctrl, textvariable=self.path_var, width=60).grid(
            row=0, column=1, sticky="ew", padx=6)
        ttk.Button(ctrl, text="Browse", command=self._browse).grid(row=0, column=2)
        ttk.Button(ctrl, text="Latest",  command=self._latest).grid(
            row=0, column=3, padx=(6, 0))

        # Row 1 — Kt
        ttk.Label(ctrl, text="Kt (Nm/A):").grid(
            row=1, column=0, sticky="w", pady=6)
        self.kt_var = tk.StringVar(value="0.1")
        ttk.Entry(ctrl, textvariable=self.kt_var, width=10).grid(
            row=1, column=1, sticky="w", padx=6)

        # Row 2 — duration + min accel
        ttk.Label(ctrl, text="Accel duration (s):").grid(
            row=2, column=0, sticky="w", pady=(0, 6))
        self.duration_var = tk.StringVar(value="2.0")
        dur_entry = ttk.Entry(ctrl, textvariable=self.duration_var, width=10)
        dur_entry.grid(row=2, column=1, sticky="w", padx=6)
        ttk.Label(ctrl, text="(leave blank to use all torque-on samples)").grid(
            row=2, column=2, columnspan=2, sticky="w", padx=6)

        ttk.Label(ctrl, text="Min |accel| (rad/s²):").grid(
            row=3, column=0, sticky="w", pady=(0, 6))
        self.min_accel_var = tk.DoubleVar(value=0.5)
        ttk.Entry(ctrl, textvariable=self.min_accel_var, width=10).grid(
            row=3, column=1, sticky="w", padx=6)

        # Row 4 — generate button
        btn_row = ttk.Frame(ctrl)
        btn_row.grid(row=4, column=0, columnspan=4, sticky="e", pady=(4, 0))
        ttk.Button(btn_row, text="Generate", command=self._generate).pack(side="left")

        # Row 5 — status
        self.status_var = tk.StringVar(value="Ready.")
        self._status_lbl = ttk.Label(ctrl, textvariable=self.status_var,
                                     foreground="grey")
        self._status_lbl.grid(row=5, column=0, columnspan=4, sticky="w")

    def _build_notebook(self):
        nb_frame = ttk.Frame(self)
        nb_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self._nb = ttk.Notebook(nb_frame)
        self._nb.pack(fill="both", expand=True)

        self._tab_time = _PlotTab(self._nb)
        self._tab_dist = _PlotTab(self._nb)
        self._nb.add(self._tab_time, text="Time Series")
        self._nb.add(self._tab_dist, text="Distribution")

    def _browse(self):
        path = filedialog.askdirectory(title="Select run folder")
        if path:
            self.path_var.set(path)

    def _latest(self):
        try:
            folder = _latest_log(self.path_var.get().strip() or "test_data_log")
            self.path_var.set(str(folder))
            self.status_var.set(f"Loaded: {folder}")
            self._status_lbl.configure(foreground="grey")
        except Exception as exc:
            messagebox.showerror("Latest log failed", str(exc))

    def _generate(self):
        folder = self.path_var.get().strip() or "test_data_log"

        try:
            kt = float(self.kt_var.get())
            if kt <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid input", "Kt must be a positive number (Nm/A).")
            return

        # Parse optional duration
        dur_str = self.duration_var.get().strip()
        if dur_str == "":
            duration_s = None
        else:
            try:
                duration_s = float(dur_str)
                if duration_s <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Invalid input",
                                     "Accel duration must be a positive number or blank.")
                return

        try:
            min_accel = float(self.min_accel_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Min accel must be a number.")
            return

        try:
            inertia, fig_time, fig_dist = run_inertia_analysis(
                folder, kt=kt, duration_s=duration_s, min_accel=min_accel
            )
        except FileNotFoundError as exc:
            messagebox.showerror("File not found", str(exc))
            return
        except Exception as exc:
            messagebox.showerror("Analysis failed", str(exc))
            return

        self._tab_time.embed(fig_time)
        self._tab_dist.embed(fig_dist)
        # Switch to time series tab after generate
        self._nb.select(self._tab_time)

        result = f"J = {inertia:.6f} kg·m²"
        self.status_var.set(result)
        self._status_lbl.configure(foreground="darkgreen")
        self.title(f"Inertia Estimation — {result}")


if __name__ == "__main__":
    DynoInertiaApp().mainloop()
