#!/usr/bin/env python3
"""Small Tk GUI for dyno_pdo.csv Bode plots."""

from __future__ import annotations

import os
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

os.environ.setdefault("MPLCONFIGDIR", "/tmp/dyno_matplotlib")

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from dyno_bode import PRESETS, compute_bode, latest_log, make_bode_figure, read_csv_header, resolve_csv_path


class DynoBodeApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dyno Bode Plot")
        self.geometry("1200x900")
        self._fig = None
        self._canvas = None
        self._toolbar = None
        self._result = None
        self._build_controls()
        self._plot_frame = ttk.LabelFrame(self, text="Preview", padding=4)
        self._plot_frame.pack(fill="both", expand=True, padx=10, pady=10)

    def _build_controls(self):
        ctrl = ttk.LabelFrame(self, text="Inputs", padding=10)
        ctrl.pack(fill="x", padx=10, pady=(10, 0))
        ctrl.columnconfigure(1, weight=1)

        self.path_var = tk.StringVar(value="test_data_log")
        ttk.Label(ctrl, text="CSV / Run / Root:").grid(row=0, column=0, sticky="w")
        ttk.Entry(ctrl, textvariable=self.path_var, width=60).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(ctrl, text="Browse", command=self._browse).grid(row=0, column=2)
        ttk.Button(ctrl, text="Latest", command=self._latest).grid(row=0, column=3, padx=(6, 0))
        ttk.Button(ctrl, text="Load Columns", command=self._load_columns).grid(row=0, column=4, padx=(6, 0))

        self.preset_var = tk.StringVar(value="main_torque_ch1")
        ttk.Label(ctrl, text="Preset:").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Combobox(ctrl, textvariable=self.preset_var, values=sorted(PRESETS), state="readonly", width=28).grid(
            row=1, column=1, sticky="w", padx=6, pady=4)

        self.ref_var = tk.StringVar()
        self.resp_var = tk.StringVar()
        self.ref_scale_var = tk.StringVar(value="1.0")
        ttk.Label(ctrl, text="Ref column:").grid(row=2, column=0, sticky="w", pady=4)
        ref_frame = ttk.Frame(ctrl)
        ref_frame.grid(row=2, column=1, sticky="w", padx=6, pady=4)
        self._ref_combo = ttk.Combobox(ref_frame, textvariable=self.ref_var, width=22)
        self._ref_combo.pack(side="left")
        ttk.Label(ref_frame, text="× scale:").pack(side="left", padx=(10, 2))
        ttk.Entry(ref_frame, textvariable=self.ref_scale_var, width=8).pack(side="left")
        ttk.Label(ctrl, text="Resp column:").grid(row=2, column=2, sticky="w", pady=4)
        self._resp_combo = ttk.Combobox(ctrl, textvariable=self.resp_var, width=22)
        self._resp_combo.grid(row=2, column=3, sticky="w", padx=6, pady=4)

        self.f0_var = tk.StringVar(value="0.1")
        self.f1_var = tk.StringVar(value="10.0")
        self.dur_var = tk.StringVar(value="10.0")
        self.kind_var = tk.StringVar(value="linear")
        ttk.Label(ctrl, text="Chirp start/end/duration:").grid(row=3, column=0, sticky="w", pady=4)
        chirp = ttk.Frame(ctrl)
        chirp.grid(row=3, column=1, columnspan=3, sticky="w", padx=6)
        ttk.Entry(chirp, textvariable=self.f0_var, width=8).pack(side="left")
        ttk.Entry(chirp, textvariable=self.f1_var, width=8).pack(side="left", padx=4)
        ttk.Entry(chirp, textvariable=self.dur_var, width=8).pack(side="left")
        ttk.Combobox(chirp, textvariable=self.kind_var, values=("linear", "exponential"), state="readonly", width=12).pack(
            side="left", padx=8)

        self.t0_var = tk.StringVar()
        self.t1_var = tk.StringVar()
        ttk.Label(ctrl, text="Trim start / end (s):").grid(row=4, column=0, sticky="w", pady=4)
        trim = ttk.Frame(ctrl)
        trim.grid(row=4, column=1, columnspan=3, sticky="w", padx=6)
        ttk.Entry(trim, textvariable=self.t0_var, width=10).pack(side="left")
        ttk.Label(trim, text="to").pack(side="left", padx=6)
        ttk.Entry(trim, textvariable=self.t1_var, width=10).pack(side="left")
        ttk.Label(trim, text="(leave blank for full range)").pack(side="left", padx=8)

        self.lowpass_var = tk.StringVar()
        self.invert_var = tk.BooleanVar(value=False)
        ttk.Label(ctrl, text="Response LP Hz:").grid(row=5, column=0, sticky="w", pady=4)
        ttk.Entry(ctrl, textvariable=self.lowpass_var, width=10).grid(row=5, column=1, sticky="w", padx=6)
        ttk.Checkbutton(ctrl, text="Invert response", variable=self.invert_var).grid(row=5, column=2, sticky="w")
        ttk.Button(ctrl, text="Generate", command=self._generate).grid(row=5, column=3, sticky="e")
        self._save_btn = ttk.Button(ctrl, text="Save", command=self._save, state="disabled")
        self._save_btn.grid(row=5, column=4, sticky="e", padx=(6, 0))

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(ctrl, textvariable=self.status_var, foreground="grey").grid(row=6, column=0, columnspan=4, sticky="w")

    def _browse(self):
        path = filedialog.askopenfilename(title="Select dyno_pdo.csv(.gz)", filetypes=(("CSV / GZ", "*.csv *.csv.gz"), ("All files", "*")))
        if path:
            self.path_var.set(path)
            self._load_columns()

    def _latest(self):
        try:
            self.path_var.set(str(latest_log(self.path_var.get().strip() or "test_data_log")))
            self._load_columns()
        except Exception as exc:
            messagebox.showerror("Latest log failed", str(exc))

    def _load_columns(self):
        try:
            cols = read_csv_header(self.path_var.get().strip() or "test_data_log")
            self._ref_combo["values"] = cols
            self._resp_combo["values"] = cols
            self.status_var.set(f"Loaded {len(cols)} columns.")
        except Exception as exc:
            messagebox.showerror("Load columns failed", str(exc))

    def _float_or_none(self, var: tk.StringVar):
        text = var.get().strip()
        return float(text) if text else None

    def _analysis_status(self, result) -> str:
        parts = []
        if result.f_3db is not None:
            parts.append(f"-3 dB {result.f_3db:.3g} Hz")
        else:
            parts.append("-3 dB not found")
        if result.f_90 is not None:
            parts.append(f"-90 deg {result.f_90:.3g} Hz")
        else:
            parts.append("-90 deg not found")
        return " | ".join(parts)

    def _generate(self):
        try:
            csv_path = resolve_csv_path(self.path_var.get().strip() or "test_data_log")
            lowpass = self._float_or_none(self.lowpass_var)
            result = compute_bode(
                csv_path,
                preset_name=self.preset_var.get(),
                reference=self.ref_var.get().strip() or None,
                response=self.resp_var.get().strip() or None,
                chirp_start_hz=self._float_or_none(self.f0_var),
                chirp_end_hz=self._float_or_none(self.f1_var),
                chirp_duration_s=self._float_or_none(self.dur_var),
                chirp_kind=self.kind_var.get(),
                trim_start_s=self._float_or_none(self.t0_var),
                trim_end_s=self._float_or_none(self.t1_var),
                ref_scale=self._float_or_none(self.ref_scale_var) or 1.0,
                invert_response=bool(self.invert_var.get()),
                lowpass_hz=lowpass,
            )
            fig = make_bode_figure(result, show_raw=lowpass is not None)
        except Exception as exc:
            messagebox.showerror("Bode plot failed", str(exc))
            return

        if self._canvas is not None:
            self._canvas.get_tk_widget().destroy()
        if self._toolbar is not None:
            self._toolbar.destroy()
        self._fig = fig
        self._result = result
        self._canvas = FigureCanvasTkAgg(fig, master=self._plot_frame)
        self._canvas.draw()
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._toolbar = NavigationToolbar2Tk(self._canvas, self._plot_frame)
        self._toolbar.update()
        self._save_btn.configure(state="normal")
        self.status_var.set(self._analysis_status(result))

    def _save(self):
        if self._fig is None:
            return
        default_dir = str(Path(self._result.csv_path).resolve().parent)
        default_name = f"bode_{self.preset_var.get()}.png"
        path = filedialog.asksaveasfilename(
            title="Save Bode Plot",
            initialdir=default_dir,
            initialfile=default_name,
            defaultextension=".png",
            filetypes=(("PNG", "*.png"), ("All files", "*")),
        )
        if not path:
            return
        try:
            self._fig.savefig(path, dpi=150, bbox_inches="tight")
            self.status_var.set(f"Saved {path}")
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))


if __name__ == "__main__":
    DynoBodeApp().mainloop()
