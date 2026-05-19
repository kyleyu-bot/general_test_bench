#!/usr/bin/env python3
"""
Dyno Live Plot — drag-and-drop live plotting from ROS2 topics.

Usage (from repo root, after sourcing ROS2):
    python3 src/interface_bridges/ros2/dyno_plot.py

Or via sudo (if bridge_ros2 is running as root with UDP DDS):
    bash src/interface_bridges/ros2/run_plot.sh

Drag any field from the Topic Browser on the left onto a plot cell on the right.
Right-click a plot cell to remove individual curves or clear all.
"""

import json
import sys
import threading
import time
from collections import deque

import numpy as np

try:
    from PyQt5.QtCore    import Qt, QTimer, QMimeData, QByteArray
    from PyQt5.QtGui     import QDrag
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget,
        QVBoxLayout, QHBoxLayout, QGridLayout,
        QSplitter, QLabel, QSpinBox, QPushButton,
        QTreeWidget, QTreeWidgetItem, QSizePolicy,
        QMenu, QAction, QFrame, QScrollArea,
    )
except ImportError:
    print("ERROR: PyQt5 not found.  pip install PyQt5", file=sys.stderr)
    sys.exit(1)

try:
    import pyqtgraph as pg
except ImportError:
    print("ERROR: pyqtgraph not found.  pip install pyqtgraph", file=sys.stderr)
    sys.exit(1)

try:
    import rclpy
    from rclpy.node       import Node
    from std_msgs.msg     import String as StringMsg
    from std_msgs.msg     import Float64, UInt32
except ImportError:
    print("ERROR: rclpy not found.  source /opt/ros/humble/setup.bash", file=sys.stderr)
    sys.exit(1)

# ── Constants ──────────────────────────────────────────────────────────────────

MIME_TYPE    = "application/x-dyno-field"
DEFAULT_HZ   = 200          # expected incoming data rate (for cache sizing)
DEFAULT_WIN  = 10           # seconds visible by default
DEFAULT_MIN  = 1            # minimum window (s)
DEFAULT_MAX  = 60           # maximum window / cache (s)

JSON_TOPICS  = [
    "/dyno/main_drive/status",
    "/dyno/dut/status",
    "/dyno/loop/stats",
    "/dyno/command",
    "/dyno/rt_command",
    "/dyno/sdo_request",
    "/dyno/sdo_response",
]
FLOAT_TOPICS = [
    "/dyno/torque/ch1",
    "/dyno/torque/ch2",
]
UINT_TOPICS  = [
    "/dyno/encoder/count",
]

CURVE_COLORS = [
    "#e74c3c", "#3498db", "#2ecc71", "#f39c12",
    "#9b59b6", "#1abc9c", "#e67e22", "#ecf0f1",
]


# ── DataStore ──────────────────────────────────────────────────────────────────

class DataStore:
    """
    Thread-safe ring-buffer keyed by (topic, field).
    Timestamps and values are stored in separate deques to avoid per-call
    tuple unpacking.  get() uses np.searchsorted (O log N) to slice the
    window instead of a O(N) list comprehension.
    """

    def __init__(self, max_samples: int):
        self._lock    = threading.Lock()
        self._ts:     dict[tuple[str, str], deque] = {}   # monotonic timestamps
        self._vs:     dict[tuple[str, str], deque] = {}   # float values
        self._fields: dict[str, list[str]]         = {}
        self._max     = max_samples

    # ── write ──────────────────────────────────────────────────────────────────

    def push(self, topic: str, field: str, t: float, value: float) -> None:
        key = (topic, field)
        with self._lock:
            if key not in self._ts:
                self._ts[key] = deque(maxlen=self._max)
                self._vs[key] = deque(maxlen=self._max)
                self._fields.setdefault(topic, [])
                if field not in self._fields[topic]:
                    self._fields[topic].append(field)
            self._ts[key].append(t)
            self._vs[key].append(value)

    # ── read ───────────────────────────────────────────────────────────────────

    def get(self, topic: str, field: str, window_s: float):
        """Return (times_relative, values) numpy arrays for the last window_s."""
        key = (topic, field)
        with self._lock:
            ts_buf = self._ts.get(key)
            if not ts_buf:
                return np.array([]), np.array([])
            # Copy inside the lock so the ROS thread can keep appending.
            ts = np.array(ts_buf, dtype=np.float64)
            vs = np.array(self._vs[key], dtype=np.float64)

        if len(ts) == 0:
            return np.array([]), np.array([])

        t_end = ts[-1]
        # Binary search for the first sample inside the window — O(log N).
        idx   = np.searchsorted(ts, t_end - window_s, side="left")
        return ts[idx:] - t_end, vs[idx:]

    def known_fields(self) -> dict[str, list[str]]:
        with self._lock:
            return {t: list(f) for t, f in self._fields.items()}

    # ── resize cache ───────────────────────────────────────────────────────────

    def set_max_samples(self, n: int) -> None:
        with self._lock:
            self._max = n
            for key in list(self._ts):
                self._ts[key] = deque(self._ts[key], maxlen=n)
                self._vs[key] = deque(self._vs[key], maxlen=n)


# ── ROS2 subscriber node ───────────────────────────────────────────────────────

class DynoPlotNode(Node):
    """Subscribes to all /dyno/* topics and pushes data into the DataStore."""

    def __init__(self, store: DataStore):
        super().__init__("dyno_plot")
        self._store = store

        for topic in JSON_TOPICS:
            self.create_subscription(
                StringMsg, topic,
                lambda msg, t=topic: self._on_json(msg, t), 10,
            )
        for topic in FLOAT_TOPICS:
            self.create_subscription(
                Float64, topic,
                lambda msg, t=topic: self._on_float(msg, t), 10,
            )
        for topic in UINT_TOPICS:
            self.create_subscription(
                UInt32, topic,
                lambda msg, t=topic: self._on_uint(msg, t), 10,
            )

    def _on_json(self, msg: StringMsg, topic: str) -> None:
        now = time.monotonic()
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        for field, value in data.items():
            try:
                self._store.push(topic, field, now, float(value))
            except (TypeError, ValueError):
                pass

    def _on_float(self, msg: Float64, topic: str) -> None:
        self._store.push(topic, "torque_nm", time.monotonic(), msg.data)

    def _on_uint(self, msg: UInt32, topic: str) -> None:
        self._store.push(topic, "value", time.monotonic(), float(msg.data))


# ── TopicBrowser ───────────────────────────────────────────────────────────────

class TopicBrowser(QTreeWidget):
    """
    Tree showing topics → fields.  Field items are draggable.
    Mime payload: b"<topic>::<field>"
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setHeaderLabel("Topics / Fields")
        self.setDragEnabled(True)
        self.setDragDropMode(QTreeWidget.DragOnly)
        self.setSelectionMode(QTreeWidget.SingleSelection)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

        self._topic_items: dict[str, QTreeWidgetItem] = {}
        self._known:       dict[str, set[str]]        = {}

        # Clicking anywhere on a topic row toggles expand/collapse.
        self.itemClicked.connect(self._on_item_clicked)

    def _on_item_clicked(self, item: QTreeWidgetItem, _col: int) -> None:
        if item.parent() is None:   # top-level = topic row
            item.setExpanded(not item.isExpanded())

    def refresh(self, fields: dict[str, list[str]]) -> None:
        for topic in sorted(fields):
            flist = fields[topic]
            if topic not in self._topic_items:
                self._known[topic] = set()
                item = QTreeWidgetItem(self, [topic])
                item.setFlags(item.flags() & ~Qt.ItemIsDragEnabled)
                item.setExpanded(True)
                self._topic_items[topic] = item

            topic_item = self._topic_items[topic]
            for field in flist:
                if field not in self._known[topic]:
                    self._known[topic].add(field)
                    child = QTreeWidgetItem(topic_item, [field])
                    child.setData(0, Qt.UserRole, f"{topic}::{field}")
                    child.setFlags(child.flags() | Qt.ItemIsDragEnabled)

    def mimeData(self, items):
        mime = QMimeData()
        if items:
            payload = items[0].data(0, Qt.UserRole) or ""
            mime.setData(MIME_TYPE, QByteArray(payload.encode()))
        return mime


# ── PlotCell ───────────────────────────────────────────────────────────────────

class PlotCell(pg.PlotWidget):
    """
    A single pyqtgraph plot that accepts dropped fields.
    Multiple curves can be plotted simultaneously.
    Right-click to remove individual curves or clear all.
    """

    def __init__(self, store: DataStore, parent=None):
        super().__init__(parent, background="#1a1a2e")
        self._store      = store
        self._curves:    list[dict] = []   # {topic, field, item}
        self._color_idx: int        = 0

        self.setAcceptDrops(True)
        self.addLegend(offset=(5, 5))
        self.showGrid(x=True, y=True, alpha=0.25)
        self.setLabel("bottom", "time (s)")
        self.setMinimumSize(200, 150)

    # ── drag / drop ────────────────────────────────────────────────────────────

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasFormat(MIME_TYPE):
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragMoveEvent(self, ev):
        ev.acceptProposedAction()

    def dropEvent(self, ev):
        raw = bytes(ev.mimeData().data(MIME_TYPE)).decode()
        if "::" not in raw:
            ev.ignore()
            return
        topic, field = raw.split("::", 1)
        self._add_curve(topic, field)
        ev.acceptProposedAction()

    # ── curves ─────────────────────────────────────────────────────────────────

    def _add_curve(self, topic: str, field: str) -> None:
        for c in self._curves:
            if c["topic"] == topic and c["field"] == field:
                return  # already present

        color  = CURVE_COLORS[self._color_idx % len(CURVE_COLORS)]
        self._color_idx += 1
        label  = f"{topic.split('/')[-1]}.{field}"
        item   = self.plot(name=label, pen=pg.mkPen(color, width=1.5))
        self._curves.append({"topic": topic, "field": field, "item": item})

    def _remove_curve(self, idx: int) -> None:
        c = self._curves.pop(idx)
        self.removeItem(c["item"])

    def _clear(self) -> None:
        for c in self._curves:
            self.removeItem(c["item"])
        self._curves.clear()

    # ── update ─────────────────────────────────────────────────────────────────

    # Maximum points rendered per curve — keeps GPU/CPU load bounded.
    _MAX_DISPLAY_PTS = 2000

    def update_curves(self, window_s: float) -> None:
        for c in self._curves:
            ts, vs = self._store.get(c["topic"], c["field"], window_s)
            n = len(ts)
            if n == 0:
                continue
            if n > self._MAX_DISPLAY_PTS:
                step = n // self._MAX_DISPLAY_PTS
                ts = ts[::step]
                vs = vs[::step]
            c["item"].setData(ts, vs)

    # ── context menu ───────────────────────────────────────────────────────────

    def contextMenuEvent(self, ev):
        menu = QMenu(self)
        for i, c in enumerate(self._curves):
            label = f"{c['topic'].split('/')[-1]}.{c['field']}"
            act   = QAction(f"Remove: {label}", self)
            act.triggered.connect(lambda _, idx=i: self._remove_curve(idx))
            menu.addAction(act)
        if self._curves:
            menu.addSeparator()
        clear_act = QAction("Clear all", self)
        clear_act.triggered.connect(self._clear)
        menu.addAction(clear_act)
        menu.exec_(ev.globalPos())


# ── PlotGrid ───────────────────────────────────────────────────────────────────

class PlotGrid(QWidget):
    """Resizable grid of PlotCell widgets."""

    def __init__(self, store: DataStore, rows: int = 2, cols: int = 2, parent=None):
        super().__init__(parent)
        self._store  = store
        self._rows   = rows
        self._cols   = cols
        self._cells: list[list[PlotCell]] = []

        self._layout = QGridLayout(self)
        self._layout.setSpacing(4)
        self._rebuild()

    def _rebuild(self) -> None:
        for row in self._cells:
            for cell in row:
                self._layout.removeWidget(cell)
                cell.deleteLater()
        self._cells = []

        for r in range(self._rows):
            row_cells = []
            for c in range(self._cols):
                cell = PlotCell(self._store, self)
                self._layout.addWidget(cell, r, c)
                row_cells.append(cell)
            self._cells.append(row_cells)

    def set_dims(self, rows: int, cols: int) -> None:
        if rows == self._rows and cols == self._cols:
            return
        self._rows = rows
        self._cols = cols
        self._rebuild()

    def update_all(self, window_s: float) -> None:
        for row in self._cells:
            for cell in row:
                cell.update_curves(window_s)


# ── DisplayBox ─────────────────────────────────────────────────────────────────

class DisplayBox(QFrame):
    """Drop-target cell showing the live current / min / max of a (topic, field) pair."""

    def __init__(self, store: DataStore, parent=None):
        super().__init__(parent)
        self._store: DataStore    = store
        self._topic: "str | None" = None
        self._field: "str | None" = None

        self.setAcceptDrops(True)
        self.setFrameShape(QFrame.Box)
        self.setLineWidth(1)
        self.setStyleSheet(
            "DisplayBox { background: #1a1a2e; border: 1px solid #444; }")
        self.setMinimumHeight(80)

        vlay = QVBoxLayout(self)
        vlay.setContentsMargins(6, 4, 6, 4)
        vlay.setSpacing(2)

        self._lbl_name = QLabel("— drop field —")
        self._lbl_name.setAlignment(Qt.AlignCenter)
        self._lbl_name.setStyleSheet("color: #888; font-size: 13px;")

        self._lbl_value = QLabel("")
        self._lbl_value.setAlignment(Qt.AlignCenter)
        self._lbl_value.setStyleSheet(
            "color: #2ecc71; font-size: 26px; font-family: monospace; font-weight: bold;")

        self._lbl_stats = QLabel("")
        self._lbl_stats.setAlignment(Qt.AlignCenter)
        self._lbl_stats.setStyleSheet("color: #888; font-size: 12px;")

        vlay.addWidget(self._lbl_name)
        vlay.addWidget(self._lbl_value)
        vlay.addWidget(self._lbl_stats)

    # ── live update ────────────────────────────────────────────────────────────

    def update(self, window_s: float) -> None:
        if self._topic is None:
            return
        _, vs = self._store.get(self._topic, self._field, window_s)
        if len(vs) == 0:
            return
        self._lbl_value.setText(f"{vs[-1]:.6g}")
        self._lbl_stats.setText(f"min {vs.min():.6g}   max {vs.max():.6g}")

    # ── internal ───────────────────────────────────────────────────────────────

    def _set_field(self, topic: str, field: str) -> None:
        self._topic = topic
        self._field = field
        self._lbl_name.setText(f"{topic.split('/')[-1]}.{field}")
        self._lbl_name.setStyleSheet("color: #ecf0f1; font-size: 13px;")
        self._lbl_value.setText("")
        self._lbl_stats.setText("")

    def _clear(self) -> None:
        self._topic = None
        self._field = None
        self._lbl_name.setText("— drop field —")
        self._lbl_name.setStyleSheet("color: #888; font-size: 13px;")
        self._lbl_value.setText("")
        self._lbl_stats.setText("")

    # ── drag / drop ────────────────────────────────────────────────────────────

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasFormat(MIME_TYPE):
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragMoveEvent(self, ev):
        ev.acceptProposedAction()

    def dropEvent(self, ev):
        raw = bytes(ev.mimeData().data(MIME_TYPE)).decode()
        if "::" not in raw:
            ev.ignore()
            return
        topic, field = raw.split("::", 1)
        self._set_field(topic, field)
        ev.acceptProposedAction()

    def contextMenuEvent(self, ev):
        menu = QMenu(self)
        if self._topic:
            label = f"{self._topic.split('/')[-1]}.{self._field}"
            menu.addAction(f"Clear: {label}", self._clear)
        else:
            menu.addAction("(empty)", lambda: None).setEnabled(False)
        menu.exec_(ev.globalPos())


# ── DisplayPanel ───────────────────────────────────────────────────────────────

class DisplayPanel(QWidget):
    """Scrollable column of live DisplayBox widgets."""

    def __init__(self, store: DataStore, parent=None):
        super().__init__(parent)
        self._store  = store
        self._boxes: list[DisplayBox] = []

        self._inner = QWidget()
        self._vlay  = QVBoxLayout(self._inner)
        self._vlay.setSpacing(4)
        self._vlay.setContentsMargins(4, 4, 4, 4)
        self._vlay.addStretch(1)   # sentinel — always at the end

        scroll = QScrollArea()
        scroll.setWidget(self._inner)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self._set_count(4)

    def set_count(self, n: int) -> None:
        self._set_count(n)

    def update(self, window_s: float) -> None:
        for box in self._boxes:
            box.update(window_s)

    def _set_count(self, n: int) -> None:
        while len(self._boxes) > n:
            box = self._boxes.pop()
            self._vlay.removeWidget(box)
            box.deleteLater()
        while len(self._boxes) < n:
            box = DisplayBox(self._store, self._inner)
            self._vlay.insertWidget(len(self._boxes), box)
            self._boxes.append(box)


# ── DynoPlotWindow ─────────────────────────────────────────────────────────────

class DynoPlotWindow(QMainWindow):
    """Main window: topic browser on the left, plot grid on the right."""

    def __init__(self, store: DataStore):
        super().__init__()
        self._store = store
        self.setWindowTitle("Dyno Live Plot")
        self.resize(1280, 720)

        pg.setConfigOptions(antialias=False, useNumba=False)

        # ── Controls bar ───────────────────────────────────────────────────────
        ctrl     = QWidget()
        ctrl_lay = QHBoxLayout(ctrl)
        ctrl_lay.setContentsMargins(6, 4, 6, 4)
        ctrl_lay.setSpacing(12)

        def labelled_spin(label: str, lo: int, hi: int, val: int, tip: str = "") -> QSpinBox:
            w = QWidget()
            h = QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(4)
            h.addWidget(QLabel(label))
            s = QSpinBox()
            s.setRange(lo, hi)
            s.setValue(val)
            s.setFixedWidth(60)
            if tip:
                s.setToolTip(tip)
            h.addWidget(s)
            ctrl_lay.addWidget(w)
            return s

        self._rows_spin    = labelled_spin("Rows:",       1,   6,  2)
        self._cols_spin    = labelled_spin("Cols:",       1,   6,  2)

        ctrl_lay.addWidget(_vline())

        self._displays_spin = labelled_spin("Displays:", 0, 20, 4)

        ctrl_lay.addWidget(_vline())

        self._min_spin     = labelled_spin("Min (s):",    1,  600,  DEFAULT_MIN,
                                           "Minimum history window")
        self._win_spin     = labelled_spin("Window (s):", 1,  600,  DEFAULT_WIN,
                                           "Visible history window")
        self._max_spin     = labelled_spin("Max (s):",    1, 3600,  DEFAULT_MAX,
                                           "Maximum cache size")

        ctrl_lay.addStretch()

        # ── Browser + grid ─────────────────────────────────────────────────────
        self._browser = TopicBrowser()
        self._browser.setMaximumWidth(280)
        self._browser.setMinimumWidth(160)

        self._grid = PlotGrid(store, rows=2, cols=2)

        self._display_panel = DisplayPanel(store)
        self._display_panel.setMinimumWidth(160)
        self._display_panel.setMaximumWidth(280)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._browser)
        splitter.addWidget(self._grid)
        splitter.addWidget(self._display_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([220, 840, 220])

        # ── Layout ─────────────────────────────────────────────────────────────
        central  = QWidget()
        vlay     = QVBoxLayout(central)
        vlay.setContentsMargins(4, 4, 4, 4)
        vlay.setSpacing(4)
        vlay.addWidget(ctrl)
        vlay.addWidget(splitter, 1)
        self.setCentralWidget(central)

        # ── Connections ────────────────────────────────────────────────────────
        self._rows_spin.valueChanged.connect(self._on_dims_changed)
        self._cols_spin.valueChanged.connect(self._on_dims_changed)
        self._displays_spin.valueChanged.connect(self._on_display_count_changed)
        self._min_spin.valueChanged.connect(self._on_limits_changed)
        self._max_spin.valueChanged.connect(self._on_limits_changed)
        self._win_spin.valueChanged.connect(self._clamp_window)

        # Initial cache size
        self._store.set_max_samples(DEFAULT_MAX * DEFAULT_HZ)

        # ── Timers ─────────────────────────────────────────────────────────────
        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._update_plots)
        self._plot_timer.start(50)   # 20 Hz redraw — plenty for visual smoothness

        self._browser_timer = QTimer(self)
        self._browser_timer.timeout.connect(self._refresh_browser)
        self._browser_timer.start(500)  # poll for new fields every 0.5 s

    # ── slots ──────────────────────────────────────────────────────────────────

    def _on_dims_changed(self) -> None:
        self._grid.set_dims(self._rows_spin.value(), self._cols_spin.value())

    def _on_display_count_changed(self) -> None:
        self._display_panel.set_count(self._displays_spin.value())

    def _on_limits_changed(self) -> None:
        lo = self._min_spin.value()
        hi = self._max_spin.value()
        # Keep min ≤ max
        if lo > hi:
            self._max_spin.blockSignals(True)
            self._max_spin.setValue(lo)
            self._max_spin.blockSignals(False)
            hi = lo
        self._store.set_max_samples(hi * DEFAULT_HZ)
        self._clamp_window()

    def _clamp_window(self) -> None:
        lo = self._min_spin.value()
        hi = self._max_spin.value()
        v  = self._win_spin.value()
        self._win_spin.blockSignals(True)
        self._win_spin.setRange(lo, hi)
        self._win_spin.setValue(max(lo, min(hi, v)))
        self._win_spin.blockSignals(False)

    def _update_plots(self) -> None:
        window_s = float(self._win_spin.value())
        self._grid.update_all(window_s)
        self._display_panel.update(window_s)

    def _refresh_browser(self) -> None:
        self._browser.refresh(self._store.known_fields())


# ── Helpers ────────────────────────────────────────────────────────────────────

def _vline() -> QWidget:
    """Thin vertical separator for the controls bar."""
    line = QWidget()
    line.setFixedWidth(1)
    line.setStyleSheet("background: #555;")
    line.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
    return line


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()

    store = DataStore(max_samples=DEFAULT_MAX * DEFAULT_HZ)
    node  = DynoPlotNode(store)

    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(node),
        daemon=True,
    )
    ros_thread.start()

    app    = QApplication(sys.argv)
    window = DynoPlotWindow(store)
    window.show()

    ret = app.exec_()

    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
