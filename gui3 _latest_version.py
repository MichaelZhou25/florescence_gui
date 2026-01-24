# -*- coding: utf-8 -*-
"""
荧光显示仪 · 试纸条T/C峰值识别版 (带浓度输入控制)
功能：
1. 串口读取荧光强度数据并绘制实时曲线
2. 自动识别T/C双峰值（T在前，C在后）
3. 界面动态增加“当前浓度”输入框，未输入禁止开始
4. 导出CSV包含浓度信息，时间从0ms开始
5. 显示信息中包含浓度值
"""
import sys, re, os, time, csv, platform
from typing import Optional, Deque, Tuple, List, cast, Type, TypeVar, Any, Dict
from collections import deque
from datetime import datetime

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import QThread, Signal

import serial
import serial.tools.list_ports as list_ports

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from scipy.signal import argrelextrema
from dataclasses import dataclass

# ---------------------------
# 跨平台字体配置
# ---------------------------
def get_monospace_font():
    """获取跨平台等宽字体"""
    system = platform.system()
    if system == "Windows":
        return "Consolas"
    elif system == "Darwin":  # macOS
        return "Menlo"
    else:  # Linux
        return "Monospace"

def setup_matplotlib_font():
    """配置Matplotlib支持中文"""
    import matplotlib.pyplot as plt
    system = platform.system()
    font_configs = {
        "Windows": ["SimHei", "Microsoft YaHei"],
        "Darwin": ["PingFang SC", "Heiti TC"],
        "Linux": ["WenQuanYi Micro Hei", "DejaVu Sans"]
    }
    plt.rcParams["font.sans-serif"] = font_configs.get(system, ["DejaVu Sans"])
    plt.rcParams["axes.unicode_minus"] = False

setup_matplotlib_font()

# ---------------------------
# 蜂鸣提示
# ---------------------------
def beep_once():
    try:
        import winsound
        winsound.Beep(1200, 120)
    except Exception:
        try:
            QtWidgets.QApplication.beep()
        except Exception:
            pass

# ---------------------------
# 串口数据读取线程
# ---------------------------
class SerialWorker(QThread):
    value = Signal(float, str)
    status = Signal(str)
    err = Signal(str)

    def __init__(self, port: str, baud: int, reconnect: bool = True, reconnect_sec: float = 1.0):
        super().__init__()
        self.port = port
        self.baud = baud
        self.reconnect = reconnect
        self.reconnect_sec = reconnect_sec
        self._ser: Optional[serial.Serial] = None

    def run(self):
        while not self.isInterruptionRequested():
            if self._ser is None or not getattr(self._ser, "is_open", False):
                ok = self._open_serial()
                if not ok:
                    if not self.reconnect:
                        return
                    for _ in range(int(self.reconnect_sec * 10)):
                        if self.isInterruptionRequested():
                            self._safe_close()
                            return
                        self.msleep(100)
                    continue

            try:
                ser = self._ser
                if ser is None or (hasattr(ser, "is_open") and not ser.is_open):
                    continue

                chunk = ser.readline()
                if not chunk:
                    continue

                s = chunk.decode(errors="ignore").strip()
                v = self._to_number(s)
                if v is not None:
                    self.value.emit(v, s)

            except Exception as e:
                self.status.emit("串口断开，准备重连…" if self.reconnect else "串口异常，已停止")
                self._safe_close()
                if not self.reconnect:
                    self.err.emit(f"串口异常: {e}")
                    return

        self._safe_close()

    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
            self.status.emit(f"已连接 {self.port} @ {self.baud}")
            return True
        except Exception as e:
            self.status.emit(f"打开失败: {e}")
            return False

    def _safe_close(self):
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    @staticmethod
    def _to_number(s: str) -> Optional[float]:
        ss = s.strip()
        m = re.search(r"0x([0-9a-fA-F]+)", ss)
        if m:
            try:
                return float(int(m.group(1), 16))
            except Exception:
                return None
        m = re.search(r"HEX\s*\(\s*(0x[0-9a-fA-F]+|[0-9a-fA-F]+)\s*\)", ss, flags=re.IGNORECASE)
        if m:
            token = m.group(1)
            try:
                if token.lower().startswith("0x"):
                    return float(int(token, 16))
                if re.fullmatch(r"\d+", token):
                    return float(int(token, 10))
                return float(int(token, 16))
            except Exception:
                return None
        m = re.search(r"(-?\d+(?:\.\d+)?)\s*([a-zA-Z°℃]+)?", ss)
        if not m:
            return None
        try:
            val = float(m.group(1))
        except Exception:
            return None
        return val

# ---------------------------
# Matplotlib绘图画布
# ---------------------------
class NeonCanvas(FigureCanvas):
    def __init__(self):
        fig = Figure(figsize=(6, 4), dpi=100, facecolor="black")
        super().__init__(fig)

        self.ax = fig.add_subplot(111, facecolor="black")
        self.ax.tick_params(colors="cyan")
        for spine in self.ax.spines.values():
            spine.set_color("cyan")
        self.ax.grid(True, color="#244061", alpha=0.3)

        self.line, = self.ax.plot([], [], lw=2, color="cyan")
        self.th_line = self.ax.axhline(np.nan, lw=1.5, color="#66ff66", alpha=0.9)
        self.t_peak_dot, = self.ax.plot([], [], 'o', color="#ffff00", markersize=8, label='T line')  # T在前
        self.c_peak_dot, = self.ax.plot([], [], 'o', color="#00ff00", markersize=8, label='C line')  # C在后

        self.ax.legend(
            loc='upper right',
            facecolor='black',
            labelcolor='white',
            framealpha=0.8,
            prop={'family': get_monospace_font(), 'size': 10}
        )
        fig.tight_layout()

        self._hist_lines: Dict[int, Any] = {}
        self._live_x = np.array([], dtype=float)
        self._live_y = np.array([], dtype=float)

    def _recompute_limits(self):
        xs: list[np.ndarray] = []
        ys: list[np.ndarray] = []

        if self._live_x.size > 0 and self._live_y.size > 0:
            xs.append(self._live_x)
            ys.append(self._live_y)

        for ln in self._hist_lines.values():
            try:
                if not ln.get_visible():
                    continue
                x = np.asarray(ln.get_xdata(), dtype=float)
                y = np.asarray(ln.get_ydata(), dtype=float)
                if x.size > 0 and y.size > 0:
                    xs.append(x)
                    ys.append(y)
            except Exception:
                continue

        if not xs or not ys:
            return

        x_all = np.concatenate(xs)
        y_all = np.concatenate(ys)

        self.ax.set_xlim(float(np.min(x_all)), float(np.max(x_all)))
        y_min, y_max = float(np.min(y_all)), float(np.max(y_all))
        pad = 1.0 if y_max == y_min else (y_max - y_min) * 0.08
        self.ax.set_ylim(y_min - pad, y_max + pad)

    def set_threshold(self, th: Optional[float]):
        if th is None:
            self.th_line.set_ydata([np.nan, np.nan])
        else:
            self.th_line.set_ydata([th, th])

    def set_alarm_color(self, alarming: bool):
        self.line.set_color("red" if alarming else "cyan")

    def set_peak_markers(self, t_peak: tuple[float, float] | None = None, c_peak: tuple[float, float] | None = None):
        if t_peak:
            self.t_peak_dot.set_data([t_peak[0]], [t_peak[1]])
        else:
            self.t_peak_dot.set_data([], [])

        if c_peak:
            self.c_peak_dot.set_data([c_peak[0]], [c_peak[1]])
        else:
            self.c_peak_dot.set_data([], [])

    def set_data(self, x: np.ndarray, y: np.ndarray):
        self._live_x = np.asarray(x, dtype=float)
        self._live_y = np.asarray(y, dtype=float)
        self.line.set_data(x, y)
        if x.size > 1:
            self._recompute_limits()
        self.draw_idle()

    def add_history(self, run_id: int, x: np.ndarray, y: np.ndarray, alpha: float = 0.65):
        if run_id in self._hist_lines:
            ln = self._hist_lines[run_id]
            ln.set_data(x, y)
            ln.set_visible(True)
        else:
            ln, = self.ax.plot(x, y, lw=1.5, alpha=alpha, label="_nolegend_")
            self._hist_lines[run_id] = ln
        self._recompute_limits()
        self.draw_idle()

    def set_history_visible(self, run_id: int, visible: bool):
        ln = self._hist_lines.get(run_id)
        if ln is None:
            return
        ln.set_visible(visible)
        self._recompute_limits()
        self.draw_idle()

    def remove_history(self, run_id: int):
        ln = self._hist_lines.pop(run_id, None)
        if ln is None:
            return
        try:
            ln.remove()
        except Exception:
            pass
        self._recompute_limits()
        self.draw_idle()

    def clear_history(self):
        for rid in list(self._hist_lines.keys()):
            self.remove_history(rid)

# ---------------------------
# T/C峰值识别算法（交换顺序）
# ---------------------------
Peak = Tuple[float, float]  # (x, y)

@dataclass
class RunRecord:
    run_id: int
    ts: float
    label: str
    x_raw: np.ndarray
    y_raw: np.ndarray
    x_plot: np.ndarray
    y_plot: np.ndarray
    t_peak: Optional[Peak] = None  # T在前
    c_peak: Optional[Peak] = None  # C在后
    tc_ratio: Optional[float] = None
    meta: Dict[str, Any] = None

def detect_tc_peaks(  # 改名为detect_tc_peaks
    y_data: np.ndarray, x_data: np.ndarray,
    min_peak_height: float = 5000, min_peak_distance: int = 100,
    t_x_min: float = 0.0, t_x_max: float = 10000.0,
    c_x_min: float = 0.0, c_x_max: float = 10000.0
) -> Tuple[Optional[Peak], Optional[Peak]]:
    if y_data.shape[0] != x_data.shape[0]:
        raise ValueError("x_data and y_data must have the same length")
    if y_data.shape[0] < 10:
        return None, None

    smoothed_y: np.ndarray = y_data.astype(float, copy=False)
    if y_data.shape[0] >= 7:
        kernel = np.ones(7, dtype=float) / 7.0
        smoothed_y = np.convolve(smoothed_y, kernel, mode="same")

    peak_indices = argrelextrema(smoothed_y, np.greater, order=5)[0]
    if peak_indices.size < 2:
        return None, None

    peaks: List[Peak] = [
        (float(x_data[i]), float(smoothed_y[i]))
        for i in peak_indices
        if float(smoothed_y[i]) >= float(min_peak_height)
    ]
    if len(peaks) < 2:
        return None, None

    peaks_sorted = sorted(peaks, key=lambda p: p[0])
    t_peak: Optional[Peak] = None
    c_peak: Optional[Peak] = None

    t_candidates = [p for p in peaks_sorted if t_x_min <= p[0] <= t_x_max]
    c_candidates = [p for p in peaks_sorted if c_x_min <= p[0] <= c_x_max]

    if t_candidates and c_candidates:
        t_peak = max(t_candidates, key=lambda p: p[1])
        c_peak = max(c_candidates, key=lambda p: p[1])
        if t_peak[0] >= c_peak[0]:
            c_candidates_after_t = [p for p in c_candidates if p[0] > t_peak[0]]
            if c_candidates_after_t:
                c_peak = max(c_candidates_after_t, key=lambda p: p[1])
            else:
                t_peak = None
                c_peak = None
    else:
        for i in range(len(peaks_sorted) - 1):
            p1 = peaks_sorted[i]
            p2 = peaks_sorted[i + 1]
            if abs(p2[0] - p1[0]) >= min_peak_distance:
                t_peak = p1
                c_peak = p2
                break

    return t_peak, c_peak  # 返回T在前，C在后

# ---------------------------
# 主窗口（GUI）
# ---------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("荧光显示仪（T/C峰值版 + 浓度控制）")
        self.resize(1400, 700)

        # 浓度控制相关变量
        self._running = False
        self.run_concentration: Optional[float] = None  # 本次测量锁定的浓度

        # 数据存储
        self.N = 5000
        self.y: Deque[float] = deque(maxlen=self.N)
        self.t: Deque[float] = deque(maxlen=self.N)
        self.raw_last: str = ""

        # 串口线程
        self.worker: Optional[SerialWorker] = None

        # 报警相关
        self.alarming = False
        self._blink_state = False

        # 自动保存
        self.auto_dir = ""
        self._current_bucket = ""
        self._auto_fp = None

        # 峰值参数
        self.min_peak_height = 5000
        self.min_peak_distance = 100
        self.t_peak_value = None  # T在前
        self.c_peak_value = None  # C在后
        self.ratio_tc = None
        self.t_x_min = 200  # 交换范围
        self.t_x_max = 400
        self.c_x_min = 400
        self.c_x_max = 800

        # UI加载
        from PySide6.QtUiTools import QUiLoader
        base = "D:\jiahui"
        ui_path = os.path.join(base, "fluor_range_form_qt5.ui")
        if not os.path.exists(ui_path):
            ui_path = os.path.join(base, "fluor_range_form.ui")
        ui_file = QtCore.QFile(ui_path)
        if not ui_file.open(QtCore.QFile.ReadOnly):
            raise RuntimeError(f"无法打开UI文件: {ui_path}")

        loader = QUiLoader()
        ui = loader.load(ui_file, self)
        ui_file.close()

        if ui is None:
            raise RuntimeError(f"UI加载失败: {ui_path}")

        self.setCentralWidget(ui)

        T = TypeVar("T", bound=QtCore.QObject)
        def must(cls: Type[T], name: str) -> T:
            w = ui.findChild(cls, name)
            if w is None:
                raise RuntimeError(f"UI里找不到: {name}")
            return cast(T, w)

        self.portCB = must(QtWidgets.QComboBox, "portCB")
        self.refreshBtn = must(QtWidgets.QPushButton, "refreshBtn")
        self.baudCB = must(QtWidgets.QComboBox, "baudCB")
        self.reconnectChk = must(QtWidgets.QCheckBox, "reconnectChk")
        self.fpsCB = must(QtWidgets.QComboBox, "fpsCB")
        self.smoothChk = must(QtWidgets.QCheckBox, "smoothChk")
        self.startBtn = must(QtWidgets.QPushButton, "startBtn")
        self.stopBtn = must(QtWidgets.QPushButton, "stopBtn")
        self.clearBtn = must(QtWidgets.QPushButton, "clearBtn")
        self.alarmEnableChk = must(QtWidgets.QCheckBox, "alarmEnableChk")
        self.thSpin = must(QtWidgets.QDoubleSpinBox, "thSpin")
        self.beepChk = must(QtWidgets.QCheckBox, "beepChk")
        self.exportBtn = must(QtWidgets.QPushButton, "exportBtn")
        self.autoSaveChk = must(QtWidgets.QCheckBox, "autoSaveChk")
        self.pickDirBtn = must(QtWidgets.QPushButton, "pickDirBtn")
        self.dirLab = must(QtWidgets.QLabel, "dirLab")
        self.peakHeightSpin = must(QtWidgets.QDoubleSpinBox, "peakHeightSpin")
        self.peakDistanceSpin = must(QtWidgets.QSpinBox, "peakDistanceSpin")
        self.tMinSpin = must(QtWidgets.QDoubleSpinBox, "cMinSpin")  # 交换名称
        self.tMaxSpin = must(QtWidgets.QDoubleSpinBox, "cMaxSpin")
        self.cMinSpin = must(QtWidgets.QDoubleSpinBox, "tMinSpin")
        self.cMaxSpin = must(QtWidgets.QDoubleSpinBox, "tMaxSpin")
        self.textBox = must(QtWidgets.QPlainTextEdit, "textBox")
        self.historyList = must(QtWidgets.QListWidget, "historyList")
        self.saveRunBtn = must(QtWidgets.QPushButton, "saveRunBtn")
        self.removeRunBtn = must(QtWidgets.QPushButton, "removeRunBtn")
        self.clearHistoryBtn = must(QtWidgets.QPushButton, "clearHistoryBtn")
        self.latestLab = must(QtWidgets.QLabel, "latestLab")
        self.rawLab = must(QtWidgets.QLabel, "rawLab")
        self.tPeakLab = must(QtWidgets.QLabel, "cPeakLab")  # 交换标签
        self.cPeakLab = must(QtWidgets.QLabel, "tPeakLab")
        self.ratioLab = must(QtWidgets.QLabel, "ratioLab")
        plotLayout = must(QtWidgets.QVBoxLayout, "plotLayout")

        self.canvas = NeonCanvas()
        plotLayout.addWidget(self.canvas)

        self.max_history = 12
        self._run_seq = 0
        self._history: Dict[int, RunRecord] = {}
        self._history_order: List[int] = []

        self.textBox.setFont(QtGui.QFont(get_monospace_font(), 11))
        for lab in (self.latestLab, self.tPeakLab, self.cPeakLab, self.ratioLab):
            lab.setFont(QtGui.QFont(get_monospace_font(), 16))

        self.refreshBtn.clicked.connect(self.refresh_ports)
        self.statusBar().showMessage("未连接")

        # 动态添加浓度输入框到状态栏
        self.concLab = QtWidgets.QLabel("当前浓度:")
        self.concEdit = QtWidgets.QLineEdit()
        self.concEdit.setFixedWidth(140)
        self.concEdit.setPlaceholderText("必填(数字)")
        self.concEdit.setValidator(QtGui.QDoubleValidator(0.0, 1e18, 8, self))

        self.statusBar().addPermanentWidget(self.concLab)
        self.statusBar().addPermanentWidget(self.concEdit)

        # 信号绑定
        self.startBtn.clicked.connect(self.start_read)
        self.stopBtn.clicked.connect(self.stop_read)
        self.clearBtn.clicked.connect(self.clear_all)
        self.concEdit.textChanged.connect(self._update_start_enable)

        self.saveRunBtn.clicked.connect(lambda: self.save_current_to_history(manual=True))
        self.removeRunBtn.clicked.connect(self.remove_selected_history)
        self.clearHistoryBtn.clicked.connect(self.clear_history)
        self.historyList.itemChanged.connect(self.on_history_item_changed)
        self.exportBtn.clicked.connect(self.export_csv)
        self.pickDirBtn.clicked.connect(self.pick_dir)
        self.thSpin.valueChanged.connect(self.on_threshold_changed)
        self.alarmEnableChk.toggled.connect(self.on_threshold_changed)
        self.fpsCB.currentTextChanged.connect(self.apply_fps)
        self.peakHeightSpin.valueChanged.connect(self.update_peak_params)
        self.peakDistanceSpin.valueChanged.connect(self.update_peak_params)
        self.tMinSpin.valueChanged.connect(self.update_tc_ranges)
        self.tMaxSpin.valueChanged.connect(self.update_tc_ranges)
        self.cMinSpin.valueChanged.connect(self.update_tc_ranges)
        self.cMaxSpin.valueChanged.connect(self.update_tc_ranges)

        self.timer = QtCore.QTimer(self)
        self.apply_fps(self.fpsCB.currentText())
        self.timer.timeout.connect(self.redraw)

        self.blinkTimer = QtCore.QTimer(self)
        self.blinkTimer.setInterval(250)
        self.blinkTimer.timeout.connect(self._blink_tick)

        self.autoTimer = QtCore.QTimer(self)
        self.autoTimer.setInterval(1000)
        self.autoTimer.timeout.connect(self._auto_save_tick)

        self.refresh_ports()
        self.on_threshold_changed()
        self.update_peak_params()
        self.update_tc_ranges()

        self._set_running(False)

    def _parse_concentration(self) -> Optional[float]:
        s = self.concEdit.text().strip()
        if not s:
            return None
        try:
            return float(s)
        except Exception:
            return None

    def _update_start_enable(self):
        conc_ok = self._parse_concentration() is not None
        can_start = (not self._running) and conc_ok
        self.startBtn.setEnabled(can_start)
        if not conc_ok:
            self.concEdit.setStyleSheet("background-color: #fff0f0;")
        else:
            self.concEdit.setStyleSheet("background-color: #f0fff0;")

    def update_peak_params(self):
        self.min_peak_height = self.peakHeightSpin.value()
        self.min_peak_distance = self.peakDistanceSpin.value()

    def update_tc_ranges(self):  # 改名为update_tc_ranges
        self.t_x_min = self.tMinSpin.value()
        self.t_x_max = self.tMaxSpin.value()
        self.c_x_min = self.cMinSpin.value()
        self.c_x_max = self.cMaxSpin.value()

    def _set_running(self, running: bool):
        self._running = running
        self.stopBtn.setEnabled(running)
        self._update_start_enable()

    def _append_log(self, s: str):
        self.textBox.appendPlainText(s)

    def refresh_ports(self):
        self.portCB.blockSignals(True)
        cur = self.portCB.currentText().strip()
        self.portCB.clear()
        ports = [p.device for p in list_ports.comports()]
        if ports:
            self.portCB.addItems(ports)
            if cur in ports:
                self.portCB.setCurrentText(cur)
        else:
            self.portCB.addItem("手动输入串口号")
            self.portCB.setEditable(True)
        self.portCB.blockSignals(False)

    def on_threshold_changed(self):
        if self.alarmEnableChk.isChecked():
            th = float(self.thSpin.value())
            self.canvas.set_threshold(th)
        else:
            self.canvas.set_threshold(None)
        self.canvas.draw_idle()

    def _set_alarm(self, on: bool):
        if on == self.alarming:
            return
        self.alarming = on
        self.canvas.set_alarm_color(on)
        if on:
            if not self.blinkTimer.isActive():
                self.blinkTimer.start()
        else:
            self.blinkTimer.stop()
            self._blink_state = False
            self.latestLab.setStyleSheet("")
            self.statusBar().showMessage("正常")

    def _blink_tick(self):
        if not self.alarming:
            return
        self._blink_state = not self._blink_state
        if self._blink_state:
            self.latestLab.setStyleSheet("background:#aa0000; color:white; padding:2px 6px;")
            if self.beepChk.isChecked():
                beep_once()
        else:
            self.latestLab.setStyleSheet("")

    def apply_fps(self, fps_text: str):
        try:
            fps = int(fps_text)
            fps = max(1, min(60, fps))
        except Exception:
            fps = 20
        self.timer.setInterval(int(1000 / fps))

    def pick_dir(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "选择自动保存目录", self.auto_dir or os.getcwd())
        if d:
            self.auto_dir = d
            self.dirLab.setText(d)
            self.dirLab.setStyleSheet("color:#0a0;")

    def _auto_open_for_bucket(self, bucket: str):
        try:
            if self._auto_fp:
                self._auto_fp.close()
        except Exception:
            pass
        self._auto_fp = None
        if not self.auto_dir:
            return
        os.makedirs(self.auto_dir, exist_ok=True)
        path = os.path.join(self.auto_dir, f"data_{bucket}.csv")
        newfile = not os.path.exists(path)
        self._auto_fp = open(path, "a", newline="", encoding="utf-8")
        self._auto_writer = csv.writer(self._auto_fp)
        if newfile:
            self._auto_writer.writerow(["timestamp_iso", "unix_time", "value", "raw"])
        self._current_bucket = bucket
        self.statusBar().showMessage(f"自动保存: {os.path.basename(path)}")

    def _auto_save_tick(self):
        if not self.autoSaveChk.isChecked():
            if self._auto_fp:
                try:
                    self._auto_fp.close()
                except Exception:
                    pass
            self._auto_fp = None
            self._current_bucket = ""
            return
        if not self.auto_dir:
            return
        now = datetime.now()
        bucket = now.strftime("%Y%m%d_%H%M")
        if bucket != self._current_bucket or self._auto_fp is None:
            self._auto_open_for_bucket(bucket)

    def start_read(self):
        port = self.portCB.currentText().strip()
        try:
            baud = int(self.baudCB.currentText())
        except ValueError:
            baud = 115200

        conc = self._parse_concentration()
        if conc is None:
            QtWidgets.QMessageBox.information(self, "提示", "请先在右下角输入“当前浓度”(数字)，才能开始测量。")
            self._update_start_enable()
            return

        self.run_concentration = conc

        self.y.clear()
        self.t.clear()
        self.raw_last = ""
        self.textBox.clear()
        self.canvas.set_data(np.array([]), np.array([]))
        self.latestLab.setText("—")
        self.rawLab.setText("")
        self.tPeakLab.setText("—")
        self.cPeakLab.setText("—")
        self.ratioLab.setText("—")
        self.t_peak_value = None
        self.c_peak_value = None
        self.ratio_tc = None
        self._set_alarm(False)

        self._append_log(f"[{time.strftime('%H:%M:%S')}] 已连接 {port}，当前浓度设定: {conc}")
        self.statusBar().showMessage(f"连接中: {port} (浓度={conc})")

        self.worker = SerialWorker(port, baud, reconnect=self.reconnectChk.isChecked(), reconnect_sec=1.0)
        self.worker.value.connect(self.on_value)
        self.worker.status.connect(self.on_status)
        self.worker.err.connect(self.on_err)
        self.worker.start()

        self.timer.start()
        self.autoTimer.start()
        self._set_running(True)

    def stop_read(self):
        self.timer.stop()
        self.autoTimer.stop()
        self._set_alarm(False)
        if self.worker and self.worker.isRunning():
            self.worker.requestInterruption()
            self.worker.wait(800)
        self.worker = None
        self.save_current_to_history(manual=False)
        try:
            if self._auto_fp:
                self._auto_fp.close()
        except Exception:
            pass
        self._auto_fp = None
        self._current_bucket = ""
        self.statusBar().showMessage("已停止")
        self._set_running(False)

    def clear_all(self):
        self.y.clear()
        self.t.clear()
        self.raw_last = ""
        self.canvas.set_data(np.array([]), np.array([]))
        self.canvas.set_peak_markers(None, None)
        self.canvas.draw_idle()
        self.latestLab.setText("—")
        self.rawLab.setText("")
        self.tPeakLab.setText("—")
        self.cPeakLab.setText("—")
        self.ratioLab.setText("—")
        self.t_peak_value = None
        self.c_peak_value = None
        self.ratio_tc = None
        self.textBox.clear()
        self._set_alarm(False)

        self.run_concentration = None
        self._update_start_enable()

        self.statusBar().showMessage("已清空")

    def _make_plot_series(self, y_raw: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        x = np.arange(y_raw.size)
        max_plot = 2000
        if y_raw.size > max_plot:
            stride = int(np.ceil(y_raw.size / max_plot))
            y_plot = y_raw[::stride]
            x_plot = x[::stride]
        else:
            y_plot = y_raw
            x_plot = x
        if self.smoothChk.isChecked() and y_plot.size >= 7:
            k = 7
            kernel = np.ones(k) / k
            y_plot = np.convolve(y_plot, kernel, mode="same")
        return x_plot.astype(float), y_plot.astype(float)

    def save_current_to_history(self, manual: bool = True):
        if len(self.y) < 10:
            if manual:
                QtWidgets.QMessageBox.information(self, "历史曲线", "当前数据量太少，无法保存到历史。")
            return

        y_raw = np.fromiter(self.y, dtype=float)
        x_raw = np.arange(y_raw.size, dtype=float)
        x_plot, y_plot = self._make_plot_series(y_raw)
        t_peak, c_peak = detect_tc_peaks(
            y_raw, x_raw,
            min_peak_height=self.min_peak_height,
            min_peak_distance=self.min_peak_distance,
            t_x_min=self.t_x_min, t_x_max=self.t_x_max,
            c_x_min=self.c_x_min, c_x_max=self.c_x_max
        )
        tc_ratio = None
        if t_peak and c_peak and c_peak[1] > 0:
            tc_ratio = float(t_peak[1] / c_peak[1])

        self._run_seq += 1
        run_id = self._run_seq
        ts = time.time()

        # 修改label格式，添加浓度信息
        conc = self.run_concentration if self.run_concentration is not None else 0
        if tc_ratio is not None:
            label = f"{time.strftime('%H:%M:%S')}  #{run_id:03d}  T/C={tc_ratio:.3f} 浓度={conc:.3f}"
        else:
            label = f"{time.strftime('%H:%M:%S')}  #{run_id:03d} 浓度={conc:.3f}"

        rec = RunRecord(
            run_id=run_id, ts=ts, label=label,
            x_raw=x_raw, y_raw=y_raw, x_plot=x_plot, y_plot=y_plot,
            t_peak=t_peak, c_peak=c_peak, tc_ratio=tc_ratio,
            meta={
                "port": self.portCB.currentText().strip(),
                "concentration": self.run_concentration,
                "smooth": bool(self.smoothChk.isChecked()),
            },
        )

        if len(self._history_order) >= self.max_history:
            old_id = self._history_order.pop(0)
            self._history.pop(old_id, None)
            self.canvas.remove_history(old_id)
            self.historyList.blockSignals(True)
            try:
                self.historyList.takeItem(0)
            finally:
                self.historyList.blockSignals(False)

        self._history[run_id] = rec
        self._history_order.append(run_id)

        item = QtWidgets.QListWidgetItem(label)
        item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)
        item.setCheckState(QtCore.Qt.Checked)
        item.setData(QtCore.Qt.UserRole, run_id)
        tip = []
        if self.run_concentration is not None:
            tip.append(f"浓度: {self.run_concentration:.3f}")
        if t_peak:
            tip.append(f"T峰: x={t_peak[0]:.0f}, y={t_peak[1]:.1f}")
        if c_peak:
            tip.append(f"C峰: x={c_peak[0]:.0f}, y={c_peak[1]:.1f}")
        if tc_ratio is not None:
            tip.append(f"T/C: {tc_ratio:.3f}")
        item.setToolTip("\n".join(tip))

        self.historyList.blockSignals(True)
        try:
            self.historyList.addItem(item)
        finally:
            self.historyList.blockSignals(False)
        self.canvas.add_history(run_id, x_plot, y_plot)
        self.statusBar().showMessage(f"已保存到历史: {label}")

    @QtCore.Slot(QtWidgets.QListWidgetItem)
    def on_history_item_changed(self, item: QtWidgets.QListWidgetItem):
        rid = item.data(QtCore.Qt.UserRole)
        if rid is None:
            return
        try:
            run_id = int(rid)
        except Exception:
            return
        visible = item.checkState() == QtCore.Qt.Checked
        self.canvas.set_history_visible(run_id, visible)

    def remove_selected_history(self):
        items = self.historyList.selectedItems()
        if not items:
            return
        self.historyList.blockSignals(True)
        try:
            for item in items:
                rid = item.data(QtCore.Qt.UserRole)
                if rid is None:
                    continue
                try:
                    run_id = int(rid)
                except Exception:
                    continue
                self.canvas.remove_history(run_id)
                self._history.pop(run_id, None)
                try:
                    self._history_order.remove(run_id)
                except ValueError:
                    pass
                row = self.historyList.row(item)
                self.historyList.takeItem(row)
        finally:
            self.historyList.blockSignals(False)
        self.statusBar().showMessage("已删除所选历史曲线")

    def clear_history(self):
        self.historyList.blockSignals(True)
        try:
            self.historyList.clear()
        finally:
            self.historyList.blockSignals(False)
        self._history.clear()
        self._history_order.clear()
        self.canvas.clear_history()
        self.statusBar().showMessage("已清空历史曲线")

    @QtCore.Slot(float, str)
    def on_value(self, v: float, raw: str):
        now = time.time()
        self.y.append(v)
        self.t.append(now)
        self.raw_last = raw
        self.latestLab.setText(f"{v:.1f}")
        self.rawLab.setText(f"raw: {raw}")
        self._append_log(f"{v:.1f}")
        if self.alarmEnableChk.isChecked():
            th = float(self.thSpin.value())
            self._set_alarm(v > th)
        else:
            self._set_alarm(False)
        if self.autoSaveChk.isChecked() and self._auto_fp:
            ts_iso = datetime.fromtimestamp(now).isoformat(timespec="milliseconds")
            try:
                self._auto_writer.writerow([ts_iso, f"{now:.3f}", v, raw])
                self._auto_fp.flush()
            except Exception:
                pass

    @QtCore.Slot(str)
    def on_status(self, s: str):
        self.statusBar().showMessage(s)
        self._append_log(f"[{time.strftime('%H:%M:%S')}] {s}")

    @QtCore.Slot(str)
    def on_err(self, e: str):
        QtWidgets.QMessageBox.warning(self, "串口错误", e)
        self.stop_read()

    @QtCore.Slot()
    def redraw(self):
        if not self.y:
            return
        y = np.fromiter(self.y, dtype=float)
        x = np.arange(y.size)
        max_plot = 2000
        if y.size > max_plot:
            stride = int(np.ceil(y.size / max_plot))
            y_plot = y[::stride]
            x_plot = x[::stride]
        else:
            y_plot = y
            x_plot = x
        if self.smoothChk.isChecked() and y_plot.size >= 5:
            k = 5
            kernel = np.ones(k) / k
            y_plot = np.convolve(y_plot, kernel, mode="same")
        t_peak, c_peak = detect_tc_peaks(
            y, x,
            min_peak_height=self.min_peak_height,
            min_peak_distance=self.min_peak_distance,
            t_x_min=self.t_x_min, t_x_max=self.t_x_max,
            c_x_min=self.c_x_min, c_x_max=self.c_x_max
        )
        if t_peak:
            self.t_peak_value = t_peak[1]
            self.tPeakLab.setText(f"{self.t_peak_value:.1f}")
        else:
            self.t_peak_value = None
            self.tPeakLab.setText("—")
        if c_peak:
            self.c_peak_value = c_peak[1]
            self.cPeakLab.setText(f"{self.c_peak_value:.1f}")
        else:
            self.c_peak_value = None
            self.cPeakLab.setText("—")
        if self.t_peak_value and self.c_peak_value and self.c_peak_value > 0:
            self.ratio_tc = self.t_peak_value / self.c_peak_value
            self.ratioLab.setText(f"{self.ratio_tc:.3f}")
        else:
            self.ratio_tc = None
            self.ratioLab.setText("—")
        self.canvas.set_data(x_plot, y_plot)
        self.canvas.set_peak_markers(t_peak, c_peak)
        self.canvas.draw_idle()

    def export_csv(self):
        if not self.y:
            QtWidgets.QMessageBox.information(self, "导出", "当前没有数据可导出。")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "导出为 CSV",
            f"data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv)"
        )
        if not path:
            return

        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)

                # 写入浓度信息
                conc = self.run_concentration
                if conc is None:
                    conc = self._parse_concentration()

                w.writerow(["concentration", conc if conc is not None else "NaN"])
                w.writerow([])

                # 写入表头
                w.writerow(["x_ms", "y"])

                # 写入数据
                if len(self.t) > 0:
                    start_time = self.t[0]
                    for ts, val in zip(self.t, self.y):
                        relative_time_ms = (ts - start_time) * 1000.0
                        w.writerow([f"{relative_time_ms:.1f}", val])

                # 写入峰值汇总
                w.writerow([])
                w.writerow(["Analysis Result", "Value"])
                w.writerow(["浓度", conc if conc is not None else "NaN"])
                w.writerow(["T peak", self.t_peak_value if self.t_peak_value else "NaN"])
                w.writerow(["C peak", self.c_peak_value if self.c_peak_value else "NaN"])
                w.writerow(["T/C ratio", self.ratio_tc if self.ratio_tc else "NaN"])

            QtWidgets.QMessageBox.information(self, "导出", "数据导出完成。")
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "导出失败", str(e))

    def closeEvent(self, e: QtGui.QCloseEvent):
        self.stop_read()
        super().closeEvent(e)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())