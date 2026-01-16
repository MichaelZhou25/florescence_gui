# -*- coding: utf-8 -*-
"""
荧光显示仪 · 试纸条C/T峰值识别版
功能：
1. 串口读取荧光强度数据并绘制实时曲线
2. 自动识别C/T双峰值（抗噪声、筛选相隔较远峰值）
3. 显示C峰值、T峰值及T/C比值
4. 阈值报警、数据导出/自动保存
5. 跨平台兼容（Windows/macOS/Linux），修复字体警告和中文显示问题
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
# 跨平台字体配置（核心修复）
# ---------------------------
def get_monospace_font():
    """获取跨平台等宽字体，消除Consolas缺失警告"""
    system = platform.system()
    if system == "Windows":
        return "Consolas"
    elif system == "Darwin":  # macOS
        return "Menlo"
    else:  # Linux
        return "Monospace"


def setup_matplotlib_font():
    """配置Matplotlib支持中文，解决中文字符缺失警告"""
    import matplotlib.pyplot as plt
    system = platform.system()
    # 适配不同系统的中文字体
    font_configs = {
        "Windows": ["SimHei", "Microsoft YaHei"],
        "Darwin": ["PingFang SC", "Heiti TC"],  # macOS
        "Linux": ["WenQuanYi Micro Hei", "DejaVu Sans"]
    }
    plt.rcParams["font.sans-serif"] = font_configs.get(system, ["DejaVu Sans"])
    plt.rcParams["axes.unicode_minus"] = False  # 解决负号显示异常问题


# 初始化Matplotlib字体配置
setup_matplotlib_font()


# ---------------------------
# 蜂鸣提示（跨平台兼容）
# ---------------------------
def beep_once():
    try:
        import winsound  # Windows专属
        winsound.Beep(1200, 120)  # 频率1200Hz，时长120ms
    except Exception:
        try:
            QtWidgets.QApplication.beep()  # Qt跨平台蜂鸣
        except Exception:
            pass


# ---------------------------
# 串口数据读取线程
# ---------------------------
class SerialWorker(QThread):
    value = Signal(float, str)  # 发射(数值, 原始字符串)
    status = Signal(str)  # 发射状态信息
    err = Signal(str)  # 发射错误信息

    def __init__(self, port: str, baud: int, reconnect: bool = True, reconnect_sec: float = 1.0):
        super().__init__()
        self.port = port
        self.baud = baud
        self.reconnect = reconnect
        self.reconnect_sec = reconnect_sec
        self._ser: Optional[serial.Serial] = None

    def run(self):
        """线程主循环：持续读取串口数据"""
        while not self.isInterruptionRequested():
            # 检查串口连接
            if self._ser is None or not getattr(self._ser, "is_open", False):
                ok = self._open_serial()
                if not ok:
                    if not self.reconnect:
                        return
                    # 自动重连等待
                    for _ in range(int(self.reconnect_sec * 10)):
                        if self.isInterruptionRequested():
                            self._safe_close()
                            return
                        self.msleep(100)
                    continue

            # 读取串口数据
            try:
                ser = self._ser
                if ser is None or (hasattr(ser, "is_open") and not ser.is_open):
                    continue

                chunk = ser.readline()
                if not chunk:
                    continue

                # 解码并解析数值
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
        """打开串口"""
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
        """安全关闭串口"""
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    @staticmethod
    def _to_number(s: str) -> Optional[float]:
        """
        通用数值解析：支持十六进制、带单位数值、纯数字
        支持格式：0xABCD、HEX(1234)、123.45mV、-67.8℃等
        """
        ss = s.strip()

        # 解析十六进制 0xABCD 格式
        m = re.search(r"0x([0-9a-fA-F]+)", ss)
        if m:
            try:
                return float(int(m.group(1), 16))
            except Exception:
                return None

        # 解析 HEX(1234) 格式
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

        # 解析数字+单位格式
        m = re.search(r"(-?\d+(?:\.\d+)?)\s*([a-zA-Z°℃]+)?", ss)
        if not m:
            return None
        try:
            val = float(m.group(1))
        except Exception:
            return None

        return val


# ---------------------------
# Matplotlib绘图画布（含峰值标注）
# ---------------------------
class NeonCanvas(FigureCanvas):
    def __init__(self):
        fig = Figure(figsize=(6, 4), dpi=100, facecolor="black")
        super().__init__(fig)

        # 初始化坐标轴
        self.ax = fig.add_subplot(111, facecolor="black")
        self.ax.tick_params(colors="cyan")
        for spine in self.ax.spines.values():
            spine.set_color("cyan")
        self.ax.grid(True, color="#244061", alpha=0.3)

        # 绘图元素：主曲线、阈值线、峰值标记
        self.line, = self.ax.plot([], [], lw=2, color="cyan")  # 实时曲线
        self.th_line = self.ax.axhline(np.nan, lw=1.5, color="#66ff66", alpha=0.9)  # 阈值线
        self.c_peak_dot, = self.ax.plot([], [], 'o', color="#00ff00", markersize=8, label='C line')  # C峰标记
        self.t_peak_dot, = self.ax.plot([], [], 'o', color="#ffff00", markersize=8, label='T line')  # T峰标记

        # 图例（指定跨平台字体）
        self.ax.legend(
            loc='upper right',
            facecolor='black',
            labelcolor='white',
            framealpha=0.8,
            prop={'family': get_monospace_font(), 'size': 10}
        )
        fig.tight_layout()

        # 历史曲线（run_id -> Line2D）
        self._hist_lines: Dict[int, Any] = {}
        self._live_x = np.array([], dtype=float)
        self._live_y = np.array([], dtype=float)

    # ---------------------------
    # 轴范围：同时考虑实时曲线 + 可见历史曲线
    # ---------------------------
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

        # x
        self.ax.set_xlim(float(np.min(x_all)), float(np.max(x_all)))

        # y with padding
        y_min, y_max = float(np.min(y_all)), float(np.max(y_all))
        pad = 1.0 if y_max == y_min else (y_max - y_min) * 0.08
        self.ax.set_ylim(y_min - pad, y_max + pad)

    def set_threshold(self, th: Optional[float]):
        """设置阈值线"""
        if th is None:
            self.th_line.set_ydata([np.nan, np.nan])
        else:
            self.th_line.set_ydata([th, th])

    def set_alarm_color(self, alarming: bool):
        """设置曲线报警颜色"""
        self.line.set_color("red" if alarming else "cyan")

    def set_peak_markers(
    self,
    c_peak: tuple[float, float] | None = None,
    t_peak: tuple[float, float] | None = None
):
    
        if c_peak:
            self.c_peak_dot.set_data([c_peak[0]], [c_peak[1]])
        else:
            self.c_peak_dot.set_data([], [])

        if t_peak:
            self.t_peak_dot.set_data([t_peak[0]], [t_peak[1]])
        else:
            self.t_peak_dot.set_data([], [])

    def set_data(self, x: np.ndarray, y: np.ndarray):
        """更新曲线数据"""
        self._live_x = np.asarray(x, dtype=float)
        self._live_y = np.asarray(y, dtype=float)
        self.line.set_data(x, y)
        if x.size > 1:
            self._recompute_limits()
        self.draw_idle()

    # ---------------------------
    # 历史曲线管理
    # ---------------------------
    def add_history(self, run_id: int, x: np.ndarray, y: np.ndarray, alpha: float = 0.65):
        """添加一条历史曲线（默认加入但不进入图例）"""
        if run_id in self._hist_lines:
            ln = self._hist_lines[run_id]
            ln.set_data(x, y)
            ln.set_visible(True)
        else:
            # label 用 _nolegend_ 避免撑爆图例
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
# C/T峰值识别算法
# ---------------------------

Peak = Tuple[float, float]  # (x, y)


@dataclass
class RunRecord:
    """一次测量（或一次快照）的数据记录"""
    run_id: int
    ts: float
    label: str
    x_raw: np.ndarray
    y_raw: np.ndarray
    x_plot: np.ndarray
    y_plot: np.ndarray
    c_peak: Optional[Peak] = None
    t_peak: Optional[Peak] = None
    tc_ratio: Optional[float] = None
    meta: Dict[str, Any] = None


def detect_ct_peaks(
    y_data: np.ndarray,
    x_data: np.ndarray,
    min_peak_height: float = 5000,
    min_peak_distance: int = 100,
    c_x_min: float = 0.0,
    c_x_max: float = 10000.0,
    t_x_min: float = 0.0,
    t_x_max: float = 10000.0
) -> Tuple[Optional[Peak], Optional[Peak]]:
    """
    识别试纸条C/T双峰值（C在前，T在后）
    :param y_data: 荧光强度数值数组
    :param x_data: 横坐标数组
    :param min_peak_height: 最小峰值高度（过滤噪声峰）
    :param min_peak_distance: 峰值最小间隔（确保C/T峰相隔较远）
    :param c_x_min: C线最小横坐标
    :param c_x_max: C线最大横坐标
    :param t_x_min: T线最小横坐标
    :param t_x_max: T线最大横坐标
    :return: (C峰(x,y), T峰(x,y))，无符合条件峰值返回(None, None)
    """
    # 维度检查
    if y_data.shape[0] != x_data.shape[0]:
        raise ValueError("x_data and y_data must have the same length")

    # 数据量不足时不识别
    if y_data.shape[0] < 10:
        return None, None

    # 步骤1：滑动平均平滑去噪（减少噪声干扰）
    smoothed_y: np.ndarray = y_data.astype(float, copy=False)
    if y_data.shape[0] >= 7:
        kernel = np.ones(7, dtype=float) / 7.0
        smoothed_y = np.convolve(smoothed_y, kernel, mode="same")

    # 步骤2：检测局部极大值（当前点比前后5个点都大）
    peak_indices = argrelextrema(smoothed_y, np.greater, order=5)[0]
    if peak_indices.size < 2:
        return None, None

    # 步骤3：筛选高度达标的峰值（强制转成 Python float，避免 numpy 标量造成类型告警）
    peaks: List[Peak] = [
        (float(x_data[i]), float(smoothed_y[i]))
        for i in peak_indices
        if float(smoothed_y[i]) >= float(min_peak_height)
    ]
    if len(peaks) < 2:
        return None, None

    # 步骤4：按横坐标排序，筛选相隔足够远的两个峰值（C在前，T在后）
    peaks_sorted = sorted(peaks, key=lambda p: p[0])

    c_peak: Optional[Peak] = None
    t_peak: Optional[Peak] = None

    # 先在C线范围内找峰值
    c_candidates: List[Peak] = [p for p in peaks_sorted if c_x_min <= p[0] <= c_x_max]
    # 在T线范围内找峰值
    t_candidates: List[Peak] = [p for p in peaks_sorted if t_x_min <= p[0] <= t_x_max]

    # 如果在指定范围内找到合适的C和T峰值
    if c_candidates and t_candidates:
        c_peak = max(c_candidates, key=lambda p: p[1])
        t_peak = max(t_candidates, key=lambda p: p[1])

        # 确保C峰在T峰之前
        if c_peak[0] >= t_peak[0]:
            t_candidates_after_c = [p for p in t_candidates if p[0] > c_peak[0]]
            if t_candidates_after_c:
                t_peak = max(t_candidates_after_c, key=lambda p: p[1])
            else:
                c_peak = None
                t_peak = None
    else:
        # 如果没有在指定范围内找到峰值，使用默认逻辑
        for i in range(len(peaks_sorted) - 1):
            p1 = peaks_sorted[i]
            p2 = peaks_sorted[i + 1]
            if abs(p2[0] - p1[0]) >= min_peak_distance:
                c_peak = p1
                t_peak = p2
                break

    return c_peak, t_peak


# ---------------------------
# 主窗口（GUI）
# ---------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("荧光显示仪（C/T峰值版）")
        self.resize(1400, 700)  # 加宽窗口适配峰值显示

        # 数据存储
        self.N = 5000  # 最大缓存数据量
        self.y: Deque[float] = deque(maxlen=self.N)  # 荧光强度值
        self.t: Deque[float] = deque(maxlen=self.N)  # 时间戳
        self.raw_last: str = ""  # 最后一条原始数据

        # 串口线程
        self.worker: Optional[SerialWorker] = None

        # 报警相关
        self.alarming = False
        self._blink_state = False

        # 自动保存相关
        self.auto_dir = ""
        self._current_bucket = ""  # 自动保存文件分桶（按分钟）
        self._auto_fp = None

        # 峰值识别参数
        self.min_peak_height = 5000  # 最小峰值高度
        self.min_peak_distance = 100  # 峰值最小间隔
        self.c_peak_value = None  # C峰值数值
        self.t_peak_value = None  # T峰值数值
        self.ratio_tc = None  # T/C比值

        # C/T线横坐标范围
        self.c_x_min = 200
        self.c_x_max = 400
        self.t_x_min = 400
        self.t_x_max = 800

        # ---------------------------
        # UI布局（从 Qt Designer 的 .ui 加载）
        # 说明：
        # - 你可以用 Qt Designer 打开 fluor_range_form.ui 自由拖拽改布局；
        # - 只要控件 objectName 不变（例如 portCB/thSpin/textBox 等），下面逻辑代码无需改动。
        # ---------------------------
        from PySide6.QtUiTools import QUiLoader

        base = os.path.dirname(__file__)
        # 优先使用兼容 Qt5 Designer 的版本（如果存在）
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

        # ---- 重要：findChild 可能返回 None（控件 objectName 不匹配 / UI没加载到位）
        # VSCode 的红线通常就是因为这里被推断成 Optional。
        # 用 must() 做“必存在”检查：找不到就立刻给出清晰报错；并且用 cast 消除类型检查器告警。
        T = TypeVar("T", bound=QtCore.QObject)
        def must(cls: Type[T], name: str) -> T:
            w = ui.findChild(cls, name)
            if w is None:
                raise RuntimeError(f"UI里找不到控件: {name}（请在 Designer 的 Object Inspector 里确认 objectName=\"{name}\"）")
            return cast(T, w)

        # 绑定 .ui 里的控件到 self（保持与原代码同名）
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

        self.cMinSpin = must(QtWidgets.QDoubleSpinBox, "cMinSpin")
        self.cMaxSpin = must(QtWidgets.QDoubleSpinBox, "cMaxSpin")
        self.tMinSpin = must(QtWidgets.QDoubleSpinBox, "tMinSpin")
        self.tMaxSpin = must(QtWidgets.QDoubleSpinBox, "tMaxSpin")

        self.textBox = must(QtWidgets.QPlainTextEdit, "textBox")

        # 历史曲线面板
        self.historyList = must(QtWidgets.QListWidget, "historyList")
        self.saveRunBtn = must(QtWidgets.QPushButton, "saveRunBtn")
        self.removeRunBtn = must(QtWidgets.QPushButton, "removeRunBtn")
        self.clearHistoryBtn = must(QtWidgets.QPushButton, "clearHistoryBtn")

        self.latestLab = must(QtWidgets.QLabel, "latestLab")
        self.rawLab = must(QtWidgets.QLabel, "rawLab")
        self.cPeakLab = must(QtWidgets.QLabel, "cPeakLab")
        self.tPeakLab = must(QtWidgets.QLabel, "tPeakLab")
        self.ratioLab = must(QtWidgets.QLabel, "ratioLab")

        plotLayout = must(QtWidgets.QVBoxLayout, "plotLayout")
        if plotLayout is None:
            raise RuntimeError("UI里找不到 plotLayout（请检查 plotContainer 里的 QVBoxLayout objectName 是否为 plotLayout）")

        # 把 Matplotlib 画布塞进占位布局
        self.canvas = NeonCanvas()
        plotLayout.addWidget(self.canvas)

        # ---------------------------
        # 历史记录存储
        # ---------------------------
        self.max_history = 12  # 最多保留多少次（可自行调大）
        self._run_seq = 0
        self._history: Dict[int, RunRecord] = {}
        self._history_order: List[int] = []

        # 字体/显示细节（Designer 里也能改，但这里确保跨平台一致）
        self.textBox.setFont(QtGui.QFont(get_monospace_font(), 11))
        for lab in (self.latestLab, self.cPeakLab, self.tPeakLab, self.ratioLab):
            lab.setFont(QtGui.QFont(get_monospace_font(), 16))

        # 刷新按钮：刷新串口列表
        self.refreshBtn.clicked.connect(self.refresh_ports)

        # 状态栏
        self.statusBar().showMessage("未连接")

# ---------------------------
        # 信号与槽绑定
        # ---------------------------
        # 控制按钮
        self.startBtn.clicked.connect(self.start_read)
        self.stopBtn.clicked.connect(self.stop_read)
        self.clearBtn.clicked.connect(self.clear_all)

        # 历史曲线：保存/勾选显示/删除
        self.saveRunBtn.clicked.connect(lambda: self.save_current_to_history(manual=True))
        self.removeRunBtn.clicked.connect(self.remove_selected_history)
        self.clearHistoryBtn.clicked.connect(self.clear_history)
        self.historyList.itemChanged.connect(self.on_history_item_changed)

        # 数据导出/目录选择
        self.exportBtn.clicked.connect(self.export_csv)
        self.pickDirBtn.clicked.connect(self.pick_dir)

        # 阈值报警
        self.thSpin.valueChanged.connect(self.on_threshold_changed)
        self.alarmEnableChk.toggled.connect(self.on_threshold_changed)

        # 绘图参数
        self.fpsCB.currentTextChanged.connect(self.apply_fps)

        # 峰值参数
        self.peakHeightSpin.valueChanged.connect(self.update_peak_params)
        self.peakDistanceSpin.valueChanged.connect(self.update_peak_params)

        # C/T线范围参数
        self.cMinSpin.valueChanged.connect(self.update_ct_ranges)
        self.cMaxSpin.valueChanged.connect(self.update_ct_ranges)
        self.tMinSpin.valueChanged.connect(self.update_ct_ranges)
        self.tMaxSpin.valueChanged.connect(self.update_ct_ranges)

        # 定时器
        self.timer = QtCore.QTimer(self)  # 绘图定时器
        self.apply_fps(self.fpsCB.currentText())
        self.timer.timeout.connect(self.redraw)

        self.blinkTimer = QtCore.QTimer(self)  # 报警闪烁定时器
        self.blinkTimer.setInterval(250)
        self.blinkTimer.timeout.connect(self._blink_tick)

        self.autoTimer = QtCore.QTimer(self)  # 自动保存定时器
        self.autoTimer.setInterval(1000)
        self.autoTimer.timeout.connect(self._auto_save_tick)

        # 初始化
        self.refresh_ports()
        self._set_running(False)
        self.on_threshold_changed()
        self.update_peak_params()
        self.update_ct_ranges()

    # ---------------------------
    # 峰值参数更新
    # ---------------------------
    def update_peak_params(self):
        """更新峰值识别参数"""
        self.min_peak_height = self.peakHeightSpin.value()
        self.min_peak_distance = self.peakDistanceSpin.value()

    # ---------------------------
    # C/T线范围更新
    # ---------------------------
    def update_ct_ranges(self):
        """更新C/T线横坐标范围"""
        self.c_x_min = self.cMinSpin.value()
        self.c_x_max = self.cMaxSpin.value()
        self.t_x_min = self.tMinSpin.value()
        self.t_x_max = self.tMaxSpin.value()

    # ---------------------------
    # 串口相关
    # ---------------------------
    def _set_running(self, running: bool):
        """设置运行状态（启用/禁用按钮）"""
        self.startBtn.setEnabled(not running)
        self.stopBtn.setEnabled(running)

    def _append_log(self, s: str):
        """追加日志"""
        self.textBox.appendPlainText(s)

    def refresh_ports(self):
        """刷新串口列表"""
        self.portCB.blockSignals(True)
        cur = self.portCB.currentText().strip()
        self.portCB.clear()

        # 获取可用串口
        ports = [p.device for p in list_ports.comports()]
        if ports:
            self.portCB.addItems(ports)
            if cur in ports:
                self.portCB.setCurrentText(cur)
        else:
            self.portCB.addItem("手动输入串口号")
            self.portCB.setEditable(True)

        self.portCB.blockSignals(False)

    # ---------------------------
    # 阈值报警
    # ---------------------------
    def on_threshold_changed(self):
        """更新阈值线"""
        if self.alarmEnableChk.isChecked():
            th = float(self.thSpin.value())
            self.canvas.set_threshold(th)
        else:
            self.canvas.set_threshold(None)
        self.canvas.draw_idle()

    def _set_alarm(self, on: bool):
        """设置报警状态"""
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
        """报警闪烁逻辑"""
        if not self.alarming:
            return
        self._blink_state = not self._blink_state
        if self._blink_state:
            self.latestLab.setStyleSheet("background:#aa0000; color:white; padding:2px 6px;")
            if self.beepChk.isChecked():
                beep_once()
        else:
            self.latestLab.setStyleSheet("")

    # ---------------------------
    # 绘图帧率
    # ---------------------------
    def apply_fps(self, fps_text: str):
        """应用绘图帧率"""
        try:
            fps = int(fps_text)
            fps = max(1, min(60, fps))
        except Exception:
            fps = 20
        self.timer.setInterval(int(1000 / fps))

    # ---------------------------
    # 自动保存
    # ---------------------------
    def pick_dir(self):
        """选择自动保存目录"""
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "选择自动保存目录", self.auto_dir or os.getcwd())
        if d:
            self.auto_dir = d
            self.dirLab.setText(d)
            self.dirLab.setStyleSheet("color:#0a0;")

    def _auto_open_for_bucket(self, bucket: str):
        """打开指定分桶的自动保存文件"""
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
        """检查并轮转自动保存文件"""
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

    # ---------------------------
    # 开始/停止/清空
    # ---------------------------
    def start_read(self):
        """开始读取串口数据"""
        port = self.portCB.currentText().strip()
        try:
            baud = int(self.baudCB.currentText())
        except ValueError:
            baud = 115200

        # 清空历史数据
        self.y.clear()
        self.t.clear()
        self.raw_last = ""
        self.textBox.clear()
        self.canvas.set_data(np.array([]), np.array([]))

        # 重置显示
        self.latestLab.setText("—")
        self.rawLab.setText("")
        self.cPeakLab.setText("—")
        self.tPeakLab.setText("—")
        self.ratioLab.setText("—")
        self.c_peak_value = None
        self.t_peak_value = None
        self.ratio_tc = None
        self._set_alarm(False)

        self._append_log(f"[{time.strftime('%H:%M:%S')}] 已连接 {port}，等待数据...")
        self.statusBar().showMessage(f"连接中: {port} …")

        # 启动串口线程
        self.worker = SerialWorker(
            port, baud,
            reconnect=self.reconnectChk.isChecked(),
            reconnect_sec=1.0
        )
        self.worker.value.connect(self.on_value)
        self.worker.status.connect(self.on_status)
        self.worker.err.connect(self.on_err)
        self.worker.start()

        # 启动定时器
        self.timer.start()
        self.autoTimer.start()
        self._set_running(True)

    def stop_read(self):
        """停止读取串口数据"""
        self.timer.stop()
        self.autoTimer.stop()
        self._set_alarm(False)

        # 停止串口线程
        if self.worker and self.worker.isRunning():
            self.worker.requestInterruption()
            self.worker.wait(800)
        self.worker = None

        # 停止后把本次曲线保存到历史（有数据才保存）
        self.save_current_to_history(manual=False)

        # 关闭自动保存文件
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
        """清空所有数据和显示"""
        self.y.clear()
        self.t.clear()
        self.raw_last = ""
        self.canvas.set_data(np.array([]), np.array([]))

        self.canvas.set_peak_markers(None, None)  # 清空C/T峰标记
        self.canvas.draw_idle()  # 刷新画布使修改生效


        # 重置显示
        self.latestLab.setText("—")
        self.rawLab.setText("")
        self.cPeakLab.setText("—")
        self.tPeakLab.setText("—")
        self.ratioLab.setText("—")
        self.c_peak_value = None
        self.t_peak_value = None
        self.ratio_tc = None

        self.textBox.clear()
        self._set_alarm(False)
        self.statusBar().showMessage("已清空")

    # ---------------------------
    # 历史曲线（保留前几次测量 / 勾选显示）
    # ---------------------------
    def _make_plot_series(self, y_raw: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """把原始数据转换成用于显示的（下采样/可选平滑）曲线"""
        x = np.arange(y_raw.size)

        max_plot = 2000
        if y_raw.size > max_plot:
            stride = int(np.ceil(y_raw.size / max_plot))
            y_plot = y_raw[::stride]
            x_plot = x[::stride]
        else:
            y_plot = y_raw
            x_plot = x

        if self.smoothChk.isChecked() and y_plot.size >= 5:
            k = 5
            kernel = np.ones(k) / k
            y_plot = np.convolve(y_plot, kernel, mode="same")

        return x_plot.astype(float), y_plot.astype(float)

    def save_current_to_history(self, manual: bool = True):
        """保存当前缓存曲线为一条历史记录（manual=True 表示用户点击保存）"""
        if len(self.y) < 10:
            if manual:
                QtWidgets.QMessageBox.information(self, "历史曲线", "当前数据量太少，无法保存到历史。")
            return

        y_raw = np.fromiter(self.y, dtype=float)
        x_raw = np.arange(y_raw.size, dtype=float)
        x_plot, y_plot = self._make_plot_series(y_raw)

        # 用原始数据计算一次峰值（存档用）
        c_peak, t_peak = detect_ct_peaks(
            y_raw, x_raw,
            min_peak_height=self.min_peak_height,
            min_peak_distance=self.min_peak_distance,
            c_x_min=self.c_x_min,
            c_x_max=self.c_x_max,
            t_x_min=self.t_x_min,
            t_x_max=self.t_x_max
        )
        tc_ratio = None
        if c_peak and t_peak and c_peak[1] > 0:
            tc_ratio = float(t_peak[1] / c_peak[1])

        self._run_seq += 1
        run_id = self._run_seq
        ts = time.time()
        label = f"{time.strftime('%H:%M:%S')}  #{run_id:03d}  T/C={tc_ratio:.3f}" if tc_ratio is not None else f"{time.strftime('%H:%M:%S')}  #{run_id:03d}"

        rec = RunRecord(
            run_id=run_id,
            ts=ts,
            label=label,
            x_raw=x_raw,
            y_raw=y_raw,
            x_plot=x_plot,
            y_plot=y_plot,
            c_peak=c_peak,
            t_peak=t_peak,
            tc_ratio=tc_ratio,
            meta={
                "port": self.portCB.currentText().strip(),
                "baud": self.baudCB.currentText().strip(),
                "smooth": bool(self.smoothChk.isChecked()),
                "min_peak_height": float(self.min_peak_height),
                "min_peak_distance": int(self.min_peak_distance),
                "c_range": (float(self.c_x_min), float(self.c_x_max)),
                "t_range": (float(self.t_x_min), float(self.t_x_max)),
            },
        )

        # 超过上限时丢弃最旧记录
        if len(self._history_order) >= self.max_history:
            old_id = self._history_order.pop(0)
            self._history.pop(old_id, None)
            self.canvas.remove_history(old_id)
            # UI里也删最旧行
            self.historyList.blockSignals(True)
            try:
                self.historyList.takeItem(0)
            finally:
                self.historyList.blockSignals(False)

        self._history[run_id] = rec
        self._history_order.append(run_id)

        # 加入列表（可勾选）
        item = QtWidgets.QListWidgetItem(label)
        item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)
        item.setCheckState(QtCore.Qt.Checked)
        # 绑定本条历史记录的唯一ID，用于勾选/删除时索引
        item.setData(QtCore.Qt.UserRole, run_id)
        tip = []
        if c_peak:
            tip.append(f"C峰: x={c_peak[0]:.0f}, y={c_peak[1]:.1f}")
        if t_peak:
            tip.append(f"T峰: x={t_peak[0]:.0f}, y={t_peak[1]:.1f}")
        if tc_ratio is not None:
            tip.append(f"T/C: {tc_ratio:.3f}")
        item.setToolTip("\n".join(tip))

        self.historyList.blockSignals(True)
        try:
            self.historyList.addItem(item)
        finally:
            self.historyList.blockSignals(False)

        # 默认勾选即显示
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

    # ---------------------------
    # 数据处理
    # ---------------------------
    @QtCore.Slot(float, str)
    def on_value(self, v: float, raw: str):
        """处理串口数据"""
        now = time.time()
        self.y.append(v)
        self.t.append(now)
        self.raw_last = raw

        # 更新显示
        self.latestLab.setText(f"{v:.1f}")
        self.rawLab.setText(f"raw: {raw}")
        self._append_log(f"{v:.1f}")

        # 检查报警
        if self.alarmEnableChk.isChecked():
            th = float(self.thSpin.value())
            self._set_alarm(v > th)
        else:
            self._set_alarm(False)

        # 自动保存
        if self.autoSaveChk.isChecked() and self._auto_fp:
            ts_iso = datetime.fromtimestamp(now).isoformat(timespec="milliseconds")
            try:
                self._auto_writer.writerow([ts_iso, f"{now:.3f}", v, raw])
                self._auto_fp.flush()
            except Exception:
                pass

    @QtCore.Slot(str)
    def on_status(self, s: str):
        """更新状态信息"""
        self.statusBar().showMessage(s)
        self._append_log(f"[{time.strftime('%H:%M:%S')}] {s}")

    @QtCore.Slot(str)
    def on_err(self, e: str):
        """显示串口错误"""
        QtWidgets.QMessageBox.warning(self, "串口错误", e)
        self.stop_read()

    # ---------------------------
    # 绘图 + 峰值识别
    # ---------------------------
    @QtCore.Slot()
    def redraw(self):
        """重绘曲线并识别峰值"""
        if not self.y:
            return

        # 转换为numpy数组
        y = np.fromiter(self.y, dtype=float)
        x = np.arange(y.size)

        # 大数据降采样（提升绘图性能）
        max_plot = 2000
        if y.size > max_plot:
            stride = int(np.ceil(y.size / max_plot))
            y_plot = y[::stride]
            x_plot = x[::stride]
        else:
            y_plot = y
            x_plot = x

        # 平滑显示
        if self.smoothChk.isChecked() and y_plot.size >= 5:
            k = 5
            kernel = np.ones(k) / k
            y_plot = np.convolve(y_plot, kernel, mode="same")

        # 峰值识别（使用原始数据保证准确性）
        c_peak, t_peak = detect_ct_peaks(
            y, x,
            min_peak_height=self.min_peak_height,
            min_peak_distance=self.min_peak_distance,
            c_x_min=self.c_x_min,
            c_x_max=self.c_x_max,
            t_x_min=self.t_x_min,
            t_x_max=self.t_x_max
        )

        # 更新峰值显示
        if c_peak:
            self.c_peak_value = c_peak[1]
            self.cPeakLab.setText(f"{self.c_peak_value:.1f}")
        else:
            self.c_peak_value = None
            self.cPeakLab.setText("—")

        if t_peak:
            self.t_peak_value = t_peak[1]
            self.tPeakLab.setText(f"{self.t_peak_value:.1f}")
        else:
            self.t_peak_value = None
            self.tPeakLab.setText("—")

        # 计算T/C比值（避免除零错误）
        if self.c_peak_value and self.t_peak_value and self.c_peak_value > 0:
            self.ratio_tc = self.t_peak_value / self.c_peak_value
            self.ratioLab.setText(f"{self.ratio_tc:.3f}")
        else:
            self.ratio_tc = None
            self.ratioLab.setText("—")

        # 更新曲线和峰值标记
        self.canvas.set_data(x_plot, y_plot)
        self.canvas.set_peak_markers(c_peak, t_peak)
        self.canvas.draw_idle()

    # ---------------------------
    # 数据导出
    # ---------------------------
    def export_csv(self):
        """导出数据为CSV（含峰值汇总）"""
        if not self.y:
            QtWidgets.QMessageBox.information(self, "导出", "当前没有数据可导出。")
            return

        # 选择保存路径
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
                # 写入表头
                w.writerow(["timestamp_iso", "unix_time", "value", "raw"])
                # 写入原始数据
                for ts, val in zip(self.t, self.y):
                    ts_iso = datetime.fromtimestamp(ts).isoformat(timespec="milliseconds")
                    w.writerow([ts_iso, f"{ts:.3f}", val, self.raw_last])
                # 写入峰值汇总
                w.writerow([])
                w.writerow(["C&T peaks", "", "", ""])
                w.writerow(["C peak", self.c_peak_value if self.c_peak_value else "未识别", "", ""])
                w.writerow(["T peak", self.t_peak_value if self.t_peak_value else "未识别", "", ""])
                w.writerow(["T/C ratio", self.ratio_tc if self.ratio_tc else "未识别", "", ""])

            QtWidgets.QMessageBox.information(self, "导出", "数据导出完成（含峰值汇总）。")
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "导出失败", str(e))

    # ---------------------------
    # 窗口关闭
    # ---------------------------
    def closeEvent(self, e: QtGui.QCloseEvent):
        """窗口关闭事件"""
        self.stop_read()
        super().closeEvent(e)


# ---------------------------
# 程序入口
# ---------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())