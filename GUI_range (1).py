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
from typing import Optional, Deque, Tuple, List
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

    def set_threshold(self, th: Optional[float]):
        """设置阈值线"""
        if th is None:
            self.th_line.set_ydata([np.nan, np.nan])
        else:
            self.th_line.set_ydata([th, th])

    def set_alarm_color(self, alarming: bool):
        """设置曲线报警颜色"""
        self.line.set_color("red" if alarming else "cyan")

    def set_peak_markers(self, c_peak: Tuple[float, float] = None, t_peak: Tuple[float, float] = None):
        """设置峰值标记位置"""
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
        self.line.set_data(x, y)
        if x.size > 1:
            self.ax.set_xlim(max(0, x.min()), x.max())
            y_min, y_max = float(np.min(y)), float(np.max(y))
            pad = 1.0 if y_max == y_min else (y_max - y_min) * 0.08
            self.ax.set_ylim(y_min - pad, y_max + pad)
        self.draw_idle()


# ---------------------------
# C/T峰值识别算法
# ---------------------------
def detect_ct_peaks(y_data: np.ndarray, x_data: np.ndarray,
                    min_peak_height: float = 5000,
                    min_peak_distance: int = 100,
                    c_x_min: float = 0.0,
                    c_x_max: float = 10000.0,
                    t_x_min: float = 0.0,
                    t_x_max: float = 10000.0) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float]]]:
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
    # 数据量不足时不识别
    if len(y_data) < 10:
        return None, None

    # 步骤1：滑动平均平滑去噪（减少噪声干扰）
    smoothed_y = y_data
    if len(y_data) >= 7:
        kernel = np.ones(7) / 7
        smoothed_y = np.convolve(y_data, kernel, mode="same")

    # 步骤2：检测局部极大值（当前点比前后5个点都大）
    peak_indices = argrelextrema(smoothed_y, np.greater, order=5)[0]
    if len(peak_indices) < 2:  # 峰值数量不足2个
        return None, None

    # 步骤3：筛选高度达标的峰值
    peaks = [(x_data[i], smoothed_y[i]) for i in peak_indices if smoothed_y[i] >= min_peak_height]
    if len(peaks) < 2:
        return None, None

    # 步骤4：按横坐标排序，筛选相隔足够远的两个峰值（C在前，T在后）
    peaks_sorted = sorted(peaks, key=lambda p: p[0])
    c_peak = None
    t_peak = None

    # 先在C线范围内找峰值
    c_candidates = [p for p in peaks_sorted if c_x_min <= p[0] <= c_x_max]
    # 在T线范围内找峰值
    t_candidates = [p for p in peaks_sorted if t_x_min <= p[0] <= t_x_max]

    # 如果在指定范围内找到合适的C和T峰值
    if c_candidates and t_candidates:
        # 选择C线范围内最高的峰值
        c_peak = max(c_candidates, key=lambda p: p[1])
        # 选择T线范围内最高的峰值
        t_peak = max(t_candidates, key=lambda p: p[1])
        # 确保C峰在T峰之前
        if c_peak[0] >= t_peak[0]:
            # 如果C峰在T峰之后，尝试找下一个合适的T峰
            t_candidates_after_c = [p for p in t_candidates if p[0] > c_peak[0]]
            if t_candidates_after_c:
                t_peak = max(t_candidates_after_c, key=lambda p: p[1])
            else:
                # 如果没有合适的T峰在C峰之后，重置
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
        # UI布局
        # ---------------------------
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # 1. 顶部控制区（串口配置 + 绘图参数）
        top_layout = QtWidgets.QHBoxLayout()

        # 串口选择
        self.portCB = QtWidgets.QComboBox()
        self.portCB.setMinimumWidth(140)
        self.refreshBtn = QtWidgets.QPushButton("刷新串口")
        self.refreshBtn.clicked.connect(self.refresh_ports)

        # 波特率选择
        self.baudCB = QtWidgets.QComboBox()
        self.baudCB.addItems(["9600","19200","38400","57600","115200"])

        # 自动重连
        self.reconnectChk = QtWidgets.QCheckBox("自动重连")
        self.reconnectChk.setChecked(True)

        # 绘图帧率
        self.fpsCB = QtWidgets.QComboBox()
        self.fpsCB.addItems(["10", "20", "30"])
        self.fpsCB.setCurrentText("20")

        # 平滑显示
        self.smoothChk = QtWidgets.QCheckBox("平滑显示")
        self.smoothChk.setChecked(True)

        # 控制按钮
        self.startBtn = QtWidgets.QPushButton("开始")
        self.stopBtn = QtWidgets.QPushButton("停止")
        self.clearBtn = QtWidgets.QPushButton("清空")

        # 组装顶部布局
        top_layout.addWidget(QtWidgets.QLabel("串口:"))
        top_layout.addWidget(self.portCB)
        top_layout.addWidget(self.refreshBtn)
        top_layout.addSpacing(8)
        top_layout.addWidget(QtWidgets.QLabel("波特率:"))
        top_layout.addWidget(self.baudCB)
        top_layout.addSpacing(12)
        top_layout.addWidget(self.reconnectChk)
        top_layout.addSpacing(12)
        top_layout.addWidget(QtWidgets.QLabel("绘图Hz:"))
        top_layout.addWidget(self.fpsCB)
        top_layout.addWidget(self.smoothChk)
        top_layout.addStretch(1)
        top_layout.addWidget(self.startBtn)
        top_layout.addWidget(self.stopBtn)
        top_layout.addWidget(self.clearBtn)
        root.addLayout(top_layout)

        # 2. 报警/保存区
        mid_layout = QtWidgets.QHBoxLayout()

        # 阈值报警配置
        self.alarmEnableChk = QtWidgets.QCheckBox("启用阈值报警")
        self.alarmEnableChk.setChecked(True)
        self.thSpin = QtWidgets.QDoubleSpinBox()
        self.thSpin.setRange(-1e12, 1e12)
        self.thSpin.setDecimals(3)
        self.thSpin.setValue(10000.0)
        self.thSpin.setSingleStep(10.0)
        self.beepChk = QtWidgets.QCheckBox("蜂鸣")
        self.beepChk.setChecked(True)

        # 数据导出/自动保存
        self.exportBtn = QtWidgets.QPushButton("导出CSV")
        self.autoSaveChk = QtWidgets.QCheckBox("自动按分钟分文件")
        self.autoSaveChk.setChecked(False)
        self.pickDirBtn = QtWidgets.QPushButton("选择目录")
        self.dirLab = QtWidgets.QLabel("未选择")
        self.dirLab.setStyleSheet("color: #888;")

        # 组装报警/保存布局
        mid_layout.addWidget(self.alarmEnableChk)
        mid_layout.addWidget(QtWidgets.QLabel("阈值:"))
        mid_layout.addWidget(self.thSpin)
        mid_layout.addWidget(self.beepChk)
        mid_layout.addStretch(1)
        mid_layout.addWidget(self.exportBtn)
        mid_layout.addWidget(self.autoSaveChk)
        mid_layout.addWidget(self.pickDirBtn)
        mid_layout.addWidget(self.dirLab)
        root.addLayout(mid_layout)

        # 3. 峰值识别参数区
        peak_param_layout = QtWidgets.QHBoxLayout()
        peak_param_layout.addWidget(QtWidgets.QLabel("峰值识别参数："))

        self.peakHeightSpin = QtWidgets.QDoubleSpinBox()
        self.peakHeightSpin.setRange(0, 1e6)
        self.peakHeightSpin.setDecimals(1)
        self.peakHeightSpin.setValue(50.0)
        self.peakHeightSpin.setToolTip("最小峰值高度（过滤噪声峰）")
        peak_param_layout.addWidget(QtWidgets.QLabel("最小高度:"))
        peak_param_layout.addWidget(self.peakHeightSpin)

        self.peakDistanceSpin = QtWidgets.QSpinBox()
        self.peakDistanceSpin.setRange(10, 500)
        self.peakDistanceSpin.setValue(50)
        self.peakDistanceSpin.setToolTip("峰值最小间隔（确保C/T峰相隔较远）")
        peak_param_layout.addWidget(QtWidgets.QLabel("最小间隔:"))
        peak_param_layout.addWidget(self.peakDistanceSpin)
        peak_param_layout.addStretch(1)
        root.addLayout(peak_param_layout)

        # 4. C/T线横坐标范围设置区
        range_layout = QtWidgets.QHBoxLayout()
        range_layout.addWidget(QtWidgets.QLabel("C线范围:"))
        self.cMinSpin = QtWidgets.QDoubleSpinBox()
        self.cMinSpin.setRange(0, 10000)
        self.cMinSpin.setDecimals(1)
        self.cMinSpin.setValue(200.0)
        self.cMinSpin.setToolTip("C线最小横坐标")
        range_layout.addWidget(QtWidgets.QLabel("最小:"))
        range_layout.addWidget(self.cMinSpin)

        self.cMaxSpin = QtWidgets.QDoubleSpinBox()
        self.cMaxSpin.setRange(0, 10000)
        self.cMaxSpin.setDecimals(1)
        self.cMaxSpin.setValue(400.0)
        self.cMaxSpin.setToolTip("C线最大横坐标")
        range_layout.addWidget(QtWidgets.QLabel("最大:"))
        range_layout.addWidget(self.cMaxSpin)

        range_layout.addSpacing(30)
        range_layout.addWidget(QtWidgets.QLabel("T线范围:"))
        self.tMinSpin = QtWidgets.QDoubleSpinBox()
        self.tMinSpin.setRange(0, 10000)
        self.tMinSpin.setDecimals(1)
        self.tMinSpin.setValue(400.0)
        self.tMinSpin.setToolTip("T线最小横坐标")
        range_layout.addWidget(QtWidgets.QLabel("最小:"))
        range_layout.addWidget(self.tMinSpin)

        self.tMaxSpin = QtWidgets.QDoubleSpinBox()
        self.tMaxSpin.setRange(0, 10000)
        self.tMaxSpin.setDecimals(1)
        self.tMaxSpin.setValue(800.0)
        self.tMaxSpin.setToolTip("T线最大横坐标")
        range_layout.addWidget(QtWidgets.QLabel("最大:"))
        range_layout.addWidget(self.tMaxSpin)

        range_layout.addStretch(1)
        root.addLayout(range_layout)

        # 5. 主显示区（日志 + 曲线）
        main_layout = QtWidgets.QHBoxLayout()
        root.addLayout(main_layout, 1)

        # 左侧：日志框（跨平台等宽字体）
        self.textBox = QtWidgets.QPlainTextEdit()
        self.textBox.setReadOnly(True)
        self.textBox.setFont(QtGui.QFont(get_monospace_font(), 11))
        self.textBox.setMaximumBlockCount(2000)
        self.textBox.setStyleSheet(
            "QPlainTextEdit { background-color: #111; color: #0f0; border: 2px solid #244061; }"
        )
        main_layout.addWidget(self.textBox, 2)

        # 右侧：绘图画布
        self.canvas = NeonCanvas()
        main_layout.addWidget(self.canvas, 5)

        # 6. 底部：数值显示区（最新值 + 峰值）
        bottom_layout = QtWidgets.QHBoxLayout()

        # 最新值显示
        latest_layout = QtWidgets.QHBoxLayout()
        latest_layout.addWidget(QtWidgets.QLabel("最新值:"))
        self.latestLab = QtWidgets.QLabel("—")
        self.latestLab.setFont(QtGui.QFont(get_monospace_font(), 16))
        latest_layout.addWidget(self.latestLab)
        self.rawLab = QtWidgets.QLabel("")
        self.rawLab.setStyleSheet("color:#888;")
        latest_layout.addWidget(self.rawLab)
        bottom_layout.addLayout(latest_layout)
        bottom_layout.addSpacing(50)

        # 峰值显示（C峰绿、T峰黄、比值橙）
        peak_layout = QtWidgets.QHBoxLayout()
        peak_layout.addWidget(QtWidgets.QLabel("C峰值:"))
        self.cPeakLab = QtWidgets.QLabel("—")
        self.cPeakLab.setFont(QtGui.QFont(get_monospace_font(), 16))
        self.cPeakLab.setStyleSheet("color: #00ff00;")
        peak_layout.addWidget(self.cPeakLab)
        peak_layout.addSpacing(20)

        peak_layout.addWidget(QtWidgets.QLabel("T峰值:"))
        self.tPeakLab = QtWidgets.QLabel("—")
        self.tPeakLab.setFont(QtGui.QFont(get_monospace_font(), 16))
        self.tPeakLab.setStyleSheet("color: #ffff00;")
        peak_layout.addWidget(self.tPeakLab)
        peak_layout.addSpacing(20)

        peak_layout.addWidget(QtWidgets.QLabel("T/C比值:"))
        self.ratioLab = QtWidgets.QLabel("—")
        self.ratioLab.setFont(QtGui.QFont(get_monospace_font(), 16))
        self.ratioLab.setStyleSheet("color: #ff6600;")
        peak_layout.addWidget(self.ratioLab)

        bottom_layout.addLayout(peak_layout)
        bottom_layout.addStretch(1)
        root.addLayout(bottom_layout)

        # 状态栏
        self.statusBar().showMessage("未连接")

        # ---------------------------
        # 信号与槽绑定
        # ---------------------------
        # 控制按钮
        self.startBtn.clicked.connect(self.start_read)
        self.stopBtn.clicked.connect(self.stop_read)
        self.clearBtn.clicked.connect(self.clear_all)

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