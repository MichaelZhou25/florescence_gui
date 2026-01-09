# -*- coding: utf-8 -*-
"""
荧光显示仪 · 数字曲线版（仪器增强版）
新增：
1) 阈值线/报警：超过阈值曲线变红 + 信息闪烁 + 蜂鸣（可开关）
2) 数据保存：一键导出 CSV；自动按分钟分文件（可选目录）
3) 更强解析：支持 0xABCD、HEX(123)、带单位（mV、V、℃、C 等）
4) 帧率自适应：绘图定频 10/20/30Hz，读线程再快也不拖 UI；可降采样绘制
"""
import sys, re, os, time, csv
from typing import Optional, Deque, Tuple
from collections import deque
from datetime import datetime

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import QThread, Signal

import serial
import serial.tools.list_ports as list_ports

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np


# ---------------------------
# 蜂鸣（跨平台兜底）
# ---------------------------
def beep_once():
    try:
        import winsound  # Windows
        winsound.Beep(1200, 120)  # 频率Hz, 时长ms
    except Exception:
        try:
            QtWidgets.QApplication.beep()
        except Exception:
            pass


# ---------------------------
# 串口读取线程
# ---------------------------
class SerialWorker(QThread):
    value = Signal(float, str)  # (数值, 原始字符串)
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
                    # 自动重连等待
                    for _ in range(int(self.reconnect_sec * 10)):
                        if self.isInterruptionRequested():
                            self._safe_close()
                            return
                        self.msleep(100)
                    continue

            # 读取
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
        """
        支持：
        - 0xABCD / 0XABCD (十六进制)
        - HEX(24413)
        - 任意文本中的数字：-12.3
        - 带单位：123mV / 2.5V / 36℃ / 36°C / 36C
          （默认不做单位换算，只提取数值；如需换算可在下面开启 scale）
        """
        ss = s.strip()

        # 1) 0xABCD
        m = re.search(r"0x([0-9a-fA-F]+)", ss)
        if m:
            try:
                return float(int(m.group(1), 16))
            except Exception:
                return None

        # 2) HEX(12345) / hex(0xABCD) 也能覆盖一部分
        m = re.search(r"HEX\s*\(\s*(0x[0-9a-fA-F]+|[0-9a-fA-F]+)\s*\)", ss, flags=re.IGNORECASE)
        if m:
            token = m.group(1)
            try:
                if token.lower().startswith("0x"):
                    return float(int(token, 16))
                # 这里若是纯数字，按十进制；若你希望纯字母也按16进制，可自行改
                if re.fullmatch(r"\d+", token):
                    return float(int(token, 10))
                return float(int(token, 16))
            except Exception:
                return None

        # 3) 数字 + 可选单位
        m = re.search(r"(-?\d+(?:\.\d+)?)\s*([a-zA-Z°℃]+)?", ss)
        if not m:
            return None
        try:
            val = float(m.group(1))
        except Exception:
            return None

        unit = (m.group(2) or "").strip()

        # ✅ 默认：不做换算，仅提取数值
        # 如你想把 V -> mV 统一，可启用下面 scale：
        # u = unit.lower()
        # scale = {"v": 1000.0, "mv": 1.0, "c": 1.0, "°c": 1.0, "℃": 1.0}
        # if u in scale: val *= scale[u]

        return val


# ---------------------------
# Matplotlib 画布（阈值线 + 变色）
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
        self.th_line = self.ax.axhline(np.nan, lw=1.5, color="#66ff66", alpha=0.9)  # 阈值线（默认隐藏）
        fig.tight_layout()

    def set_threshold(self, th: Optional[float]):
        if th is None:
            self.th_line.set_ydata([np.nan, np.nan])
        else:
            self.th_line.set_ydata([th, th])

    def set_alarm_color(self, alarming: bool):
        self.line.set_color("red" if alarming else "cyan")

    def set_data(self, x: np.ndarray, y: np.ndarray):
        self.line.set_data(x, y)
        if x.size > 1:
            self.ax.set_xlim(max(0, x.min()), x.max())
            y_min, y_max = float(np.min(y)), float(np.max(y))
            pad = 1.0 if y_max == y_min else (y_max - y_min) * 0.08
            self.ax.set_ylim(y_min - pad, y_max + pad)
        self.draw_idle()


# ---------------------------
# 主窗口
# ---------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("荧光显示仪（仪器增强版）")
        self.resize(1200, 650)

        # 数据：值 + 时间戳（用于导出）
        self.N = 5000
        self.y: Deque[float] = deque(maxlen=self.N)
        self.t: Deque[float] = deque(maxlen=self.N)  # time.time()
        self.raw_last: str = ""

        self.worker: Optional[SerialWorker] = None

        # 报警相关
        self.alarming = False
        self._blink_state = False

        # 自动保存相关
        self.auto_dir = ""
        self._current_bucket = ""  # YYYYMMDD_HHMM
        self._auto_fp = None

        # -------- UI --------
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # 顶部控制区（串口 + 绘图帧率）
        top = QtWidgets.QHBoxLayout()

        self.portCB = QtWidgets.QComboBox()
        self.portCB.setMinimumWidth(140)

        self.refreshBtn = QtWidgets.QPushButton("刷新串口")
        self.refreshBtn.clicked.connect(self.refresh_ports)

        self.baudCB = QtWidgets.QComboBox()
        self.baudCB.addItems(["115200", "57600", "38400", "19200", "9600"])

        self.reconnectChk = QtWidgets.QCheckBox("自动重连")
        self.reconnectChk.setChecked(True)

        self.fpsCB = QtWidgets.QComboBox()
        self.fpsCB.addItems(["10", "20", "30"])
        self.fpsCB.setCurrentText("20")

        self.smoothChk = QtWidgets.QCheckBox("平滑显示")
        self.smoothChk.setChecked(True)

        self.startBtn = QtWidgets.QPushButton("开始")
        self.stopBtn = QtWidgets.QPushButton("停止")
        self.clearBtn = QtWidgets.QPushButton("清空")

        top.addWidget(QtWidgets.QLabel("串口:"))
        top.addWidget(self.portCB)
        top.addWidget(self.refreshBtn)
        top.addSpacing(8)
        top.addWidget(QtWidgets.QLabel("波特率:"))
        top.addWidget(self.baudCB)
        top.addSpacing(12)
        top.addWidget(self.reconnectChk)
        top.addSpacing(12)
        top.addWidget(QtWidgets.QLabel("绘图Hz:"))
        top.addWidget(self.fpsCB)
        top.addWidget(self.smoothChk)
        top.addStretch(1)
        top.addWidget(self.startBtn)
        top.addWidget(self.stopBtn)
        top.addWidget(self.clearBtn)
        root.addLayout(top)

        # 报警 / 保存区
        mid = QtWidgets.QHBoxLayout()

        # 阈值设置
        self.alarmEnableChk = QtWidgets.QCheckBox("启用阈值报警")
        self.alarmEnableChk.setChecked(True)

        self.thSpin = QtWidgets.QDoubleSpinBox()
        self.thSpin.setRange(-1e12, 1e12)
        self.thSpin.setDecimals(3)
        self.thSpin.setValue(1000.0)
        self.thSpin.setSingleStep(10.0)

        self.beepChk = QtWidgets.QCheckBox("蜂鸣")
        self.beepChk.setChecked(True)

        mid.addWidget(self.alarmEnableChk)
        mid.addWidget(QtWidgets.QLabel("阈值:"))
        mid.addWidget(self.thSpin)
        mid.addWidget(self.beepChk)
        mid.addStretch(1)

        # 导出/自动保存
        self.exportBtn = QtWidgets.QPushButton("导出CSV")
        self.autoSaveChk = QtWidgets.QCheckBox("自动按分钟分文件")
        self.autoSaveChk.setChecked(False)
        self.pickDirBtn = QtWidgets.QPushButton("选择目录")
        self.dirLab = QtWidgets.QLabel("未选择")
        self.dirLab.setStyleSheet("color: #888;")

        mid.addWidget(self.exportBtn)
        mid.addWidget(self.autoSaveChk)
        mid.addWidget(self.pickDirBtn)
        mid.addWidget(self.dirLab)

        root.addLayout(mid)

        # 主显示区
        main_layout = QtWidgets.QHBoxLayout()
        root.addLayout(main_layout, 1)

        # 左：信息框
        self.textBox = QtWidgets.QPlainTextEdit()
        self.textBox.setReadOnly(True)
        self.textBox.setFont(QtGui.QFont("Consolas", 11))
        self.textBox.setMaximumBlockCount(2000)  # ✅ 自动限行，不卡
        self.textBox.setStyleSheet(
            "QPlainTextEdit { background-color: #111; color: #0f0; border: 2px solid #244061; }"
        )
        main_layout.addWidget(self.textBox, 2)

        # 右：曲线
        self.canvas = NeonCanvas()
        main_layout.addWidget(self.canvas, 5)

        # 底部最新值
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("最新值:"))
        self.latestLab = QtWidgets.QLabel("—")
        self.latestLab.setFont(QtGui.QFont("Consolas", 16))
        row.addWidget(self.latestLab)
        self.rawLab = QtWidgets.QLabel("")
        self.rawLab.setStyleSheet("color:#888;")
        row.addWidget(self.rawLab)
        row.addStretch(1)
        root.addLayout(row)

        # 状态栏
        self.statusBar().showMessage("未连接")

        # 事件绑定
        self.startBtn.clicked.connect(self.start_read)
        self.stopBtn.clicked.connect(self.stop_read)
        self.clearBtn.clicked.connect(self.clear_all)
        self.exportBtn.clicked.connect(self.export_csv)
        self.pickDirBtn.clicked.connect(self.pick_dir)

        self.thSpin.valueChanged.connect(self.on_threshold_changed)
        self.alarmEnableChk.toggled.connect(self.on_threshold_changed)

        self.fpsCB.currentTextChanged.connect(self.apply_fps)

        # 绘图定时器（帧率自适应：只影响画，不影响读）
        self.timer = QtCore.QTimer(self)
        self.apply_fps(self.fpsCB.currentText())
        self.timer.timeout.connect(self.redraw)

        # 闪烁定时器
        self.blinkTimer = QtCore.QTimer(self)
        self.blinkTimer.setInterval(250)
        self.blinkTimer.timeout.connect(self._blink_tick)

        # 自动保存轮转定时器（每秒检查一次是否跨分钟）
        self.autoTimer = QtCore.QTimer(self)
        self.autoTimer.setInterval(1000)
        self.autoTimer.timeout.connect(self._auto_save_tick)

        # 初始化
        self.refresh_ports()
        self._set_running(False)
        self.on_threshold_changed()

    # ---------- 基础 UI ----------
    def _set_running(self, running: bool):
        self.startBtn.setEnabled(not running)
        self.stopBtn.setEnabled(running)

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

    # ---------- 阈值/报警 ----------
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
            self.latestLab.setStyleSheet("")  # 恢复
            self.statusBar().showMessage("正常")

    def _blink_tick(self):
        if not self.alarming:
            return
        self._blink_state = not self._blink_state
        # 闪烁：label 背景红/透明
        if self._blink_state:
            self.latestLab.setStyleSheet("background:#aa0000; color:white; padding:2px 6px;")
            if self.beepChk.isChecked():
                beep_once()
        else:
            self.latestLab.setStyleSheet("")

    # ---------- 帧率 ----------
    def apply_fps(self, fps_text: str):
        try:
            fps = int(fps_text)
            fps = max(1, min(60, fps))
        except Exception:
            fps = 20
        self.timer.setInterval(int(1000 / fps))

    # ---------- 自动保存 ----------
    def pick_dir(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "选择自动保存目录", self.auto_dir or os.getcwd())
        if d:
            self.auto_dir = d
            self.dirLab.setText(d)
            self.dirLab.setStyleSheet("color:#0a0;")

    def _auto_open_for_bucket(self, bucket: str):
        # 关闭旧文件
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
            # 关闭
            if self._auto_fp:
                try:
                    self._auto_fp.close()
                except Exception:
                    pass
                self._auto_fp = None
            self._current_bucket = ""
            return

        if not self.auto_dir:
            # 没选目录就不写
            return

        now = datetime.now()
        bucket = now.strftime("%Y%m%d_%H%M")
        if bucket != self._current_bucket or self._auto_fp is None:
            self._auto_open_for_bucket(bucket)

    # ---------- 开始/停止 ----------
    def start_read(self):
        port = self.portCB.currentText().strip()
        try:
            baud = int(self.baudCB.currentText())
        except ValueError:
            baud = 115200

        self.y.clear()
        self.t.clear()
        self.raw_last = ""
        self.textBox.clear()
        self.canvas.set_data(np.array([]), np.array([]))
        self.latestLab.setText("—")
        self.rawLab.setText("")
        self._set_alarm(False)

        self._append_log(f"[{time.strftime('%H:%M:%S')}] 已经连接 {port}，请按下 K1 键")
        self.statusBar().showMessage(f"连接中: {port} …")

        self.worker = SerialWorker(
            port, baud,
            reconnect=self.reconnectChk.isChecked(),
            reconnect_sec=1.0
        )
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

        # 关闭自动文件
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
        self.latestLab.setText("—")
        self.rawLab.setText("")
        self.textBox.clear()
        self._set_alarm(False)
        self.statusBar().showMessage("已清空")

    # ---------- 数据到达 ----------
    @QtCore.Slot(float, str)
    def on_value(self, v: float, raw: str):
        now = time.time()
        self.y.append(v)
        self.t.append(now)
        self.raw_last = raw

        self.latestLab.setText(str(int(v)))
        self.rawLab.setText(f"raw: {raw}")

        # 信息框：按需你可改成每N条记录一次
        self._append_log(str(int(v)))

        # 阈值判断
        if self.alarmEnableChk.isChecked():
            th = float(self.thSpin.value())
            self._set_alarm(v > th)
        else:
            self._set_alarm(False)

        # 自动保存：写当前行（文件轮转在 _auto_save_tick 做）
        if self.autoSaveChk.isChecked() and self._auto_fp:
            ts_iso = datetime.fromtimestamp(now).isoformat(timespec="milliseconds")
            try:
                self._auto_writer.writerow([ts_iso, f"{now:.3f}", v, raw])
                # 适度 flush，避免断电丢数据（可按需调低频率）
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

    # ---------- 绘图（帧率定频 + 可降采样） ----------
    @QtCore.Slot()
    def redraw(self):
        if not self.y:
            return

        y = np.fromiter(self.y, dtype=float)
        x = np.arange(y.size)

        # 大数据降采样绘图（避免 UI 卡）
        max_plot = 2000
        if y.size > max_plot:
            stride = int(np.ceil(y.size / max_plot))
            y = y[::stride]
            x = x[::stride]

        # 平滑
        if self.smoothChk.isChecked() and y.size >= 5:
            k = 5
            kernel = np.ones(k) / k
            y = np.convolve(y, kernel, mode="same")

        self.canvas.set_data(x, y)

    # ---------- 导出 CSV ----------
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
                w.writerow(["timestamp_iso", "unix_time", "value"])
                for ts, val in zip(self.t, self.y):
                    ts_iso = datetime.fromtimestamp(ts).isoformat(timespec="milliseconds")
                    w.writerow([ts_iso, f"{ts:.3f}", val])
            QtWidgets.QMessageBox.information(self, "导出", "导出完成。")
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
