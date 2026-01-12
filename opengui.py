import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # 加载 .ui 文件
        loader = QUiLoader()
        ui_file = QFile("florescence_gui.ui")
        ui_file.open(QFile.ReadOnly)
        self.window = loader.load(ui_file)
        ui_file.close()

        # 将 .ui 文件中的窗口设置为当前窗口
        self.setCentralWidget(self.window)

        # 获取 .ui 文件中的控件（比如按钮、文本框）
        # 假设你的 .ui 文件中有一个按钮叫 "pushButton"，一个标签叫 "label"
        self.pushButton = self.window.findChild(type(self.window.pushButton), "pushButton")
        self.label = self.window.findChild(type(self.window.label), "label")

        # 连接按钮点击事件
        if self.pushButton:
            self.pushButton.clicked.connect(self.on_button_click)

    def on_button_click(self):
        # 点击按钮时，改变标签文字
        if self.label:
            self.label.setText("你点击了按钮！")

# 创建应用
app = QApplication(sys.argv)
window = MainWindow()
window.window.show()  # 显示 .ui 文件中的窗口
app.exec()