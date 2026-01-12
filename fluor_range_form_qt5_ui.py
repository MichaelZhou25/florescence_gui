# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'fluor_range_form_qt5.ui'
##
## Created by: Qt User Interface Compiler version 6.9.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QHBoxLayout, QLabel, QPlainTextEdit, QPushButton,
    QSizePolicy, QSpacerItem, QSpinBox, QVBoxLayout,
    QWidget)

class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(1400, 700)
        self.rootLayout = QVBoxLayout(Form)
        self.rootLayout.setObjectName(u"rootLayout")
        self.topLayout = QHBoxLayout()
        self.topLayout.setObjectName(u"topLayout")
        self.labelPort = QLabel(Form)
        self.labelPort.setObjectName(u"labelPort")

        self.topLayout.addWidget(self.labelPort)

        self.portCB = QComboBox(Form)
        self.portCB.setObjectName(u"portCB")
        self.portCB.setMinimumSize(QSize(140, 0))

        self.topLayout.addWidget(self.portCB)

        self.refreshBtn = QPushButton(Form)
        self.refreshBtn.setObjectName(u"refreshBtn")

        self.topLayout.addWidget(self.refreshBtn)

        self.spacerTop1 = QSpacerItem(8, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.topLayout.addItem(self.spacerTop1)

        self.labelBaud = QLabel(Form)
        self.labelBaud.setObjectName(u"labelBaud")

        self.topLayout.addWidget(self.labelBaud)

        self.baudCB = QComboBox(Form)
        self.baudCB.addItem("")
        self.baudCB.addItem("")
        self.baudCB.addItem("")
        self.baudCB.addItem("")
        self.baudCB.addItem("")
        self.baudCB.setObjectName(u"baudCB")
        self.baudCB.setMinimumSize(QSize(90, 0))

        self.topLayout.addWidget(self.baudCB)

        self.spacerTop2 = QSpacerItem(12, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.topLayout.addItem(self.spacerTop2)

        self.reconnectChk = QCheckBox(Form)
        self.reconnectChk.setObjectName(u"reconnectChk")
        self.reconnectChk.setChecked(True)

        self.topLayout.addWidget(self.reconnectChk)

        self.spacerTop3 = QSpacerItem(12, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.topLayout.addItem(self.spacerTop3)

        self.labelFps = QLabel(Form)
        self.labelFps.setObjectName(u"labelFps")

        self.topLayout.addWidget(self.labelFps)

        self.fpsCB = QComboBox(Form)
        self.fpsCB.addItem("")
        self.fpsCB.addItem("")
        self.fpsCB.addItem("")
        self.fpsCB.setObjectName(u"fpsCB")

        self.topLayout.addWidget(self.fpsCB)

        self.smoothChk = QCheckBox(Form)
        self.smoothChk.setObjectName(u"smoothChk")
        self.smoothChk.setChecked(True)

        self.topLayout.addWidget(self.smoothChk)

        self.spacerTopStretch = QSpacerItem(40, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.topLayout.addItem(self.spacerTopStretch)

        self.startBtn = QPushButton(Form)
        self.startBtn.setObjectName(u"startBtn")

        self.topLayout.addWidget(self.startBtn)

        self.stopBtn = QPushButton(Form)
        self.stopBtn.setObjectName(u"stopBtn")

        self.topLayout.addWidget(self.stopBtn)

        self.clearBtn = QPushButton(Form)
        self.clearBtn.setObjectName(u"clearBtn")

        self.topLayout.addWidget(self.clearBtn)


        self.rootLayout.addLayout(self.topLayout)

        self.midLayout = QHBoxLayout()
        self.midLayout.setObjectName(u"midLayout")
        self.alarmEnableChk = QCheckBox(Form)
        self.alarmEnableChk.setObjectName(u"alarmEnableChk")
        self.alarmEnableChk.setChecked(True)

        self.midLayout.addWidget(self.alarmEnableChk)

        self.labelTh = QLabel(Form)
        self.labelTh.setObjectName(u"labelTh")

        self.midLayout.addWidget(self.labelTh)

        self.thSpin = QDoubleSpinBox(Form)
        self.thSpin.setObjectName(u"thSpin")
        self.thSpin.setMinimum(-1000000000000.000000000000000)
        self.thSpin.setMaximum(1000000000000.000000000000000)
        self.thSpin.setDecimals(3)
        self.thSpin.setSingleStep(10.000000000000000)
        self.thSpin.setValue(10000.000000000000000)

        self.midLayout.addWidget(self.thSpin)

        self.beepChk = QCheckBox(Form)
        self.beepChk.setObjectName(u"beepChk")
        self.beepChk.setChecked(True)

        self.midLayout.addWidget(self.beepChk)

        self.spacerMidStretch = QSpacerItem(40, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.midLayout.addItem(self.spacerMidStretch)

        self.exportBtn = QPushButton(Form)
        self.exportBtn.setObjectName(u"exportBtn")

        self.midLayout.addWidget(self.exportBtn)

        self.autoSaveChk = QCheckBox(Form)
        self.autoSaveChk.setObjectName(u"autoSaveChk")
        self.autoSaveChk.setChecked(False)

        self.midLayout.addWidget(self.autoSaveChk)

        self.pickDirBtn = QPushButton(Form)
        self.pickDirBtn.setObjectName(u"pickDirBtn")

        self.midLayout.addWidget(self.pickDirBtn)

        self.dirLab = QLabel(Form)
        self.dirLab.setObjectName(u"dirLab")

        self.midLayout.addWidget(self.dirLab)


        self.rootLayout.addLayout(self.midLayout)

        self.peakParamLayout = QHBoxLayout()
        self.peakParamLayout.setObjectName(u"peakParamLayout")
        self.labelPeakParam = QLabel(Form)
        self.labelPeakParam.setObjectName(u"labelPeakParam")

        self.peakParamLayout.addWidget(self.labelPeakParam)

        self.labelMinHeight = QLabel(Form)
        self.labelMinHeight.setObjectName(u"labelMinHeight")

        self.peakParamLayout.addWidget(self.labelMinHeight)

        self.peakHeightSpin = QDoubleSpinBox(Form)
        self.peakHeightSpin.setObjectName(u"peakHeightSpin")
        self.peakHeightSpin.setMinimum(0.000000000000000)
        self.peakHeightSpin.setMaximum(1000000.000000000000000)
        self.peakHeightSpin.setDecimals(1)
        self.peakHeightSpin.setValue(50.000000000000000)

        self.peakParamLayout.addWidget(self.peakHeightSpin)

        self.labelMinDist = QLabel(Form)
        self.labelMinDist.setObjectName(u"labelMinDist")

        self.peakParamLayout.addWidget(self.labelMinDist)

        self.peakDistanceSpin = QSpinBox(Form)
        self.peakDistanceSpin.setObjectName(u"peakDistanceSpin")
        self.peakDistanceSpin.setMinimum(10)
        self.peakDistanceSpin.setMaximum(500)
        self.peakDistanceSpin.setValue(50)

        self.peakParamLayout.addWidget(self.peakDistanceSpin)

        self.spacerPeakStretch = QSpacerItem(40, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.peakParamLayout.addItem(self.spacerPeakStretch)


        self.rootLayout.addLayout(self.peakParamLayout)

        self.rangeLayout = QHBoxLayout()
        self.rangeLayout.setObjectName(u"rangeLayout")
        self.labelCRange = QLabel(Form)
        self.labelCRange.setObjectName(u"labelCRange")

        self.rangeLayout.addWidget(self.labelCRange)

        self.labelCMin = QLabel(Form)
        self.labelCMin.setObjectName(u"labelCMin")

        self.rangeLayout.addWidget(self.labelCMin)

        self.cMinSpin = QDoubleSpinBox(Form)
        self.cMinSpin.setObjectName(u"cMinSpin")
        self.cMinSpin.setMinimum(0.000000000000000)
        self.cMinSpin.setMaximum(10000.000000000000000)
        self.cMinSpin.setDecimals(1)
        self.cMinSpin.setValue(200.000000000000000)

        self.rangeLayout.addWidget(self.cMinSpin)

        self.labelCMax = QLabel(Form)
        self.labelCMax.setObjectName(u"labelCMax")

        self.rangeLayout.addWidget(self.labelCMax)

        self.cMaxSpin = QDoubleSpinBox(Form)
        self.cMaxSpin.setObjectName(u"cMaxSpin")
        self.cMaxSpin.setMinimum(0.000000000000000)
        self.cMaxSpin.setMaximum(10000.000000000000000)
        self.cMaxSpin.setDecimals(1)
        self.cMaxSpin.setValue(400.000000000000000)

        self.rangeLayout.addWidget(self.cMaxSpin)

        self.spacerRangeGap = QSpacerItem(30, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.rangeLayout.addItem(self.spacerRangeGap)

        self.labelTRange = QLabel(Form)
        self.labelTRange.setObjectName(u"labelTRange")

        self.rangeLayout.addWidget(self.labelTRange)

        self.labelTMin = QLabel(Form)
        self.labelTMin.setObjectName(u"labelTMin")

        self.rangeLayout.addWidget(self.labelTMin)

        self.tMinSpin = QDoubleSpinBox(Form)
        self.tMinSpin.setObjectName(u"tMinSpin")
        self.tMinSpin.setMinimum(0.000000000000000)
        self.tMinSpin.setMaximum(10000.000000000000000)
        self.tMinSpin.setDecimals(1)
        self.tMinSpin.setValue(400.000000000000000)

        self.rangeLayout.addWidget(self.tMinSpin)

        self.labelTMax = QLabel(Form)
        self.labelTMax.setObjectName(u"labelTMax")

        self.rangeLayout.addWidget(self.labelTMax)

        self.tMaxSpin = QDoubleSpinBox(Form)
        self.tMaxSpin.setObjectName(u"tMaxSpin")
        self.tMaxSpin.setMinimum(0.000000000000000)
        self.tMaxSpin.setMaximum(10000.000000000000000)
        self.tMaxSpin.setDecimals(1)
        self.tMaxSpin.setValue(800.000000000000000)

        self.rangeLayout.addWidget(self.tMaxSpin)

        self.spacerRangeStretch = QSpacerItem(40, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.rangeLayout.addItem(self.spacerRangeStretch)


        self.rootLayout.addLayout(self.rangeLayout)

        self.mainLayout = QHBoxLayout()
        self.mainLayout.setObjectName(u"mainLayout")
        self.textBox = QPlainTextEdit(Form)
        self.textBox.setObjectName(u"textBox")
        self.textBox.setReadOnly(True)
        self.textBox.setMaximumBlockCount(2000)

        self.mainLayout.addWidget(self.textBox)

        self.plotContainer = QWidget(Form)
        self.plotContainer.setObjectName(u"plotContainer")
        self.plotLayout = QVBoxLayout(self.plotContainer)
        self.plotLayout.setObjectName(u"plotLayout")
        self.plotLayout.setContentsMargins(0, 0, 0, 0)

        self.mainLayout.addWidget(self.plotContainer)


        self.rootLayout.addLayout(self.mainLayout)

        self.bottomLayout = QHBoxLayout()
        self.bottomLayout.setObjectName(u"bottomLayout")
        self.latestLayout = QHBoxLayout()
        self.latestLayout.setObjectName(u"latestLayout")
        self.labelLatest = QLabel(Form)
        self.labelLatest.setObjectName(u"labelLatest")

        self.latestLayout.addWidget(self.labelLatest)

        self.latestLab = QLabel(Form)
        self.latestLab.setObjectName(u"latestLab")

        self.latestLayout.addWidget(self.latestLab)

        self.rawLab = QLabel(Form)
        self.rawLab.setObjectName(u"rawLab")

        self.latestLayout.addWidget(self.rawLab)


        self.bottomLayout.addLayout(self.latestLayout)

        self.spacerBottomGap = QSpacerItem(50, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.bottomLayout.addItem(self.spacerBottomGap)

        self.peakLayout = QHBoxLayout()
        self.peakLayout.setObjectName(u"peakLayout")
        self.labelCPeak = QLabel(Form)
        self.labelCPeak.setObjectName(u"labelCPeak")

        self.peakLayout.addWidget(self.labelCPeak)

        self.cPeakLab = QLabel(Form)
        self.cPeakLab.setObjectName(u"cPeakLab")

        self.peakLayout.addWidget(self.cPeakLab)

        self.spacerPeakGap1 = QSpacerItem(20, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.peakLayout.addItem(self.spacerPeakGap1)

        self.labelTPeak = QLabel(Form)
        self.labelTPeak.setObjectName(u"labelTPeak")

        self.peakLayout.addWidget(self.labelTPeak)

        self.tPeakLab = QLabel(Form)
        self.tPeakLab.setObjectName(u"tPeakLab")

        self.peakLayout.addWidget(self.tPeakLab)

        self.spacerPeakGap2 = QSpacerItem(20, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.peakLayout.addItem(self.spacerPeakGap2)

        self.labelRatio = QLabel(Form)
        self.labelRatio.setObjectName(u"labelRatio")

        self.peakLayout.addWidget(self.labelRatio)

        self.ratioLab = QLabel(Form)
        self.ratioLab.setObjectName(u"ratioLab")

        self.peakLayout.addWidget(self.ratioLab)


        self.bottomLayout.addLayout(self.peakLayout)

        self.spacerBottomStretch = QSpacerItem(40, 10, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.bottomLayout.addItem(self.spacerBottomStretch)


        self.rootLayout.addLayout(self.bottomLayout)


        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"\u8367\u5149\u663e\u793a\u4eea\uff08C/T\u5cf0\u503c\u7248\uff09", None))
        self.labelPort.setText(QCoreApplication.translate("Form", u"\u4e32\u53e3:", None))
        self.refreshBtn.setText(QCoreApplication.translate("Form", u"\u5237\u65b0\u4e32\u53e3", None))
        self.labelBaud.setText(QCoreApplication.translate("Form", u"\u6ce2\u7279\u7387:", None))
        self.baudCB.setItemText(0, QCoreApplication.translate("Form", u"9600", None))
        self.baudCB.setItemText(1, QCoreApplication.translate("Form", u"19200", None))
        self.baudCB.setItemText(2, QCoreApplication.translate("Form", u"38400", None))
        self.baudCB.setItemText(3, QCoreApplication.translate("Form", u"57600", None))
        self.baudCB.setItemText(4, QCoreApplication.translate("Form", u"115200", None))

        self.reconnectChk.setText(QCoreApplication.translate("Form", u"\u81ea\u52a8\u91cd\u8fde", None))
        self.labelFps.setText(QCoreApplication.translate("Form", u"\u7ed8\u56feHz:", None))
        self.fpsCB.setItemText(0, QCoreApplication.translate("Form", u"10", None))
        self.fpsCB.setItemText(1, QCoreApplication.translate("Form", u"20", None))
        self.fpsCB.setItemText(2, QCoreApplication.translate("Form", u"30", None))

        self.fpsCB.setCurrentText(QCoreApplication.translate("Form", u"20", None))
        self.smoothChk.setText(QCoreApplication.translate("Form", u"\u5e73\u6ed1\u663e\u793a", None))
        self.startBtn.setText(QCoreApplication.translate("Form", u"\u5f00\u59cb", None))
        self.stopBtn.setText(QCoreApplication.translate("Form", u"\u505c\u6b62", None))
        self.clearBtn.setText(QCoreApplication.translate("Form", u"\u6e05\u7a7a", None))
        self.alarmEnableChk.setText(QCoreApplication.translate("Form", u"\u542f\u7528\u9608\u503c\u62a5\u8b66", None))
        self.labelTh.setText(QCoreApplication.translate("Form", u"\u9608\u503c:", None))
        self.beepChk.setText(QCoreApplication.translate("Form", u"\u8702\u9e23", None))
        self.exportBtn.setText(QCoreApplication.translate("Form", u"\u5bfc\u51faCSV", None))
        self.autoSaveChk.setText(QCoreApplication.translate("Form", u"\u81ea\u52a8\u6309\u5206\u949f\u5206\u6587\u4ef6", None))
        self.pickDirBtn.setText(QCoreApplication.translate("Form", u"\u9009\u62e9\u76ee\u5f55", None))
        self.dirLab.setText(QCoreApplication.translate("Form", u"\u672a\u9009\u62e9", None))
        self.dirLab.setStyleSheet(QCoreApplication.translate("Form", u"color: #888;", None))
        self.labelPeakParam.setText(QCoreApplication.translate("Form", u"\u5cf0\u503c\u8bc6\u522b\u53c2\u6570\uff1a", None))
        self.labelMinHeight.setText(QCoreApplication.translate("Form", u"\u6700\u5c0f\u9ad8\u5ea6:", None))
#if QT_CONFIG(tooltip)
        self.peakHeightSpin.setToolTip(QCoreApplication.translate("Form", u"\u6700\u5c0f\u5cf0\u503c\u9ad8\u5ea6\uff08\u8fc7\u6ee4\u566a\u58f0\u5cf0\uff09", None))
#endif // QT_CONFIG(tooltip)
        self.labelMinDist.setText(QCoreApplication.translate("Form", u"\u6700\u5c0f\u95f4\u9694:", None))
#if QT_CONFIG(tooltip)
        self.peakDistanceSpin.setToolTip(QCoreApplication.translate("Form", u"\u5cf0\u503c\u6700\u5c0f\u95f4\u9694\uff08\u786e\u4fddC/T\u5cf0\u76f8\u9694\u8f83\u8fdc\uff09", None))
#endif // QT_CONFIG(tooltip)
        self.labelCRange.setText(QCoreApplication.translate("Form", u"C\u7ebf\u8303\u56f4:", None))
        self.labelCMin.setText(QCoreApplication.translate("Form", u"\u6700\u5c0f:", None))
#if QT_CONFIG(tooltip)
        self.cMinSpin.setToolTip(QCoreApplication.translate("Form", u"C\u7ebf\u6700\u5c0f\u6a2a\u5750\u6807", None))
#endif // QT_CONFIG(tooltip)
        self.labelCMax.setText(QCoreApplication.translate("Form", u"\u6700\u5927:", None))
#if QT_CONFIG(tooltip)
        self.cMaxSpin.setToolTip(QCoreApplication.translate("Form", u"C\u7ebf\u6700\u5927\u6a2a\u5750\u6807", None))
#endif // QT_CONFIG(tooltip)
        self.labelTRange.setText(QCoreApplication.translate("Form", u"T\u7ebf\u8303\u56f4:", None))
        self.labelTMin.setText(QCoreApplication.translate("Form", u"\u6700\u5c0f:", None))
#if QT_CONFIG(tooltip)
        self.tMinSpin.setToolTip(QCoreApplication.translate("Form", u"T\u7ebf\u6700\u5c0f\u6a2a\u5750\u6807", None))
#endif // QT_CONFIG(tooltip)
        self.labelTMax.setText(QCoreApplication.translate("Form", u"\u6700\u5927:", None))
#if QT_CONFIG(tooltip)
        self.tMaxSpin.setToolTip(QCoreApplication.translate("Form", u"T\u7ebf\u6700\u5927\u6a2a\u5750\u6807", None))
#endif // QT_CONFIG(tooltip)
        self.textBox.setStyleSheet(QCoreApplication.translate("Form", u"QPlainTextEdit { background-color: #111; color: #0f0; border: 2px solid #244061; }", None))
        self.labelLatest.setText(QCoreApplication.translate("Form", u"\u6700\u65b0\u503c:", None))
        self.latestLab.setText(QCoreApplication.translate("Form", u"\u2014", None))
        self.rawLab.setText("")
        self.rawLab.setStyleSheet(QCoreApplication.translate("Form", u"color:#888;", None))
        self.labelCPeak.setText(QCoreApplication.translate("Form", u"C\u5cf0\u503c:", None))
        self.cPeakLab.setText(QCoreApplication.translate("Form", u"\u2014", None))
        self.cPeakLab.setStyleSheet(QCoreApplication.translate("Form", u"color: #00ff00;", None))
        self.labelTPeak.setText(QCoreApplication.translate("Form", u"T\u5cf0\u503c:", None))
        self.tPeakLab.setText(QCoreApplication.translate("Form", u"\u2014", None))
        self.tPeakLab.setStyleSheet(QCoreApplication.translate("Form", u"color: #ffff00;", None))
        self.labelRatio.setText(QCoreApplication.translate("Form", u"T/C\u6bd4\u503c:", None))
        self.ratioLab.setText(QCoreApplication.translate("Form", u"\u2014", None))
        self.ratioLab.setStyleSheet(QCoreApplication.translate("Form", u"color: #ff6600;", None))
    # retranslateUi

