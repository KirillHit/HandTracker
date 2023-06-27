#!/env python
# -*- coding: utf-8 -*-

import sys

import cv2

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtWidgets import QMessageBox

import HandObject
import RobotControl
from CameraAnalysis import VideoThread
from config.SettingClass import Settings

# pyuic5 -x PyQtWindow.ui -o PyQtWindow.py

class Ui_MainWindow(object):
    def __init__(self):
        self.CameraThread = VideoThread()
        self.RobotThread = RobotControl.RobotObject()
        self.HandTracker = HandObject.HandTracker()
        self.HandExist = False

    def setupUi(self, MainWindow):
        # Код окна из PQ дизайнера
        # region
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(1200, 614)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.verticalFrame = QtWidgets.QFrame(self.centralwidget)
        self.verticalFrame.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.verticalFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.verticalFrame.setLineWidth(2)
        self.verticalFrame.setObjectName("verticalFrame")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.verticalFrame)
        self.verticalLayout_4.setContentsMargins(-1, 1, -1, 5)
        self.verticalLayout_4.setSpacing(2)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.frame = QtWidgets.QFrame(self.verticalFrame)
        self.frame.setFrameShape(QtWidgets.QFrame.Panel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setLineWidth(2)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setContentsMargins(-1, 5, -1, 5)
        self.verticalLayout.setSpacing(5)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_10 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_10.sizePolicy().hasHeightForWidth())
        self.label_10.setSizePolicy(sizePolicy)
        self.label_10.setStyleSheet("font: 75 12pt \"Calibri\";")
        self.label_10.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.label_10.setFrameShadow(QtWidgets.QFrame.Raised)
        self.label_10.setLineWidth(1)
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.verticalLayout.addWidget(self.label_10)
        self.frame1 = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame1.sizePolicy().hasHeightForWidth())
        self.frame1.setSizePolicy(sizePolicy)
        self.frame1.setObjectName("frame1")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.frame1)
        self.gridLayout_3.setContentsMargins(-1, 5, -1, 5)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.Start_cam = QtWidgets.QPushButton(self.frame1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Start_cam.sizePolicy().hasHeightForWidth())
        self.Start_cam.setSizePolicy(sizePolicy)
        self.Start_cam.setObjectName("Start_cam")
        self.gridLayout_3.addWidget(self.Start_cam, 0, 2, 1, 1)
        self.label = QtWidgets.QLabel(self.frame1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setAlignment(QtCore.Qt.AlignLeading | QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.label.setObjectName("label")
        self.gridLayout_3.addWidget(self.label, 0, 0, 1, 1)
        self.NumCamEditLine = QtWidgets.QLineEdit(self.frame1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(100)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.NumCamEditLine.sizePolicy().hasHeightForWidth())
        self.NumCamEditLine.setSizePolicy(sizePolicy)
        self.NumCamEditLine.setMinimumSize(QtCore.QSize(0, 0))
        self.NumCamEditLine.setMaximumSize(QtCore.QSize(50, 16777215))
        self.NumCamEditLine.setBaseSize(QtCore.QSize(0, 0))
        self.NumCamEditLine.setObjectName("NumCamEditLine")
        self.gridLayout_3.addWidget(self.NumCamEditLine, 0, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.frame1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setText("")
        self.label_2.setObjectName("label_2")
        self.gridLayout_3.addWidget(self.label_2, 0, 3, 1, 1)
        self.verticalLayout.addWidget(self.frame1)
        self.horizontalFrame_2 = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalFrame_2.sizePolicy().hasHeightForWidth())
        self.horizontalFrame_2.setSizePolicy(sizePolicy)
        self.horizontalFrame_2.setObjectName("horizontalFrame_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalFrame_2)
        self.horizontalLayout_2.setContentsMargins(-1, 5, -1, 5)
        self.horizontalLayout_2.setSpacing(5)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.lab_2 = QtWidgets.QLabel(self.horizontalFrame_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lab_2.sizePolicy().hasHeightForWidth())
        self.lab_2.setSizePolicy(sizePolicy)
        self.lab_2.setObjectName("lab_2")
        self.horizontalLayout_2.addWidget(self.lab_2)
        self.CalibDist = QtWidgets.QLineEdit(self.horizontalFrame_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.CalibDist.sizePolicy().hasHeightForWidth())
        self.CalibDist.setSizePolicy(sizePolicy)
        self.CalibDist.setMaximumSize(QtCore.QSize(50, 16777215))
        self.CalibDist.setObjectName("CalibDist")
        self.horizontalLayout_2.addWidget(self.CalibDist)
        self.label_3 = QtWidgets.QLabel(self.horizontalFrame_2)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_2.addWidget(self.label_3)
        self.label_4 = QtWidgets.QLabel(self.horizontalFrame_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setText("")
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_2.addWidget(self.label_4)
        self.horizontalLayout_2.setStretch(0, 1)
        self.horizontalLayout_2.setStretch(1, 1)
        self.horizontalLayout_2.setStretch(2, 1)
        self.horizontalLayout_2.setStretch(3, 10)
        self.verticalLayout.addWidget(self.horizontalFrame_2)
        self.line = QtWidgets.QFrame(self.frame)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout.addWidget(self.line)
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setContentsMargins(-1, 7, -1, 7)
        self.gridLayout_4.setHorizontalSpacing(7)
        self.gridLayout_4.setVerticalSpacing(10)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.TimeApprox = QtWidgets.QSlider(self.frame)
        self.TimeApprox.setMaximum(2000)
        self.TimeApprox.setPageStep(1)
        self.TimeApprox.setProperty("value", 500)
        self.TimeApprox.setOrientation(QtCore.Qt.Horizontal)
        self.TimeApprox.setObjectName("TimeApprox")
        self.gridLayout_4.addWidget(self.TimeApprox, 2, 1, 1, 1)
        self.CalibCam = QtWidgets.QSlider(self.frame)
        self.CalibCam.setMaximum(500)
        self.CalibCam.setPageStep(1)
        self.CalibCam.setProperty("value", 80)
        self.CalibCam.setOrientation(QtCore.Qt.Horizontal)
        self.CalibCam.setObjectName("CalibCam")
        self.gridLayout_4.addWidget(self.CalibCam, 0, 1, 1, 1)
        self.Lab_TimeApprox = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_TimeApprox.sizePolicy().hasHeightForWidth())
        self.Lab_TimeApprox.setSizePolicy(sizePolicy)
        self.Lab_TimeApprox.setObjectName("Lab_TimeApprox")
        self.gridLayout_4.addWidget(self.Lab_TimeApprox, 2, 2, 1, 1)
        self.FixedParam = QtWidgets.QSlider(self.frame)
        self.FixedParam.setMaximum(20)
        self.FixedParam.setPageStep(1)
        self.FixedParam.setProperty("value", 3)
        self.FixedParam.setOrientation(QtCore.Qt.Horizontal)
        self.FixedParam.setObjectName("FixedParam")
        self.gridLayout_4.addWidget(self.FixedParam, 3, 1, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.frame)
        self.label_8.setObjectName("label_8")
        self.gridLayout_4.addWidget(self.label_8, 4, 0, 1, 1)
        self.Lab_CamAngle = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_CamAngle.sizePolicy().hasHeightForWidth())
        self.Lab_CamAngle.setSizePolicy(sizePolicy)
        self.Lab_CamAngle.setObjectName("Lab_CamAngle")
        self.gridLayout_4.addWidget(self.Lab_CamAngle, 1, 2, 1, 1)
        self.Lab_FixedParam = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_FixedParam.sizePolicy().hasHeightForWidth())
        self.Lab_FixedParam.setSizePolicy(sizePolicy)
        self.Lab_FixedParam.setObjectName("Lab_FixedParam")
        self.gridLayout_4.addWidget(self.Lab_FixedParam, 3, 2, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.frame)
        self.label_5.setObjectName("label_5")
        self.gridLayout_4.addWidget(self.label_5, 1, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.frame)
        self.label_6.setObjectName("label_6")
        self.gridLayout_4.addWidget(self.label_6, 2, 0, 1, 1)
        self.Lab_FixedParam_Z = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_FixedParam_Z.sizePolicy().hasHeightForWidth())
        self.Lab_FixedParam_Z.setSizePolicy(sizePolicy)
        self.Lab_FixedParam_Z.setObjectName("Lab_FixedParam_Z")
        self.gridLayout_4.addWidget(self.Lab_FixedParam_Z, 4, 2, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.frame)
        self.label_7.setObjectName("label_7")
        self.gridLayout_4.addWidget(self.label_7, 3, 0, 1, 1)
        self.CamAngle = QtWidgets.QSlider(self.frame)
        self.CamAngle.setMaximum(180)
        self.CamAngle.setPageStep(1)
        self.CamAngle.setProperty("value", 0)
        self.CamAngle.setOrientation(QtCore.Qt.Horizontal)
        self.CamAngle.setObjectName("CamAngle")
        self.gridLayout_4.addWidget(self.CamAngle, 1, 1, 1, 1)
        self.Lab_CalibCam = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_CalibCam.sizePolicy().hasHeightForWidth())
        self.Lab_CalibCam.setSizePolicy(sizePolicy)
        self.Lab_CalibCam.setMinimumSize(QtCore.QSize(40, 0))
        self.Lab_CalibCam.setObjectName("Lab_CalibCam")
        self.gridLayout_4.addWidget(self.Lab_CalibCam, 0, 2, 1, 1)
        self.lab = QtWidgets.QLabel(self.frame)
        self.lab.setObjectName("lab")
        self.gridLayout_4.addWidget(self.lab, 0, 0, 1, 1)
        self.FixedParam_Z = QtWidgets.QSlider(self.frame)
        self.FixedParam_Z.setMaximum(20)
        self.FixedParam_Z.setPageStep(1)
        self.FixedParam_Z.setProperty("value", 4)
        self.FixedParam_Z.setOrientation(QtCore.Qt.Horizontal)
        self.FixedParam_Z.setObjectName("FixedParam_Z")
        self.gridLayout_4.addWidget(self.FixedParam_Z, 4, 1, 1, 1)
        self.gridLayout_4.setColumnStretch(0, 1)
        self.gridLayout_4.setColumnStretch(1, 10)
        self.gridLayout_4.setColumnStretch(2, 2)
        self.verticalLayout.addLayout(self.gridLayout_4)
        self.line_2 = QtWidgets.QFrame(self.frame)
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.verticalLayout.addWidget(self.line_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setContentsMargins(-1, 5, -1, 5)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_11 = QtWidgets.QLabel(self.frame)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_3.addWidget(self.label_11)
        self.Hand_Coords = QtWidgets.QLabel(self.frame)
        self.Hand_Coords.setStyleSheet("font: 75 14pt \"Calibri\";\n"
                                       "color: rgb(0, 170, 0);")
        self.Hand_Coords.setObjectName("Hand_Coords")
        self.horizontalLayout_3.addWidget(self.Hand_Coords)
        self.label_13 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_13.sizePolicy().hasHeightForWidth())
        self.label_13.setSizePolicy(sizePolicy)
        self.label_13.setText("")
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_3.addWidget(self.label_13)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.verticalLayout.addLayout(self.verticalLayout_2)
        self.label_9 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_9.sizePolicy().hasHeightForWidth())
        self.label_9.setSizePolicy(sizePolicy)
        self.label_9.setStyleSheet("font: 75 12pt \"Calibri\";")
        self.label_9.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.label_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.verticalLayout.addWidget(self.label_9)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_12 = QtWidgets.QLabel(self.frame)
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_4.addWidget(self.label_12)
        self.IpLineEdit = QtWidgets.QLineEdit(self.frame)
        self.IpLineEdit.setMinimumSize(QtCore.QSize(140, 0))
        self.IpLineEdit.setMaximumSize(QtCore.QSize(140, 16777215))
        font = QtGui.QFont()
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        self.IpLineEdit.setFont(font)
        self.IpLineEdit.setInputMethodHints(QtCore.Qt.ImhNone)
        self.IpLineEdit.setObjectName("IpLineEdit")
        self.horizontalLayout_4.addWidget(self.IpLineEdit)
        self.label_14 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_14.sizePolicy().hasHeightForWidth())
        self.label_14.setSizePolicy(sizePolicy)
        self.label_14.setText("")
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_4.addWidget(self.label_14)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.ConnectBut = QtWidgets.QPushButton(self.frame)
        self.ConnectBut.setMinimumSize(QtCore.QSize(140, 0))
        self.ConnectBut.setObjectName("ConnectBut")
        self.horizontalLayout_10.addWidget(self.ConnectBut)
        self.label_21 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_21.sizePolicy().hasHeightForWidth())
        self.label_21.setSizePolicy(sizePolicy)
        self.label_21.setText("")
        self.label_21.setObjectName("label_21")
        self.horizontalLayout_10.addWidget(self.label_21)
        self.verticalLayout.addLayout(self.horizontalLayout_10)
        self.line_4 = QtWidgets.QFrame(self.frame)
        self.line_4.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_4.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_4.setObjectName("line_4")
        self.verticalLayout.addWidget(self.line_4)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.StartRobotBut = QtWidgets.QPushButton(self.frame)
        self.StartRobotBut.setObjectName("StartRobotBut")
        self.horizontalLayout_9.addWidget(self.StartRobotBut)
        self.PauseRobotBut = QtWidgets.QPushButton(self.frame)
        self.PauseRobotBut.setObjectName("PauseRobotBut")
        self.horizontalLayout_9.addWidget(self.PauseRobotBut)
        self.label_19 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_19.sizePolicy().hasHeightForWidth())
        self.label_19.setSizePolicy(sizePolicy)
        self.label_19.setText("")
        self.label_19.setObjectName("label_19")
        self.horizontalLayout_9.addWidget(self.label_19)
        self.verticalLayout.addLayout(self.horizontalLayout_9)
        self.horizontalFrame = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalFrame.sizePolicy().hasHeightForWidth())
        self.horizontalFrame.setSizePolicy(sizePolicy)
        self.horizontalFrame.setObjectName("horizontalFrame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalFrame)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout.addWidget(self.horizontalFrame)
        self.horizontalLayout_5.addWidget(self.frame)
        self.Lab_Cam = QtWidgets.QLabel(self.verticalFrame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_Cam.sizePolicy().hasHeightForWidth())
        self.Lab_Cam.setSizePolicy(sizePolicy)
        self.Lab_Cam.setMinimumSize(QtCore.QSize(0, 0))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(240, 240, 240))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        self.Lab_Cam.setPalette(palette)
        self.Lab_Cam.setWhatsThis("")
        self.Lab_Cam.setAutoFillBackground(False)
        self.Lab_Cam.setStyleSheet("font: 75 14pt \"Calibri\"")
        self.Lab_Cam.setInputMethodHints(QtCore.Qt.ImhNone)
        self.Lab_Cam.setFrameShape(QtWidgets.QFrame.Panel)
        self.Lab_Cam.setFrameShadow(QtWidgets.QFrame.Raised)
        self.Lab_Cam.setLineWidth(2)
        self.Lab_Cam.setAlignment(QtCore.Qt.AlignCenter)
        self.Lab_Cam.setObjectName("Lab_Cam")
        self.horizontalLayout_5.addWidget(self.Lab_Cam)
        self.horizontalLayout_5.setStretch(0, 1)
        self.horizontalLayout_5.setStretch(1, 2)
        self.verticalLayout_4.addLayout(self.horizontalLayout_5)
        self.verticalLayout_4.setStretch(0, 10)
        self.gridLayout_6.addWidget(self.verticalFrame, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        # endregion

        # Самая важная строчка
        MainWindow.setWindowIcon(QIcon("icon.jpg"))

        # Позволяет изображению деформироваться
        # self.Lab_Cam.setScaledContents(True)
        # Чёрный фон камеры
        self.Lab_Cam.setStyleSheet("background-color: black; color: rgb(255, 255, 255); font: 75 14pt 'Calibri'")

        # Ip по умолчанию
        settings = Settings('config/parameters.json')
        self.IpLineEdit.setText(f"{settings.port}:{settings.host}")

        # Настройка полей и ползунков
        # region
        self.retranslateUi(MainWindow)
        self.CalibCam.valueChanged['int'].connect(self.Lab_CalibCam.setNum)
        self.CamAngle.valueChanged['int'].connect(self.Lab_CamAngle.setNum)
        self.FixedParam.valueChanged['int'].connect(self.Lab_FixedParam.setNum)
        self.TimeApprox.valueChanged['int'].connect(self.Lab_TimeApprox.setNum)
        self.FixedParam_Z.valueChanged['int'].connect(self.Lab_FixedParam_Z.setNum)

        self.Lab_CalibCam.setText(str(self.CalibCam.value()))
        self.Lab_CamAngle.setText(str(self.CamAngle.value()))
        self.Lab_FixedParam.setText(str(self.FixedParam.value()))
        self.Lab_TimeApprox.setText(str(self.TimeApprox.value()))
        self.Lab_FixedParam_Z.setText(str(self.FixedParam_Z.value()))

        # connect its signal to the update_image slot
        self.CameraThread.change_pixmap_signal.connect(self.update_image)
        self.CameraThread.Cam_error_signal.connect(self.Cam_error)
        self.CameraThread.Hand_find.connect(self.Hand_update)
        self.RobotThread.GetHand.connect(self.HandToRobot)
        self.RobotThread.RobotMessage.connect(self.showDialog)

        self.CalibDist.editingFinished.connect(self.change_cam)
        self.CalibCam.valueChanged.connect(self.change_cam)
        self.CamAngle.valueChanged.connect(self.change_cam)
        self.FixedParam.valueChanged.connect(self.change_cam)
        self.TimeApprox.valueChanged.connect(self.change_cam)
        self.FixedParam_Z.valueChanged.connect(self.change_cam)

        self.Start_cam.clicked.connect(self.new_Cam)
        self.ConnectBut.clicked.connect(self.RobotConnect)
        self.StartRobotBut.clicked.connect(self.RobotThread.RobotStart)
        self.PauseRobotBut.clicked.connect(self.RobotThread.stop)

        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        # endregion

        # Попытка подключится к камере по умолчанию
        # self.new_Cam()
        # Обновление данных класса HandTracker
        self.change_cam()

    def RobotConnect(self):
        self.RobotThread.RobotConnect(self.IpLineEdit.text())

    def HandToRobot(self):
        if self.HandTracker.TrackingProcess:
            CamInfo = self.HandTracker.get_Hand()
            if self.HandExist:
                self.RobotThread.SetHand(CamInfo)

    def new_Cam(self):
        if self.NumCamEditLine.text().isdigit():
            self.CameraThread.set_cam(int(self.NumCamEditLine.text()))
            self.HandTracker.width = self.CameraThread.width
            self.HandTracker.height = self.CameraThread.height
            self.RobotThread.GoHome()
        else:
            self.showDialog("Введите число большее и равное нулю")

    def change_cam(self):
        self.HandTracker.CalibDist = int(self.CalibDist.text())
        self.HandTracker.CalibCam = int(self.CalibCam.value())
        self.HandTracker.CamAngle = (int(self.CamAngle.value()) * 3.14) // 180
        self.HandTracker.FixedParam = int(self.FixedParam.value())
        self.HandTracker.TimeApprox = int(self.TimeApprox.value())
        self.HandTracker.LenApprox = 60 * int(self.TimeApprox.value()) // 1000
        self.HandTracker.FixedParam_Z = int(self.FixedParam_Z.value())

    def closeEvent(self, event):
        self.CameraThread.stop()
        event.accept()

    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        self.Lab_Cam.setPixmap(self.convert_cv_qt(cv_img))

    def Hand_update(self, Cordinate, SizeFactor, HandExist, Compress):
        if HandExist:
            self.Hand_Coords.setText(self.HandTracker.give_Hand(Cordinate, SizeFactor, Compress))
            if not self.HandExist:
                self.HandExist = True
        elif self.HandExist:
            self.Hand_Coords.setText("No hand")
            self.HandExist = False

    def Cam_error(self, Current_cam):
        self.Lab_Cam.setText(f"Камера {Current_cam} не найдена")
        self.Hand_Coords.setText("No hand")
        self.showDialog("Камера не найдена.\n"
                        "Всем подключённым камерам присваиваются номера от нуля и далее по возрастанию. \n"
                        "0 - по умолчанию веб-камера."
                        "* Иногда номера могут пропускаться")

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.Lab_Cam.size().width(), self.Lab_Cam.size().height(), Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def showDialog(self, text):
        msgBox = QMessageBox()
        msgBox.setWindowTitle("HandTracker")
        msgBox.setIcon(QMessageBox.Information)
        msgBox.setText(text)
        msgBox.setStandardButtons(QMessageBox.Ok)
        msgBox.exec()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "HandTracker"))
        self.label_10.setText(_translate("MainWindow", "Интерфейс управления камерой"))
        self.Start_cam.setText(_translate("MainWindow", "Обновить"))
        self.label.setText(_translate("MainWindow", "Номер камеры:"))
        self.NumCamEditLine.setText(_translate("MainWindow", "0"))
        self.lab_2.setText(_translate("MainWindow", "Дистанция калибровки"))
        self.CalibDist.setText(_translate("MainWindow", "500"))
        self.label_3.setText(_translate("MainWindow", "мм"))
        self.Lab_TimeApprox.setText(_translate("MainWindow", "0"))
        self.label_8.setText(_translate("MainWindow", "Порог движения Z"))
        self.Lab_CamAngle.setText(_translate("MainWindow", "0"))
        self.Lab_FixedParam.setText(_translate("MainWindow", "0"))
        self.label_5.setText(_translate("MainWindow", "Наклон камеры"))
        self.label_6.setText(_translate("MainWindow", "Время усреднения (мс)"))
        self.Lab_FixedParam_Z.setText(_translate("MainWindow", "0"))
        self.label_7.setText(_translate("MainWindow", "Порог движения"))
        self.Lab_CalibCam.setText(_translate("MainWindow", "0"))
        self.lab.setText(_translate("MainWindow", "Эквивалент камеры"))
        self.label_11.setText(_translate("MainWindow", "Координаты руки:"))
        self.Hand_Coords.setText(_translate("MainWindow", "No hands"))
        self.label_9.setText(_translate("MainWindow", "Интерфейс робота"))
        self.label_12.setText(_translate("MainWindow", "Хост и порт:"))
        self.IpLineEdit.setText(_translate("MainWindow", "192.168.0.10:48569"))
        self.ConnectBut.setText(_translate("MainWindow", "Подключиться"))
        self.StartRobotBut.setText(_translate("MainWindow", "Начать управление"))
        self.PauseRobotBut.setText(_translate("MainWindow", "Остановить"))
        self.Lab_Cam.setText(_translate("MainWindow", "<html><head/><body><p>Камеры нет</p></body></html>"))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
