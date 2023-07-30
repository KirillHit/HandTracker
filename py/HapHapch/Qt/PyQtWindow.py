# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'PyQtWindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(1200, 645)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_6.setContentsMargins(5, 5, 5, 5)
        self.gridLayout_6.setSpacing(5)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setSpacing(5)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame.sizePolicy().hasHeightForWidth())
        self.frame.setSizePolicy(sizePolicy)
        self.frame.setMinimumSize(QtCore.QSize(0, 0))
        self.frame.setMaximumSize(QtCore.QSize(600, 16777215))
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
        self.label.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
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
        self.CalibDist.setText("")
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
        self.CalibCam = QtWidgets.QSlider(self.frame)
        self.CalibCam.setMaximum(300)
        self.CalibCam.setPageStep(1)
        self.CalibCam.setProperty("value", 0)
        self.CalibCam.setOrientation(QtCore.Qt.Horizontal)
        self.CalibCam.setObjectName("CalibCam")
        self.gridLayout_4.addWidget(self.CalibCam, 0, 1, 1, 1)
        self.FixedParam = QtWidgets.QSlider(self.frame)
        self.FixedParam.setMaximum(20)
        self.FixedParam.setPageStep(1)
        self.FixedParam.setOrientation(QtCore.Qt.Horizontal)
        self.FixedParam.setObjectName("FixedParam")
        self.gridLayout_4.addWidget(self.FixedParam, 2, 1, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.frame)
        self.label_8.setObjectName("label_8")
        self.gridLayout_4.addWidget(self.label_8, 3, 0, 1, 1)
        self.TimeApprox = QtWidgets.QSlider(self.frame)
        self.TimeApprox.setMaximum(2000)
        self.TimeApprox.setPageStep(1)
        self.TimeApprox.setProperty("value", 0)
        self.TimeApprox.setOrientation(QtCore.Qt.Horizontal)
        self.TimeApprox.setObjectName("TimeApprox")
        self.gridLayout_4.addWidget(self.TimeApprox, 1, 1, 1, 1)
        self.Lab_FixedParam_Z = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_FixedParam_Z.sizePolicy().hasHeightForWidth())
        self.Lab_FixedParam_Z.setSizePolicy(sizePolicy)
        self.Lab_FixedParam_Z.setObjectName("Lab_FixedParam_Z")
        self.gridLayout_4.addWidget(self.Lab_FixedParam_Z, 3, 2, 1, 1)
        self.Lab_TimeApprox = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_TimeApprox.sizePolicy().hasHeightForWidth())
        self.Lab_TimeApprox.setSizePolicy(sizePolicy)
        self.Lab_TimeApprox.setObjectName("Lab_TimeApprox")
        self.gridLayout_4.addWidget(self.Lab_TimeApprox, 1, 2, 1, 1)
        self.Lab_CalibCam = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_CalibCam.sizePolicy().hasHeightForWidth())
        self.Lab_CalibCam.setSizePolicy(sizePolicy)
        self.Lab_CalibCam.setMinimumSize(QtCore.QSize(40, 0))
        self.Lab_CalibCam.setObjectName("Lab_CalibCam")
        self.gridLayout_4.addWidget(self.Lab_CalibCam, 0, 2, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.frame)
        self.label_6.setObjectName("label_6")
        self.gridLayout_4.addWidget(self.label_6, 1, 0, 1, 1)
        self.Lab_FixedParam = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_FixedParam.sizePolicy().hasHeightForWidth())
        self.Lab_FixedParam.setSizePolicy(sizePolicy)
        self.Lab_FixedParam.setObjectName("Lab_FixedParam")
        self.gridLayout_4.addWidget(self.Lab_FixedParam, 2, 2, 1, 1)
        self.FixedParam_Z = QtWidgets.QSlider(self.frame)
        self.FixedParam_Z.setMaximum(20)
        self.FixedParam_Z.setPageStep(1)
        self.FixedParam_Z.setOrientation(QtCore.Qt.Horizontal)
        self.FixedParam_Z.setObjectName("FixedParam_Z")
        self.gridLayout_4.addWidget(self.FixedParam_Z, 3, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.frame)
        self.label_7.setObjectName("label_7")
        self.gridLayout_4.addWidget(self.label_7, 2, 0, 1, 1)
        self.lab = QtWidgets.QLabel(self.frame)
        self.lab.setObjectName("lab")
        self.gridLayout_4.addWidget(self.lab, 0, 0, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_4)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.SaveSettingsBut = QtWidgets.QPushButton(self.frame)
        self.SaveSettingsBut.setObjectName("SaveSettingsBut")
        self.horizontalLayout_6.addWidget(self.SaveSettingsBut)
        self.LoadSettingsBut = QtWidgets.QPushButton(self.frame)
        self.LoadSettingsBut.setObjectName("LoadSettingsBut")
        self.horizontalLayout_6.addWidget(self.LoadSettingsBut)
        self.ResetSettingsBut = QtWidgets.QPushButton(self.frame)
        self.ResetSettingsBut.setObjectName("ResetSettingsBut")
        self.horizontalLayout_6.addWidget(self.ResetSettingsBut)
        self.verticalLayout.addLayout(self.horizontalLayout_6)
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
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Hand_Coords.sizePolicy().hasHeightForWidth())
        self.Hand_Coords.setSizePolicy(sizePolicy)
        self.Hand_Coords.setMinimumSize(QtCore.QSize(300, 0))
        self.Hand_Coords.setStyleSheet("font: 75 12pt \"Calibri\";\n"
"color: rgb(0, 170, 0);")
        self.Hand_Coords.setObjectName("Hand_Coords")
        self.horizontalLayout_3.addWidget(self.Hand_Coords)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.line_4 = QtWidgets.QFrame(self.frame)
        self.line_4.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_4.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_4.setObjectName("line_4")
        self.verticalLayout.addWidget(self.line_4)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_9 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_9.sizePolicy().hasHeightForWidth())
        self.label_9.setSizePolicy(sizePolicy)
        self.label_9.setStyleSheet("font: 75 12pt \"Calibri\";")
        self.label_9.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.label_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.verticalLayout_3.addWidget(self.label_9)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_12 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_12.sizePolicy().hasHeightForWidth())
        self.label_12.setSizePolicy(sizePolicy)
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_4.addWidget(self.label_12)
        self.IpLineEdit = QtWidgets.QLineEdit(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.IpLineEdit.sizePolicy().hasHeightForWidth())
        self.IpLineEdit.setSizePolicy(sizePolicy)
        self.IpLineEdit.setMinimumSize(QtCore.QSize(0, 0))
        self.IpLineEdit.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        self.IpLineEdit.setFont(font)
        self.IpLineEdit.setInputMethodHints(QtCore.Qt.ImhNone)
        self.IpLineEdit.setText("")
        self.IpLineEdit.setObjectName("IpLineEdit")
        self.horizontalLayout_4.addWidget(self.IpLineEdit)
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.StartServerBut = QtWidgets.QPushButton(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.StartServerBut.sizePolicy().hasHeightForWidth())
        self.StartServerBut.setSizePolicy(sizePolicy)
        self.StartServerBut.setMinimumSize(QtCore.QSize(0, 0))
        self.StartServerBut.setObjectName("StartServerBut")
        self.horizontalLayout_7.addWidget(self.StartServerBut)
        self.PauseRobotBut = QtWidgets.QPushButton(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.PauseRobotBut.sizePolicy().hasHeightForWidth())
        self.PauseRobotBut.setSizePolicy(sizePolicy)
        self.PauseRobotBut.setObjectName("PauseRobotBut")
        self.horizontalLayout_7.addWidget(self.PauseRobotBut)
        self.verticalLayout_3.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setContentsMargins(-1, -1, 0, -1)
        self.horizontalLayout_10.setSpacing(10)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_15 = QtWidgets.QLabel(self.frame)
        self.label_15.setObjectName("label_15")
        self.horizontalLayout_10.addWidget(self.label_15)
        self.FrequencySlender = QtWidgets.QSpinBox(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.FrequencySlender.sizePolicy().hasHeightForWidth())
        self.FrequencySlender.setSizePolicy(sizePolicy)
        self.FrequencySlender.setMinimum(1)
        self.FrequencySlender.setProperty("value", 30)
        self.FrequencySlender.setObjectName("FrequencySlender")
        self.horizontalLayout_10.addWidget(self.FrequencySlender)
        self.label_5 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setText("")
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_10.addWidget(self.label_5)
        self.horizontalLayout_10.setStretch(0, 1)
        self.verticalLayout_3.addLayout(self.horizontalLayout_10)
        self.horizontalLayout_8.addLayout(self.verticalLayout_3)
        self.line_6 = QtWidgets.QFrame(self.frame)
        self.line_6.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_6.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_6.setObjectName("line_6")
        self.horizontalLayout_8.addWidget(self.line_6)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_14 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_14.sizePolicy().hasHeightForWidth())
        self.label_14.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(9)
        self.label_14.setFont(font)
        self.label_14.setStyleSheet("font: 75 12pt \"Calibri\";")
        self.label_14.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.label_14.setFrameShadow(QtWidgets.QFrame.Raised)
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.verticalLayout_2.addWidget(self.label_14)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.StartGameBut = QtWidgets.QPushButton(self.frame)
        self.StartGameBut.setMinimumSize(QtCore.QSize(0, 0))
        self.StartGameBut.setObjectName("StartGameBut")
        self.horizontalLayout_9.addWidget(self.StartGameBut)
        self.StopGameBut = QtWidgets.QPushButton(self.frame)
        self.StopGameBut.setMinimumSize(QtCore.QSize(0, 0))
        self.StopGameBut.setObjectName("StopGameBut")
        self.horizontalLayout_9.addWidget(self.StopGameBut)
        self.verticalLayout_2.addLayout(self.horizontalLayout_9)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_17 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_17.sizePolicy().hasHeightForWidth())
        self.label_17.setSizePolicy(sizePolicy)
        self.label_17.setObjectName("label_17")
        self.horizontalLayout.addWidget(self.label_17)
        self.timeEdit = QtWidgets.QTimeEdit(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.timeEdit.sizePolicy().hasHeightForWidth())
        self.timeEdit.setSizePolicy(sizePolicy)
        self.timeEdit.setObjectName("timeEdit")
        self.horizontalLayout.addWidget(self.timeEdit)
        self.label_19 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_19.sizePolicy().hasHeightForWidth())
        self.label_19.setSizePolicy(sizePolicy)
        self.label_19.setText("")
        self.label_19.setObjectName("label_19")
        self.horizontalLayout.addWidget(self.label_19)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_18 = QtWidgets.QLabel(self.frame)
        self.label_18.setObjectName("label_18")
        self.horizontalLayout_11.addWidget(self.label_18)
        self.Add_timeEdit = QtWidgets.QTimeEdit(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Add_timeEdit.sizePolicy().hasHeightForWidth())
        self.Add_timeEdit.setSizePolicy(sizePolicy)
        self.Add_timeEdit.setObjectName("Add_timeEdit")
        self.horizontalLayout_11.addWidget(self.Add_timeEdit)
        self.AddTime_checkBox = QtWidgets.QCheckBox(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.AddTime_checkBox.sizePolicy().hasHeightForWidth())
        self.AddTime_checkBox.setSizePolicy(sizePolicy)
        self.AddTime_checkBox.setText("")
        self.AddTime_checkBox.setObjectName("AddTime_checkBox")
        self.horizontalLayout_11.addWidget(self.AddTime_checkBox)
        self.verticalLayout_2.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_22 = QtWidgets.QLabel(self.frame)
        self.label_22.setObjectName("label_22")
        self.horizontalLayout_12.addWidget(self.label_22)
        self.lab_game_timeout = QtWidgets.QLabel(self.frame)
        self.lab_game_timeout.setMinimumSize(QtCore.QSize(30, 0))
        self.lab_game_timeout.setStyleSheet("font: 12pt \"Calibri\";\n"
"color: rgb(0, 170, 0);")
        self.lab_game_timeout.setObjectName("lab_game_timeout")
        self.horizontalLayout_12.addWidget(self.lab_game_timeout)
        self.label_23 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_23.sizePolicy().hasHeightForWidth())
        self.label_23.setSizePolicy(sizePolicy)
        self.label_23.setText("")
        self.label_23.setObjectName("label_23")
        self.horizontalLayout_12.addWidget(self.label_23)
        self.verticalLayout_2.addLayout(self.horizontalLayout_12)
        self.horizontalLayout_8.addLayout(self.verticalLayout_2)
        self.horizontalLayout_8.setStretch(0, 1)
        self.horizontalLayout_8.setStretch(2, 1)
        self.verticalLayout.addLayout(self.horizontalLayout_8)
        self.line_3 = QtWidgets.QFrame(self.frame)
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.verticalLayout.addWidget(self.line_3)
        self.SendList = QtWidgets.QListWidget(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.SendList.sizePolicy().hasHeightForWidth())
        self.SendList.setSizePolicy(sizePolicy)
        self.SendList.setMinimumSize(QtCore.QSize(0, 40))
        self.SendList.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.SendList.setStyleSheet("")
        self.SendList.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.SendList.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.SendList.setAutoScrollMargin(16)
        self.SendList.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.SendList.setProperty("showDropIndicator", True)
        self.SendList.setMovement(QtWidgets.QListView.Static)
        self.SendList.setModelColumn(0)
        self.SendList.setUniformItemSizes(False)
        self.SendList.setBatchSize(100)
        self.SendList.setWordWrap(True)
        self.SendList.setSelectionRectVisible(False)
        self.SendList.setObjectName("SendList")
        self.verticalLayout.addWidget(self.SendList)
        self.horizontalLayout_5.addWidget(self.frame)
        self.Lab_Cam = QtWidgets.QLabel(self.centralwidget)
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
        self.horizontalLayout_5.setStretch(0, 7)
        self.horizontalLayout_5.setStretch(1, 13)
        self.gridLayout_6.addLayout(self.horizontalLayout_5, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "CatControl"))
        self.label_10.setText(_translate("MainWindow", "Камера"))
        self.Start_cam.setText(_translate("MainWindow", "Обновить"))
        self.label.setText(_translate("MainWindow", "Номер камеры:"))
        self.NumCamEditLine.setText(_translate("MainWindow", "0"))
        self.lab_2.setText(_translate("MainWindow", "Дистанция калибровки"))
        self.label_3.setText(_translate("MainWindow", "мм"))
        self.label_8.setText(_translate("MainWindow", "Порог движения Z"))
        self.Lab_FixedParam_Z.setText(_translate("MainWindow", "0"))
        self.Lab_TimeApprox.setText(_translate("MainWindow", "0"))
        self.Lab_CalibCam.setText(_translate("MainWindow", "0"))
        self.label_6.setText(_translate("MainWindow", "Время усреднения (мс)"))
        self.Lab_FixedParam.setText(_translate("MainWindow", "0"))
        self.label_7.setText(_translate("MainWindow", "Порог движения"))
        self.lab.setText(_translate("MainWindow", "Эквивалент камеры"))
        self.SaveSettingsBut.setText(_translate("MainWindow", "Сохранить"))
        self.LoadSettingsBut.setText(_translate("MainWindow", "Загрузить"))
        self.ResetSettingsBut.setText(_translate("MainWindow", "Сбросить"))
        self.label_11.setText(_translate("MainWindow", "Координаты руки:"))
        self.Hand_Coords.setText(_translate("MainWindow", "No hands"))
        self.label_9.setText(_translate("MainWindow", "Робот"))
        self.label_12.setText(_translate("MainWindow", "Адрес:"))
        self.StartServerBut.setText(_translate("MainWindow", "Запустить"))
        self.PauseRobotBut.setText(_translate("MainWindow", "Остановить"))
        self.label_15.setText(_translate("MainWindow", "Частота обновления:"))
        self.label_14.setText(_translate("MainWindow", "Игра"))
        self.StartGameBut.setText(_translate("MainWindow", "Начать играть"))
        self.StopGameBut.setText(_translate("MainWindow", "Прервать игру"))
        self.label_17.setText(_translate("MainWindow", "Время игры:"))
        self.label_18.setText(_translate("MainWindow", "Бонусное время:"))
        self.label_22.setText(_translate("MainWindow", "До конца:"))
        self.lab_game_timeout.setText(_translate("MainWindow", "0:00"))
        self.Lab_Cam.setText(_translate("MainWindow", "<html><head/><body><p>Камеры нет</p></body></html>"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
