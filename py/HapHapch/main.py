#!/env python
# -*- coding: utf-8 -*-

import sys

import cv2

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtWidgets import QMessageBox

import HandObject
import RobotControl
from CameraAnalysis import VideoThread
from Qt.PyQtWindow import Ui_MainWindow
from SettingClass import Settings


# from config.SettingClass import Settings

# pyuic5 -x PyQtWindow.ui -o PyQtWindow.py

class RobotWindow(Ui_MainWindow):
    def __init__(self):
        self.CameraThread = VideoThread()
        self.RobotThread = RobotControl.RobotObject()
        self.HandTracker = HandObject.HandTracker()
        self.Settings = Settings()
        self.HandExist = False

    def setupUi(self, MainWindow):
        # Код окна из Qt дизайнера
        super().setupUi(MainWindow)

        # Самая важная строчка
        MainWindow.setWindowIcon(QIcon("icon.jpg"))

        # Позволяет изображению деформироваться
        # self.Lab_Cam.setScaledContents(True)
        # Чёрный фон камеры
        self.Lab_Cam.setStyleSheet("background-color: black; color: rgb(255, 255, 255); font: 75 14pt 'Calibri'")

        '''
        # Ip по умолчанию
        HostP = self.RobotThread.Sender.get_Host()
        # settings = Settings('config/parameters.json')
        # self.IpLineEdit.setText(f"{settings.port}:{settings.host}")
        self.IpLineEdit.setText(HostP)
        '''

        # Настройка полей и ползунков
        # region
        self.CalibCam.valueChanged['int'].connect(self.Lab_CalibCam.setNum)
        self.CamAngle.valueChanged['int'].connect(self.Lab_CamAngle.setNum)
        self.FixedParam.valueChanged['int'].connect(self.Lab_FixedParam.setNum)
        self.TimeApprox.valueChanged['int'].connect(self.Lab_TimeApprox.setNum)
        self.FixedParam_Z.valueChanged['int'].connect(self.Lab_FixedParam_Z.setNum)
        self.TimeoutSlender.valueChanged['int'].connect(self.Lab_UpdateTime.setNum)

        self.Lab_CalibCam.setText(str(self.CalibCam.value()))
        self.Lab_CamAngle.setText(str(self.CamAngle.value()))
        self.Lab_FixedParam.setText(str(self.FixedParam.value()))
        self.Lab_TimeApprox.setText(str(self.TimeApprox.value()))
        self.Lab_FixedParam_Z.setText(str(self.FixedParam_Z.value()))
        self.Lab_UpdateTime.setText(str(self.TimeoutSlender.value()))

        self.CalibDist.editingFinished.connect(self.change_cam)
        self.CalibCam.valueChanged.connect(self.change_cam)
        self.CamAngle.valueChanged.connect(self.change_cam)
        self.FixedParam.valueChanged.connect(self.change_cam)
        self.TimeApprox.valueChanged.connect(self.change_cam)
        self.FixedParam_Z.valueChanged.connect(self.change_cam)
        self.CompressBox.stateChanged.connect(self.change_cam)
        self.TimeoutSlender.valueChanged.connect(self.change_cam)

        self.Start_cam.clicked.connect(self.new_Cam)
        self.StartServerBut.clicked.connect(self.RobotConnect)
        self.StartRobotBut.clicked.connect(self.RobotThread.RobotStart)
        self.PauseRobotBut.clicked.connect(self.RobotThread.stop)

        # Settings Button
        self.SaveSettingsBut.clicked.connect(self.SaveSettings)
        self.LoadSettingsBut.clicked.connect(self.LoadSettings)
        self.ResetSettingsBut.clicked.connect(self.ResetSettings)

        # connect its signal to the update_image slot
        self.CameraThread.change_pixmap_signal.connect(self.update_image)
        self.CameraThread.Cam_error_signal.connect(self.Cam_error)
        self.CameraThread.Hand_find.connect(self.Hand_update)
        self.RobotThread.GetHand.connect(self.HandToRobot)
        self.RobotThread.RobotMessage.connect(self.showDialog)
        self.RobotThread.SendListUpdate.connect(self.UpdateItem)
        # endregion

        # Попытка подключится к камере по умолчанию
        # self.new_Cam()
        # Обновление данных класса HandTracker
        self.change_cam()

        self.LoadSettings()

    def LoadSettings(self):
        Settings = self.Settings.get_settings()
        print(Settings)

    def ResetSettings(self):
        Settings = self.Settings.get_defaults_settings()
        print(Settings)

    def SaveSettings(self):
        Settings = {"host": '',
                    "port": 48569,
                    "timeout": 5}
        self.Settings.save_settings(Settings)

    def UpdateItem(self, Item):
        if self.SendList.count() > 50:
            self.SendList.takeItem(0)
        self.SendList.addItem(Item)
        self.SendList.scrollToBottom()

    def RobotConnect(self):
        port_host = self.IpLineEdit.text()
        if port_host.count(":"):
            self.RobotThread.RobotConnect(port_host)
        else:
            self.showDialog("Введены некорректные данные.")

    def HandToRobot(self):
        if self.HandTracker.TrackingProcess:
            CamInfo = self.HandTracker.get_Hand()
            if self.HandExist:
                self.RobotThread.SetHand(CamInfo)
        else:
            self.RobotThread.GoHome()

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
        self.HandTracker.ApproxCompress = bool(self.CompressBox.checkState())

        self.RobotThread.sleep_time = 1000 // int(self.TimeoutSlender.value())

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
        super().retranslateUi(MainWindow)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = RobotWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
