#!/env python
# -*- coding: utf-8 -*-

import sys
import numpy as np

import cv2

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMessageBox, QApplication
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QPixmap, QIcon, QImage
from Qt.PyQtWindow import Ui_MainWindow

import HandObject
import Robot
from CameraAnalysis import VideoThread
from SettingClass import Settings


class RobotWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        # Загрузка окна
        super().__init__()
        self.setupUi(self)

        # Подключение потоков
        self.CameraThread = VideoThread()
        self.RobotThread = Robot.Robot()

        # Контейнер для хранения положения руки
        self.HandTracker = HandObject.HandTracker()

        self.Settings = Settings()

        # Самая важная строчка
        self.setWindowIcon(QIcon("icon.jpg"))

        # Позволяет изображению деформироваться
        # self.Lab_Cam.setScaledContents(True)
        # Чёрный фон камеры
        self.Lab_Cam.setStyleSheet("background-color: black; color: rgb(255, 255, 255); font: 75 14pt 'Calibri'")

        # Настройка полей и ползунков
        # region
        self.CalibCam.valueChanged['int'].connect(lambda a: self.Lab_CalibCam.setNum(a/100))
        self.FixedParam.valueChanged['int'].connect(self.Lab_FixedParam.setNum)
        self.TimeApprox.valueChanged['int'].connect(self.Lab_TimeApprox.setNum)
        self.FixedParam_Z.valueChanged['int'].connect(self.Lab_FixedParam_Z.setNum)

        self.Lab_CalibCam.setText(str(self.CalibCam.value()))
        self.Lab_FixedParam.setText(str(self.FixedParam.value()))
        self.Lab_TimeApprox.setText(str(self.TimeApprox.value()))
        self.Lab_FixedParam_Z.setText(str(self.FixedParam_Z.value()))

        self.CalibDist.editingFinished.connect(self.change_cam)
        self.CalibCam.valueChanged.connect(self.change_cam)
        self.FixedParam.valueChanged.connect(self.change_cam)
        self.TimeApprox.valueChanged.connect(self.change_cam)
        self.FixedParam_Z.valueChanged.connect(self.change_cam)
        self.FrequencySlender.valueChanged.connect(self.change_cam)

        self.Start_cam.clicked.connect(self.new_Cam)
        self.StartServerBut.clicked.connect(self.StartServer)
        self.PauseRobotBut.clicked.connect(self.RobotThread.stop)

        # Settings Button
        self.SaveSettingsBut.clicked.connect(self.SaveSettings)
        self.LoadSettingsBut.clicked.connect(self.LoadSettings)
        self.ResetSettingsBut.clicked.connect(lambda: self.LoadSettings(defaults=True))

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
        # Загрузка настроек
        self.LoadSettings()
        # Обновление данных класса HandTracker
        self.change_cam()

    @pyqtSlot(bool)
    def LoadSettings(self, defaults=False):
        if defaults:
            Settings = self.Settings.get_defaults_settings()
        else:
            Settings = self.Settings.get_settings()

        try:
            self.CalibDist.setText(Settings["CalibDist"])
            self.CalibCam.setValue(Settings["CalibCam"])
            self.TimeApprox.setValue(Settings["TimeApprox"])
            self.FixedParam.setValue(Settings["FixedParam"])
            self.FixedParam_Z.setValue(Settings["FixedParam_Z"])
            self.IpLineEdit.setText(f"{Settings['host']}:{Settings['port']}")
            self.FrequencySlender.setValue(Settings["timeout"])
        except Exception as e:
            print("Not found:" + str(e))
            self.Settings.save_settings(self.Settings.get_defaults_settings())

    @pyqtSlot()
    def SaveSettings(self):
        Settings = {"CalibDist": self.CalibDist.text(),
                    "CalibCam": self.CalibCam.value(),
                    "TimeApprox": self.TimeApprox.value(),
                    "FixedParam": self.FixedParam.value(),
                    "FixedParam_Z": self.FixedParam_Z.value(),
                    "host": self.IpLineEdit.text().split(':')[0],
                    "port": self.IpLineEdit.text().split(':')[1],
                    "timeout": self.FrequencySlender.value()}

        self.Settings.save_settings(Settings)

    @pyqtSlot()
    def StartServer(self):
        port_host = self.IpLineEdit.text()
        try:
            port = port_host.split(':')[0]
            host = port_host.split(':')[1]
        except Exception:
            self.showDialog("Некорректный адрес: " + port_host)
            return
        timeout = self.FrequencySlender.value()
        self.RobotThread.start_server(port, host, timeout)

    @pyqtSlot()
    def new_Cam(self):
        if self.NumCamEditLine.text().isdigit():
            self.Start_cam.setEnabled(False)
            self.CameraThread.set_cam(int(self.NumCamEditLine.text()))
            self.HandTracker.width = self.CameraThread.width
            self.HandTracker.height = self.CameraThread.height
            QTimer.singleShot(100, lambda: self.Start_cam.setEnabled(True))
            self.RobotThread.GoHome()
        else:
            self.showDialog("Введите число большее и равное нулю")

    @pyqtSlot()
    def change_cam(self):
        self.HandTracker.CalibDist = int(self.CalibDist.text())
        self.HandTracker.CalibCam = int(self.CalibCam.value()) / 100
        self.HandTracker.FixedParam = int(self.FixedParam.value())
        self.HandTracker.TimeApprox = int(self.TimeApprox.value())
        self.HandTracker.LenApprox = 60 * int(self.TimeApprox.value()) // 1000
        self.HandTracker.FixedParam_Z = int(self.FixedParam_Z.value())

        self.RobotThread.sleep_time = 1000 // int(self.FrequencySlender.value())

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        self.Lab_Cam.setPixmap(self.convert_cv_qt(cv_img))

    @pyqtSlot(np.single, np.single, np.single, bool, bool)
    def Hand_update(self, x, y, size_factor, hand_exist, compress):
        if hand_exist:
            self.Hand_Coords.setText(self.HandTracker.give_Hand((x, y), size_factor, compress))
        else:
            self.Hand_Coords.setText("No hand")

    @pyqtSlot()
    def HandToRobot(self):
        if self.HandTracker.TrackingProcess:
            self.RobotThread.SetHand(self.HandTracker.get_Hand())
        else:
            self.RobotThread.GoHome()

    @pyqtSlot(int)
    def Cam_error(self, Current_cam):
        self.Lab_Cam.setText(f"Камера {Current_cam} не найдена")
        self.Hand_Coords.setText("No hand")
        self.showDialog("Камера не найдена.\n"
                        "Всем подключённым камерам присваиваются номера от нуля и далее по возрастанию. \n"
                        "0 - по умолчанию веб-камера."
                        "* Иногда номера могут пропускаться")

    @pyqtSlot(str)
    def showDialog(self, text):
        msgBox = QMessageBox()
        msgBox.setWindowTitle("HandTracker")
        msgBox.setIcon(QMessageBox.Information)
        msgBox.setText(text)
        msgBox.setStandardButtons(QMessageBox.Ok)
        msgBox.exec()

    @pyqtSlot(str)
    def UpdateItem(self, Item):
        if self.SendList.count() > 50:
            self.SendList.takeItem(0)
        self.SendList.addItem(Item)
        self.SendList.scrollToBottom()

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.Lab_Cam.size().width(), self.Lab_Cam.size().height(), Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def closeEvent(self, event):
        self.CameraThread.stop()
        self.RobotThread.stop()
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    ui = RobotWindow()
    ui.show()
    sys.exit(app.exec_())
