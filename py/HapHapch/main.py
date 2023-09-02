#!/env python
# -*- coding: utf-8 -*-

import sys
import time

import numpy as np

import cv2

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import Qt, QTimer, pyqtSlot, QTime
from PyQt5.QtGui import QPixmap, QIcon, QImage
from Qt.MainWindow.PyQtWindow import Ui_MainWindow
from GameWindow import GameWindow as GameWindow_ui

import HandObject
import Robot
from CameraAnalysis import VideoThread
from SettingClass import Settings


class RobotWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    end_delay = 2000
    calib_radius = 250

    def __init__(self):
        # Загрузка окна
        super().__init__()
        self.setupUi(self)

        # Подключение потоков
        self.CameraThread = VideoThread(radius=self.calib_radius)
        self.RobotThread = Robot.Robot()

        # Контейнер для хранения положения руки
        self.HandTracker = HandObject.HandTracker(radius=self.calib_radius)

        self.Settings = Settings()

        # Самая важная строчка
        self.setWindowIcon(QIcon("icon.jpg"))

        # Позволяет изображению деформироваться
        # self.Lab_Cam.setScaledContents(True)
        # Чёрный фон камеры
        self.Lab_Cam.setStyleSheet("background-color: black; color: rgb(255, 255, 255); font: 75 14pt 'Calibri'")

        self.GameWindow = GameWindow_ui()
        self.GameWindow.setWindowFlags(Qt.CustomizeWindowHint | Qt.WindowMinimizeButtonHint | Qt.WindowMaximizeButtonHint)
        self.GameWindow.show()
        self.GameWindow.resizeEvent(None)
        self.GameWindow.DoMes("stop")

        # Начало, остановка игры и таймеры
        # region
        self.StartGameBut.clicked.connect(lambda: self.GameFlag(True))
        self.StopGameBut.clicked.connect(lambda: self.GameFlag(False))

        self.timeEdit.timeChanged.connect(self.LabTimeUpdate)
        self.Add_timeEdit.timeChanged.connect(self.LabTimeUpdate)
        self.AddTime_checkBox.stateChanged.connect(self.LabTimeUpdate)

        self.game_timer = QTimer()
        self.game_timer.timeout.connect(self.StopGame)

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.UpdateTimer)
        self.update_timer.setInterval(100)
        # endregion

        # Настройка полей и ползунков
        # region
        self.CalibCam.valueChanged['int'].connect(lambda a: self.Lab_CalibCam.setNum(a / 100))
        self.CalibCam_Z.valueChanged['int'].connect(lambda a: self.Lab_CalibCam_Z.setNum(a / 100))
        self.TimeApprox.valueChanged['int'].connect(self.Lab_TimeApprox.setNum)

        self.Lab_CalibCam.setText(str(self.CalibCam.value()/100))
        self.Lab_CalibCam_Z.setText(str(self.CalibCam_Z.value()/100))
        self.Lab_TimeApprox.setText(str(self.TimeApprox.value()))

        self.CalibDist.editingFinished.connect(self.change_cam)
        self.CalibCam.valueChanged.connect(self.change_cam)
        self.CalibCam_Z.valueChanged.connect(self.change_cam)
        self.TimeApprox.valueChanged.connect(self.change_cam)
        self.FrequencySlender.valueChanged.connect(self.change_cam)
        self.Change_XY_Box.stateChanged.connect(self.change_cam)
        self.Inv_X_Box.stateChanged.connect(self.change_cam)
        self.Inv_Y_Box.stateChanged.connect(self.change_cam)

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
        # Обновление таймера
        self.LabTimeUpdate()

    @pyqtSlot()
    def LabTimeUpdate(self):
        if not self.HandTracker.TrackingProcess:
            if self.AddTime_checkBox.isChecked():
                time = self.Add_timeEdit.time()
            else:
                time = self.timeEdit.time()
            time = time.toString("mm:ss")
            self.lab_game_timeout.setText(time)
            self.GameWindow.set_time(time, self.GameWindow.progressBar.maximum())

    @pyqtSlot(bool)
    def GameFlag(self, game_flag):
        self.CameraThread.game_flag = game_flag
        self.game_timer.stop()
        self.update_timer.stop()
        self.HandTracker.stop_tracking()
        self.LabTimeUpdate()

        if game_flag:
            self.GameWindow.DoMes("calib")
        else:
            self.GameWindow.DoMes("stop")

    @pyqtSlot()
    def StartGame(self):
        if self.AddTime_checkBox.isChecked():
            time = self.Add_timeEdit.time()
        else:
            time = self.timeEdit.time()
        self.game_timer.start((time.minute() * 60 + time.second()) * 1000)
        self.update_timer.start()

        self.GameWindow.DoMes("start")

    @pyqtSlot()
    def StopGame(self):
        self.game_timer.stop()
        self.update_timer.stop()

        # Задержка после окончания времени
        self.CameraThread.game_flag = False
        QTimer.singleShot(self.end_delay, lambda: self.GameFlag(True))

        self.HandTracker.stop_tracking()
        self.LabTimeUpdate()

        self.GameWindow.DoMes("end_time")
        QTimer.singleShot(self.end_delay, lambda: self.GameWindow.DoMes("calib"))

    @pyqtSlot()
    def UpdateTimer(self):
        time = int(self.game_timer.remainingTime() / 1000)
        (minutes, seconds) = divmod(time, 60)
        time = f"{minutes:02.0f}:{seconds:02.0f}"
        self.lab_game_timeout.setText(time)
        self.GameWindow.set_time(time, self.game_timer.remainingTime() * self.GameWindow.progressBar.maximum() // self.game_timer.interval())

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
            self.CalibCam_Z.setValue(Settings["CalibCam_Z"])
            self.IpLineEdit.setText(f"{Settings['host']}:{Settings['port']}")
            self.FrequencySlender.setValue(Settings["timeout"])
            self.Change_XY_Box.setChecked(Settings["Change_XY"])
            self.Inv_X_Box.setChecked(Settings["Inv_X"])
            self.Inv_Y_Box.setChecked(Settings["Inv_Y"])
            self.timeEdit.setTime(QTime(0, *map(int, Settings["game_time"].split(":"))))
            self.Add_timeEdit.setTime(QTime(0, *map(int, Settings["add_game_time"].split(":"))))
        except Exception as e:
            print("Not found:" + str(e))
            self.Settings.save_settings(self.Settings.get_defaults_settings())

    @pyqtSlot()
    def SaveSettings(self):
        Settings = {"CalibDist": self.CalibDist.text(),
                    "CalibCam": self.CalibCam.value(),
                    "TimeApprox": self.TimeApprox.value(),
                    "CalibCam_Z": self.CalibCam_Z.value(),
                    "host": self.IpLineEdit.text().split(':')[0],
                    "port": self.IpLineEdit.text().split(':')[1],
                    "timeout": self.FrequencySlender.value(),
                    "Change_XY": self.Change_XY_Box.isChecked(),
                    "Inv_X": self.Inv_X_Box.isChecked(),
                    "Inv_Y": self.Inv_Y_Box.isChecked(),
                    "game_time": self.timeEdit.time().toString().split(":", 1)[1],
                    "add_game_time": self.Add_timeEdit.time().toString().split(":", 1)[1]}

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
        timeout = 8
        self.RobotThread.start_server(port, host, timeout)

    @pyqtSlot()
    def new_Cam(self):
        if self.NumCamEditLine.text().isdigit():
            self.Start_cam.setEnabled(False)
            self.CameraThread.set_cam(int(self.NumCamEditLine.text()))
            QTimer.singleShot(100, lambda: self.Start_cam.setEnabled(True))
            self.RobotThread.GoHome()
        else:
            self.showDialog("Введите число большее и равное нулю")

    @pyqtSlot()
    def change_cam(self):
        self.HandTracker.CalibDist = int(self.CalibDist.text())
        self.HandTracker.CalibCam = int(self.CalibCam.value()) / 100
        self.HandTracker.CalibCam_Z = int(self.CalibCam_Z.value()) / 100
        # self.HandTracker.TimeApprox = int(self.TimeApprox.value())
        # self.HandTracker.LenApprox = 60 * int(self.TimeApprox.value()) // 1000
        self.HandTracker.NewAveragingLen(int(self.TimeApprox.value()))

        self.RobotThread.sleep_time = 1000 // int(self.FrequencySlender.value())

        self.HandTracker.Change_XY = self.Change_XY_Box.isChecked()
        self.HandTracker.Inv_X = self.Inv_X_Box.isChecked()
        self.HandTracker.Inv_Y = self.Inv_Y_Box.isChecked()

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        img = self.convert_cv_qt(cv_img)
        self.Lab_Cam.setPixmap(img.scaled(self.Lab_Cam.size().width(), self.Lab_Cam.size().height(), Qt.KeepAspectRatio))
        self.GameWindow.pix_lab.setPixmap(img.scaled(self.GameWindow.pix_lab.size().width(),
                                                     self.GameWindow.pix_lab.size().height(), Qt.KeepAspectRatio))

    @pyqtSlot(np.single, np.single, np.single, bool, bool)
    def Hand_update(self, x, y, size_factor, hand_exist, compress):
        if hand_exist:
            result = self.HandTracker.give_Hand((x, y), size_factor, compress)
            self.Hand_Coords.setText(result)
            if result == "Success":
                self.StartGame()
        else:
            self.Hand_Coords.setText("No hand")
            self.HandTracker.StartTime = 0
        self.CameraThread.calibration_flag = not self.HandTracker.TrackingProcess

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
        return QPixmap.fromImage(QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888))

    def closeEvent(self, event):
        self.RobotThread.stop()
        self.CameraThread.stop()
        self.RobotThread.quit()
        self.CameraThread.quit()
        self.GameWindow.close()
        event.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    ui = RobotWindow()
    ui.show()
    sys.exit(app.exec_())
