# !/env python
# -*- coding: utf-8 -*-
import os
import random

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QIcon, QPixmap
import Qt

from Qt.GameWindow.PyQtWindow import Ui_MainWindow


class GameWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        # Загрузка окна
        super().__init__()
        self.setupUi(self)

        # Самая важная строчка
        self.setWindowIcon(QIcon("icon.jpg"))
        self.logo_lab.setPixmap(QPixmap("Qt/GameWindow/logo.png"))

        self.lcd_time.setDigitCount(5)

        self.description_lab.setAlignment(QtCore.Qt.AlignJustify)

    def DoMes(self, mes=""):
        match mes:
            case "calib":
                self.status_lab.setText("Калибровка камеры")
                self.description_lab.setText("<ul><li>Держите руку <b>ладонью вверх</b> перед камерой.</li>"
                                             "<li>Для начала игры поместите руку в зелёный круг в центре кадра.</li>"
                                             "<li>При калибровки держите руку на уровне метки.</li></ul>")
            case "stop":
                self.status_lab.setText("Игра остановлена")
                self.description_lab.setText("<ul><li>Оператор остановил игру.</li></ul>")
            case "start":
                self.status_lab.setText("Игра началась")
                self.description_lab.setText("<ul><li>Держите руку ладонью вверх.</li>"
                                             "<li>Захват робота повторяет движения руки.</li>"
                                             "<li>Двигайте рукой плавно.</li>"
                                             "<li>Для сжатия захвата, сожмите руку.</li>"
                                             "<li>В случае потери руки, поднесите её ближе к камере.</li></ul>")
            case "end_time":
                self.status_lab.setText("Время закончилось")
                self.description_lab.setText("Игра перезапустится через несколько секунд.")

    def resizeEvent(self, event):
        self.logo_lab.setPixmap(QPixmap("Qt/GameWindow/logo.png").scaledToWidth(self.logo_lab.width()))

    def set_time(self, time, per_time):
        self.lcd_time.display(time)
        self.progressBar.setValue(per_time)

    def closeEvent(self, event):
        event.accept()
