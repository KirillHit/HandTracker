# !/env python
# -*- coding: utf-8 -*-
import os
import random

from PyQt5 import QtWidgets
from PyQt5.QtCore import QSize, QTimer
from PyQt5.QtGui import QIcon, QMovie
import Qt

from Qt.GameWindow.PyQtWindow import Ui_MainWindow


class GameWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        # Загрузка окна
        super().__init__()
        self.setupUi(self)

        # Самая важная строчка
        self.setWindowIcon(QIcon("icon.jpg"))

        self.lcd_time.setDigitCount(5)

        self.new_img()
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.new_img)
        self.update_timer.setInterval(1000)
        self.update_timer.start()

    def new_img(self):
        list = os.listdir("Qt/GameWindow/img")
        img = list[random.randint(0, len(list) - 1)]
        self.movie = QMovie("Qt/GameWindow/img/" + img)
        self.gif_lab.setMovie(self.movie)
        self.resizeEvent(None)
        self.movie.start()

    def resizeEvent(self, event):
        rect = self.gif_lab.size()
        size = QSize(min(rect.width(), rect.height()), min(rect.width(), rect.height()))
        self.gif_lab.movie().setScaledSize(size)

    def set_time(self, time, per_time):
        self.lcd_time.display(time)
        self.progressBar.setValue(per_time)

    def closeEvent(self, event):
        event.accept()
