# !/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtWidgets
from PyQt5.QtGui import QIcon
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

    def set_time(self, time, per_time):
        self.lcd_time.display(time)
        self.progressBar.setValue(per_time)

    def closeEvent(self, event):
        event.accept()
