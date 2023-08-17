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
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_6.setContentsMargins(5, 5, 5, 5)
        self.gridLayout_6.setSpacing(5)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.description_lab = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.description_lab.sizePolicy().hasHeightForWidth())
        self.description_lab.setSizePolicy(sizePolicy)
        self.description_lab.setSizeIncrement(QtCore.QSize(0, 0))
        self.description_lab.setFrameShape(QtWidgets.QFrame.Panel)
        self.description_lab.setFrameShadow(QtWidgets.QFrame.Raised)
        self.description_lab.setLineWidth(2)
        self.description_lab.setObjectName("description_lab")
        self.horizontalLayout.addWidget(self.description_lab)
        self.lcd_time = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_time.setMinimumSize(QtCore.QSize(200, 100))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.lcd_time.setFont(font)
        self.lcd_time.setFrameShape(QtWidgets.QFrame.Panel)
        self.lcd_time.setFrameShadow(QtWidgets.QFrame.Raised)
        self.lcd_time.setLineWidth(2)
        self.lcd_time.setSmallDecimalPoint(False)
        self.lcd_time.setDigitCount(4)
        self.lcd_time.setMode(QtWidgets.QLCDNumber.Dec)
        self.lcd_time.setSegmentStyle(QtWidgets.QLCDNumber.Filled)
        self.lcd_time.setProperty("value", 0.0)
        self.lcd_time.setProperty("intValue", 0)
        self.lcd_time.setObjectName("lcd_time")
        self.horizontalLayout.addWidget(self.lcd_time)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.progressBar = QtWidgets.QProgressBar(self.centralwidget)
        self.progressBar.setMaximum(1000000)
        self.progressBar.setProperty("value", 24)
        self.progressBar.setTextVisible(False)
        self.progressBar.setOrientation(QtCore.Qt.Horizontal)
        self.progressBar.setObjectName("progressBar")
        self.verticalLayout.addWidget(self.progressBar)
        self.pix_lab = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pix_lab.sizePolicy().hasHeightForWidth())
        self.pix_lab.setSizePolicy(sizePolicy)
        self.pix_lab.setText("")
        self.pix_lab.setObjectName("pix_lab")
        self.verticalLayout.addWidget(self.pix_lab)
        self.gridLayout_6.addLayout(self.verticalLayout, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "CatControl"))
        self.description_lab.setText(_translate("MainWindow", "TextLabel"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
