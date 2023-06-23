from PyQt5 import QtCore, QtGui, QtWidgets
import RobotControl


class Ui_MainWindow(object):
    def __init__(self):
        self.RobotThread = RobotControl.RobotObject()

    def setupUi(self, MainWindow):
        #region
        MainWindow.setObjectName("MainWindow")
        MainWindow.setWindowModality(QtCore.Qt.NonModal)
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setMinimumSize(QtCore.QSize(800, 600))
        self.centralwidget.setMaximumSize(QtCore.QSize(800, 16777215))
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.Slide_1 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_1.setMaximum(180)
        self.Slide_1.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_1.setObjectName("Slide_1")
        self.verticalLayout_3.addWidget(self.Slide_1)
        self.Slide_2 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_2.setMaximum(180)
        self.Slide_2.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_2.setObjectName("Slide_2")
        self.verticalLayout_3.addWidget(self.Slide_2)
        self.Slide_3 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_3.setMaximum(180)
        self.Slide_3.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_3.setObjectName("Slide_3")
        self.verticalLayout_3.addWidget(self.Slide_3)
        self.Slide_4 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_4.setMaximum(180)
        self.Slide_4.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_4.setObjectName("Slide_4")
        self.verticalLayout_3.addWidget(self.Slide_4)
        self.Slide_5 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_5.setMaximum(180)
        self.Slide_5.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_5.setObjectName("Slide_5")
        self.verticalLayout_3.addWidget(self.Slide_5)
        self.Slide_6 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_6.setMaximum(180)
        self.Slide_6.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_6.setObjectName("Slide_6")
        self.verticalLayout_3.addWidget(self.Slide_6)
        self.horizontalLayout.addLayout(self.verticalLayout_3)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        # endregion

        self.Slide_1.valueChanged.connect(self.Change_hand)
        self.Slide_2.valueChanged.connect(self.Change_hand)
        self.Slide_3.valueChanged.connect(self.Change_hand)
        self.Slide_4.valueChanged.connect(self.Change_hand)
        self.Slide_5.valueChanged.connect(self.Change_hand)
        self.Slide_6.valueChanged.connect(self.Change_hand)

        self.Change_hand()

        self.RobotThread.GetHand.connect(self.HandToRobot)

    def Change_hand(self):
        self.Hand = [self.Slide_1.value(), self.Slide_2.value(), self.Slide_3.value(),
                     self.Slide_4.value(), self.Slide_5.value(), self.Slide_6.value()]

    def HandToRobot(self):
        self.RobotThread.SetHand(self.Hand)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "TextLabel"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
