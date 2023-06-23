from PyQt5 import QtCore, QtGui, QtWidgets
import RobotControl

# pyuic5 -x ControlBoard.ui -o QTControlBoard.py

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
        self.Slide_1.setMinimum(-400)
        self.Slide_1.setMaximum(400)
        self.Slide_1.setPageStep(1)
        self.Slide_1.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_1.setObjectName("Slide_1")
        self.verticalLayout_3.addWidget(self.Slide_1)
        self.Slide_2 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_2.setMinimum(-200)
        self.Slide_2.setMaximum(200)
        self.Slide_2.setPageStep(1)
        self.Slide_2.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_2.setObjectName("Slide_2")
        self.verticalLayout_3.addWidget(self.Slide_2)
        self.Slide_3 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_3.setMinimum(-200)
        self.Slide_3.setMaximum(200)
        self.Slide_3.setPageStep(1)
        self.Slide_3.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_3.setObjectName("Slide_3")
        self.verticalLayout_3.addWidget(self.Slide_3)
        self.Slide_4 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_4.setMinimum(-200)
        self.Slide_4.setMaximum(200)
        self.Slide_4.setPageStep(1)
        self.Slide_4.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_4.setObjectName("Slide_4")
        self.verticalLayout_3.addWidget(self.Slide_4)
        self.Slide_5 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_5.setMinimum(-200)
        self.Slide_5.setMaximum(200)
        self.Slide_5.setPageStep(1)
        self.Slide_5.setOrientation(QtCore.Qt.Horizontal)
        self.Slide_5.setObjectName("Slide_5")
        self.verticalLayout_3.addWidget(self.Slide_5)
        self.Slide_6 = QtWidgets.QSlider(self.centralwidget)
        self.Slide_6.setMinimum(-200)
        self.Slide_6.setMaximum(200)
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

        self.Slide_1.setValue(40)
        self.Slide_2.setValue(10)
        self.Slide_3.setValue(40)
        self.Slide_4.setValue(0)
        self.Slide_5.setValue(0)
        self.Slide_6.setValue(100)

        self.HandToRobot()

        self.Change_hand()

        self.RobotThread.GetHand.connect(self.HandToRobot)

        self.RobotThread.start()

    def Change_hand(self):
        self.Hand = [self.Slide_1.value()/100, self.Slide_2.value()/100, self.Slide_3.value()/100,
                     self.Slide_4.value()/100, self.Slide_5.value()/100, self.Slide_6.value()/100]

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
