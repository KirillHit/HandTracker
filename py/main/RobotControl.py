from PyQt5.QtCore import QThread, pyqtSignal
import time

class RobotObject(QThread):
    GetHand = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.prevTime = time.time()


    def timerEvent(self, event):
        self.GetHand.emit()

    def startTimer(self):
        self.startTimer(100)

    def run(self):
        """
        while self._run_flag:
            self.GetHand.emit()
            #self.msleep(30)
        """

    def SetHand(self, Hand):
        t = time.time()
        print(t - self.prevTime, Hand)
        self.prevTime = t

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()