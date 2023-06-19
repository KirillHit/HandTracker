from PyQt5.QtCore import QThread, pyqtSignal

class RobotObject(QThread):
    change_pixmap_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._run_flag = True

    def run(self):

        while self._run_flag:
        self.change_pixmap_signal.emit()

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()