# -*- coding: utf-8 -*-

from PyQt5.QtCore import QThread, pyqtSignal

from RobotDataSender import RobotSender

class RobotObject(QThread):
    GetHand = pyqtSignal()
    RobotMessage = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run_flag = False
        self.Home_pose = [0, 0, 0.8]
        self.Hand = self.Home_pose
        self.Sender = RobotSender

    def RobotConnect(self, RobotPort):
        self.Sender.setPortHost(RobotPort)

    def RobotStart(self):
        pass

    def run(self):
        # not rospy.is_shutdown():
        while self._run_flag:
            self.GetHand.emit()
            self.msleep(10)

    def SetHand(self, CamInfo):
        x = round((CamInfo["Hand"][1] + CamInfo["height"] / 2) / CamInfo["height"], 3)
        y = round((CamInfo["Hand"][0] + CamInfo["width"] / 2) / CamInfo["width"], 3)
        z = round((CamInfo["CalibDist"] - CamInfo["Hand"][2]) / CamInfo["CalibDist"] + self.Home_pose[2], 3)

        self.Hand = [x, y, z]
        self.Compress = CamInfo["Compress"]

        for i, cord in enumerate(self.Hand):
            if cord > 1:
                self.Hand[i] = 1
            elif cord < 0:
                self.Hand[i] = 0

    def GoHome(self):
        self.Hand = self.Home_pose

    def stop(self):
        if self._run_flag:
            """Sets run flag to False and waits for thread to finish"""
            self._run_flag = False
            self.wait()