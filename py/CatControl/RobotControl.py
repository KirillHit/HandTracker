# -*- coding: utf-8 -*-

from PyQt5.QtCore import QThread, pyqtSignal

import socket
import os
import math

class RobotObject(QThread):
    GetHand = pyqtSignal()
    RobotMessage = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run_flag = False
        self.Home_pose = [0.4, 0, 0.3]
        self.HomePoseFlag = False
        self.Hand = self.Home_pose

        self.Compress = False
        self.CompressFlag = False

        self.PrevHand = None
        self.PrevCompress = None

        self.max_cord = [0.8, 0.3, 0.6]
        self.min_cord = [0.3, -0.3, 0]

        self.ConnectFlag = False
        self.InitFlag = False

    def RobotConnect(self, RobotModel, SimulationFlag, RobotIp="192.168.0.2"):
        pass

    def RobotStart(self):
        if not self._run_flag and self.ConnectFlag:
            self.PrevHand = None
            self.PrevCompress = None
            self.GoHome()

            self._run_flag = True
            self.start()

    def run(self):
        while self._run_flag and not rospy.is_shutdown():
            self.GetHand.emit()


            # Пропуск идентичных запросов
            if self.HomePoseFlag:
                if self.CurHomePoseFlag:
                    continue
                else:
                    self.CurHomePoseFlag = True
            elif self.PrevCompress == self.Compress and self.PrevHand == self.Hand:
                continue
            elif self.CurHomePoseFlag:
                self.CurHomePoseFlag = False

            self.PrevCompress = self.Compress
            self.PrevHand = self.Hand


            if self.Compress and not self.CompressFlag:
                self.CompressFlag = True
                self.sleep(2)
            elif not self.Compress and self.CompressFlag:
                self.CompressFlag = False
                self.sleep(2)

    def SetHand(self, CamInfo, PrecisionParam):
        x = ((self.max_cord[0] - self.min_cord[0]) / (CamInfo["height"] * PrecisionParam))\
            * (CamInfo["height"] * PrecisionParam // 2 - CamInfo["Hand"][1]) + self.min_cord[0]
        y = ((self.max_cord[1] - self.min_cord[1]) / (CamInfo["width"] * PrecisionParam)) * CamInfo["Hand"][0]
        z = (CamInfo["CalibDist"] - CamInfo["Hand"][2]) / 1000 + self.Home_pose[2]
        '''
        x = CamInfo["height"]/100
        y = CamInfo["Hand"][0]/100
        z = (CamInfo["CalibDist"] - CamInfo["Hand"][2]) / 100 + self.Home_pose[2]
        '''

        self.Hand = [x, y, z]
        self.Compress = CamInfo["Compress"]

        for i, cord in enumerate(self.Hand):
            if cord > self.max_cord[i]:
                self.Hand[i] = self.max_cord[i]
            elif cord < self.min_cord[i]:
                self.Hand[i] = self.min_cord[i]

        if self.HomePoseFlag:
            self.HomePoseFlag = False

    def GoHome(self):
        self.Hand = self.Home_pose
        self.Compress = False
        self.HomePoseFlag = True
        self.CurHomePoseFlag = False

    def stop(self):
        if self._run_flag:
            """Sets run flag to False and waits for thread to finish"""
            self._run_flag = False
            self.wait()