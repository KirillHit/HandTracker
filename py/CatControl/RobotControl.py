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
        self.Sender = RobotSender()

    def RobotConnect(self, port_host):
        self.Sender.set_host_port(port_host)
        self.Sender.connect()

        self.Compress = False
        self.CompressFlag = False

        self.PrevHand = None
        self.PrevCompress = None


    def RobotStart(self):
        if not self._run_flag:
            self._run_flag = True
            self.start()

    def stop(self):
        if self._run_flag:
            self._run_flag = False
            self.wait()

    def run(self):
        # not rospy.is_shutdown():
        while self._run_flag:
            self.GetHand.emit()
            self.msleep(10)

            if self.PrevCompress == self.Compress and self.PrevHand == self.Hand:
                # rospy.loginfo("Continue")
                continue
            elif self.CurHomePoseFlag:
                self.CurHomePoseFlag = False

            self.PrevCompress = self.Compress
            self.PrevHand = self.Hand

            self.mgc.set_start_state(self.rc.get_current_state())

            if self.Compress and not self.CompressFlag:
                cmdhandler_client("as", "CLOSE")
                self.CompressFlag = True
                self.mgc.stop()
                self.sleep(2)
            elif not self.Compress and self.CompressFlag:
                cmdhandler_client("as", "OPEN")
                self.CompressFlag = False
                self.mgc.stop()
                self.sleep(2)


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