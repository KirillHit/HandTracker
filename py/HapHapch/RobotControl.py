# -*- coding: utf-8 -*-

from PyQt5.QtCore import QThread, pyqtSignal

from RobotDataSender import RobotSender


class RobotObject(QThread):
    GetHand = pyqtSignal()
    RobotMessage = pyqtSignal(str)
    SendListUpdate = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run_flag = False
        self.Home_pose = [0, 0.5, 0.8]
        self.Hand = self.Home_pose
        self.Sender = RobotSender()
        self.sleep_time = 30

        self.ConnectFlag = False

    def RobotConnect(self, port_host):
        self.Sender.set_host_port(port_host)
        if self.Sender.connect():
            self.Sender.settings.save_settings()
            self.ConnectFlag = True
            self.Compress = False
            self.Hand = self.Home_pose
            self.PrevHand = None
            self.PrevCompress = None
            self.SendListUpdate.emit("Подключение установленно!")
        else:
            self.SendListUpdate.emit("Подключение не было установленно.")

    def RobotStart(self):
        if not self._run_flag and self.ConnectFlag:
            self.Hand = self.Home_pose
            self._run_flag = True
            self.start()
            self.SendListUpdate.emit("Управление активно!")

    def stop(self):
        if self._run_flag:
            self._run_flag = False
            self.wait()
            self.SendListUpdate.emit("Управление приостановлено.")

    def run(self):
        # not rospy.is_shutdown():
        while self._run_flag:
            self.GetHand.emit()
            self.msleep(self.sleep_time)

            if self.PrevCompress == self.Compress and self.PrevHand == self.Hand:
                continue
            self.PrevCompress = self.Compress
            self.PrevHand = self.Hand

            try:
                self.Sender.send_formatted_from_robot(*self.Hand, self.Compress)
                self.SendListUpdate.emit("Sent: " + ';'.join(["{:5.3f}".format(i) for i in self.Hand]) + f";{str(self.Compress)};")
            except Exception as e:
                self.RobotMessage.emit("Соединение разорвано:\n" + str(e))
                self.ConnectFlag = False
                self.stop()

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
