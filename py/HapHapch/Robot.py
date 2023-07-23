import socket
from PyQt5.QtCore import QThread, pyqtSignal, QTime, Qt


class Robot(QThread):
    GetHand = pyqtSignal()
    RobotMessage = pyqtSignal(str)
    SendListUpdate = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self._run_flag = False
        self.is_connected = False

        self.host = ''
        self.port = 48569
        self.timeout = 5
        self.debug_mode = True
        self.separator = ';'

        self.Home_pose = [0, 0.5, 0.8]
        self.Compress = False
        self.Hand = self.Home_pose
        self.sleep_time = 30

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __print_debug(self, msg):
        if self.debug_mode:
            print(msg)

    def __connect(self):
        self.SendListUpdate.emit("Server started. Waiting for connect...")
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.host, int(self.port)))
        self.server.listen()
        try:
            self.__connection, ip_address = self.server.accept()
            self.__connection.settimeout(self.timeout)
            self.SendListUpdate.emit(f"Client with IP {ip_address} was connected")
            self.is_connected = True
        except Exception as e:
            pass

    def run(self):
        real_time = QTime()
        time = QTime()
        time.start()
        while self._run_flag:
            print(self.sleep_time, time.elapsed())
            if not self.is_connected:
                self.__connect()
            elif time.elapsed() > self.sleep_time:
                time.start()
                self.GetHand.emit()
                message = ';'.join(["{:5.3f}".format(i) for i in self.Hand]) + f";{str(self.Compress)};"
                '''
                if self.PrevCompress == self.Compress and self.PrevHand == self.Hand:
                    continue
                self.PrevCompress = self.Compress
                self.PrevHand = self.Hand
                '''
                try:
                    self.__connection.sendall(message.encode())
                    received_data = self.__connection.recv(1024).decode()
                    self.SendListUpdate.emit(f"{real_time.currentTime().toString(format=Qt.ISODateWithMs)} sent: {message} received: {received_data}")
                except Exception as e:
                    self.__print_debug(e)
                    if self.is_connected:
                        self.is_connected = False
                        try:
                            self.__connection.close()
                        except Exception as e:
                            self.__print_debug(e)
                    print("Connection broken or timeout exceeded. Restarting server")

    def start_server(self, port, host, timeout):
        self.host = port
        self.port = host
        self.timeout = timeout

        if self._run_flag:
            self.stop()

        self._run_flag = True
        self.send_mess = True
        self.start()

    def stop(self):
        try:
            self.server.close()
        except Exception as e:
            print(e)
        if self._run_flag:
            self._run_flag = False
            self.wait()
            self.SendListUpdate.emit("Server closed.")

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

if __name__ == "__main__":
    #import cv2

    rdt = Robot()
    rdt.start()
    while not rdt.is_connected:
        pass
    #cv2.namedWindow("test")
    while True:
        if rdt.is_connected:
            rdt.send('Lol;')
            #print("aaaa" + str(rdt.receive()))