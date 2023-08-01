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

        self.Home_pose = True
        self.Compress = False
        self.Hand = [0, 0, 0]
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
            if not self.is_connected:
                self.__connect()
            elif time.elapsed() > self.sleep_time:
                time.start()
                self.GetHand.emit()
                if self.Home_pose:
                    message = "home;"
                else:
                    message = "go;" + ';'.join(["{:5.3f}".format(i) for i in self.Hand]) + f";{str(self.Compress)};"
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
        self.Home_pose = True
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

    def SetHand(self, hand_info):
        self.Hand = hand_info[0]
        self.Compress = hand_info[1]
        self.Home_pose = False

    def GoHome(self):
        self.Home_pose = True

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