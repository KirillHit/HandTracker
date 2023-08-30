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
        self.server_flag = False

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

    def __start_server(self):
        self.SendListUpdate.emit("Server started. Waiting for connect...")
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.server.bind((self.host, int(self.port)))
            self.server.listen()
            self.server_flag = True
        except Exception as e:
            self.SendListUpdate.emit("Server error.")
            self.__print_debug(e)
            self.stop()

    def __connect(self):
        try:
            self.__connection, ip_address = self.server.accept()
            self.__connection.settimeout(self.timeout)
            self.SendListUpdate.emit(f"Client with IP {ip_address} was connected")
            self.is_connected = True
        except Exception as e:
            self.SendListUpdate.emit("Connection error.")
            self.__print_debug(e)

    def run(self):
        real_time = QTime()
        time = QTime()
        time.start()
        while self._run_flag:
            if not self.server_flag:
                self.__start_server()
            if not self.is_connected:
                self.__connect()
            elif time.elapsed() > self.sleep_time:
                time.start()
                self.GetHand.emit()
                if self.Home_pose:
                    message = "home;"
                else:
                    message = "go;" + ';'.join(["{:.3f}".format(i) for i in self.Hand]) + f";{str(self.Compress)};"
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
                    self.__print_debug("Connection broken or timeout exceeded. Restarting server")

    def start_server(self, host, port, timeout):
        self.host = host
        self.port = port
        self.timeout = timeout

        if self._run_flag:
            self.stop()

        self._run_flag = True
        self.send_mess = True
        self.Home_pose = True
        self.start()

    def SetHand(self, hand_info):
        self.Hand = hand_info[0]
        self.Compress = hand_info[1]
        self.Home_pose = False

    def GoHome(self):
        self.Home_pose = True

    def stop(self):
        if self._run_flag:
            self._run_flag = False
            self.terminate()
        if self.is_connected:
            try:
                self.__connection.close()
                self.is_connected = False
            except Exception as e:
                self.__print_debug(e)
        try:
            self.server.close()
            self.server_flag = False
            self.is_connected = False
        except Exception as e:
            self.__print_debug(e)
        self.SendListUpdate.emit("Server closed.")

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