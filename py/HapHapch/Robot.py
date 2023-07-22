import socket as sockets
from PyQt5.QtCore import QThread, pyqtSignal


class Robot(QThread):
    GetHand = pyqtSignal()
    RobotMessage = pyqtSignal(str)
    SendListUpdate = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self._run_flag = False
        self.is_connected = False

        self.sent_percentage = 0
        self.drawn_percentage = 0
        self.stop_required = False

        self.host = ''
        self.port = 48569
        self.timeout = 0
        self.debug_mode = True
        self.separator = ';'

        self.socket = sockets.socket(sockets.AF_INET, sockets.SOCK_STREAM)

    def __print_debug(self, msg):
        if self.debug_mode:
            print(msg)

    def __connect(self):
        self.SendListUpdate.emit("Server started. Waiting for connect...")
        self.socket = sockets.socket(sockets.AF_INET, sockets.SOCK_STREAM)
        self.socket.bind((self.host, int(self.port)))
        self.socket.listen()
        try:
            self.__connection, ip_address = self.socket.accept()
            self.__connection.settimeout(self.timeout)
            self.SendListUpdate.emit(f"Client with IP {ip_address} was connected")
            self.is_connected = True
        except Exception as e:
            pass

    def __send(self):
        self.stop_required = False
        _array = []
        for i in range(0, len(self.__data), 15):
            package = ['contour']
            for j in range(15):
                try:
                    package.append(self.__data[i + j][0])
                    package.append(self.__data[i + j][1])
                    package.append(self.__data[i + j][2])
                except IndexError:
                    pass
            _data = self.__prepare_sending_data(package)
            _array.append(_data)
        data_length = len(_array)
        _data = self.__prepare_sending_data(['start', data_length])
        self.__connection.sendall(_data)
        received_data = self.__connection.recv(1024)
        while received_data != b'start_accepted':
            print(received_data)
            received_data = self.__connection.recv(1024)
        l = 1
        for element in _array:
            self.__connection.sendall(element)
            received_data = self.__connection.recv(1024)
            print(received_data)
            self.drawn_percentage = float(received_data.decode())
            self.sent_percentage = l * 100 / data_length
            l = l + 1
            if self.stop_required:
                self.__connection.sendall(b'stop;')
                received_data = self.__connection.recv(1024)
                self.drawn_percentage = float(received_data.decode())
                self.stop_required = False
                break
        while self.drawn_percentage != 100:
            self.__connection.sendall(b'heartbeat;')
            received_data = self.__connection.recv(1024)
            self.drawn_percentage = float(received_data.decode())
            if self.stop_required:
                self.__connection.sendall(b'stop;')
                received_data = self.__connection.recv(1024)
                self.drawn_percentage = float(received_data.decode())
                self.stop_required = False
                break

    def run(self):
        wait = lambda: self.__connection.sendall(b'heartbeat;')
        self.__data = ""
        while self._run_flag:
            if not self.is_connected:
                self.__connect()
            else:
                try:
                    if self.__data != '':
                        self.__send()
                        self.__data = ''
                    else:
                        wait()
                        received_data = self.__connection.recv(1024)
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
        self.start()

    def __prepare_sending_data(self, data):
        if isinstance(data, int):
            _data = str(data) + self.separator
        if isinstance(data, str):
            _data = data + self.separator
        if isinstance(data, list):
            _data = self.separator.join(map(str, data)) + self.separator
        return _data.encode()

    def stop(self):
        if self._run_flag:
            try:
                self.socket.close()
            except Exception as e:
                print(e)

            self._run_flag = False
            self.wait()
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