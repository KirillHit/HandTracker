import os
import json
import socket as sockets
from threading import Thread

import debugpy


class Robot(Thread):
    __HOST_STRING = 'host'
    __PORT_STRING = 'port'
    __TIMEOUT_STRING = 'timeout'
    __DEBUG_MODE_STRING = 'debug_mode'
    __SEPARATOR_STRING = 'separator'

    def __init__(self, config_path='config/robot.json'):
        Thread.__init__(self)

        self.__config_path = config_path
        self.__config_dir, self.__config_file = os.path.split(config_path)
        if os.path.exists(self.__config_path):
            self.__load_settings()
        else:
            if not os.path.exists(self.__config_dir):
                os.makedirs(self.__config_dir)
            self.__load_defaults()
        self.is_connected = False
        self.is_sending = False
        self.sent_percentage = 0
        self.drawn_percentage = 0
        self.stop_required = False

    def __load_defaults(self):
        self.host = ''
        self.port = 48569
        self.timeout = 5
        self.debug_mode = True
        self.separator = ';'

        self.save_settings()

    def __load_settings(self):
        try:
            with open(self.__config_path, 'r') as file:
                config = json.load(file)
                self.host = config[self.__HOST_STRING]
                self.port = config[self.__PORT_STRING]
                self.timeout = config[self.__TIMEOUT_STRING]
                self.debug_mode = config[self.__DEBUG_MODE_STRING]
                self.separator = config[self.__SEPARATOR_STRING]
        except Exception as e:
            print(e)
            self.__load_defaults()

    def __print_debug(self, msg):
        if self.debug_mode:
            print(msg)

    def __connect(self):
        print("Server started. Waiting for connect...")
        socket = sockets.socket(sockets.AF_INET, sockets.SOCK_STREAM)
        socket.bind((self.host, self.port))
        socket.listen()
        self.__connection, ip_address = socket.accept()
        self.__connection.settimeout(self.timeout)
        print(f"Client with IP {ip_address} was connected")
        self.is_connected = True

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
        self.is_sending = False

    def __thread(self):
        debugpy.debug_this_thread()
        wait = lambda: self.__connection.sendall(b'heartbeat;')
        self.__data = ""
        while True:
            if not self.is_connected:
                self.__connect()
            try:
                if self.__data != '':
                    self.__send()
                    self.__data = ''
                else:
                    wait()
                    received_data = self.__connection.recv(1024)
            except Exception as e:
                self.__print_debug(e)
                self.is_connected = False
                try:
                    self.__connection.close()
                except Exception as e:
                    self.__print_debug(e)
                print("Connection broken or timeout exceeded. Restarting server")

    #     __wait = lambda: self.__connection.sendall(b'wait;')
    #     self.__connect()
    #     self.__response = False
    #     while True:
    #         try:
    #             if self.__data != '':
    #                 self.__connection.sendall(self.__data.encode())
    #                 self.__data = ''
    #                 self.__response = False
    #             else:
    #                 __wait()
    #             self.received_data = self.__connection.recv(1024)
    #             self.__print_debug(self.received_data)
    #             self.__response = True

    def run(self):
        self.__thread()
        self.__print_debug("RobotData thread started")

    def __connection_error_handler(self, e):
        self.__print_debug(e)
        print("Connection broken or timeout exceeded. Restarting server")
        self.is_connected = False
        try:
            self.__connection.close()
        except Exception as e:
            self.__print_debug(e)

    def __prepare_sending_data(self, data):
        if isinstance(data, int):
            _data = str(data) + self.separator
        if isinstance(data, str):
            _data = data + self.separator
        if isinstance(data, list):
            _data = self.separator.join(map(str, data)) + self.separator
        return _data.encode()

    def send(self, data):
        self.is_sending = True
        self.__data = data

    def save_settings(self):
        try:
            with open(self.__config_path, 'r') as file:
                config = json.load(file)
        except Exception as e:
            print(e)
            config = {}

        config[self.__HOST_STRING] = self.host
        config[self.__PORT_STRING] = self.port
        config[self.__TIMEOUT_STRING] = self.timeout
        config[self.__DEBUG_MODE_STRING] = self.debug_mode
        config[self.__SEPARATOR_STRING] = self.separator

        with open(self.__config_path, 'w') as file:
            json.dump(config, file, indent=4)


if __name__ == "__main__":
    import cv2

    rdt = Robot(config_path='config/parameters.json')
    rdt.start()
    while not rdt.is_connected:
        pass
    cv2.namedWindow("test")
    while True:
        if rdt.is_connected:
            rdt.send('Lol;')
            print("aaaa" + str(rdt.receive()))