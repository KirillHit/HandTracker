import socket
from config.SettingClass import Settings


class RobotSender:
    def __init__(self):
        self.settings = Settings()

        self.__data = ''
        self.is_connected = False

    def __print_debug(self, msg):
        if self.settings.debug_mode:
            print(msg)

    def connect(self):
        try:
            # prepiar socket
            self.__connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__connection.settimeout(self.settings.timeout)

            # start connection with server
            self.__connection.connect((self.settings.host, self.settings.port))
            self.is_connected = True
        except Exception as ex:
            self.__print_debug(f"Error has been handled: {ex}. Socket connection closing.")
            self.__connection.close()
        finally:
            return self.is_connected

    def disconnect(self):
        try:
            self.__connection.close()
        except Exception as ex:
            self.__print_debug(f"Error has been handled: {ex}.")
        finally:
            self.is_connected = False

    def __send(self, data):
        if not self.is_connected:
            self.__print_debug("Has no active connection")
            return

        if isinstance(data, str):
            self.__data = data + self.settings.separator
        if isinstance(data, list):
            self.__data = self.settings.separator.join(map(str, data)) + self.settings.separator
        self.__connection.sendall(self.__data.encode())
        __data = ''

    def send_from_robot(self, x, y, z, compress):
        self.__send([x, y, z, compress])

    def send_formatted_from_robot(self, x, y, z, compress):
        self.__send([
            "{:10.3f}".format(x),
            "{:10.3f}".format(y),
            "{:10.3f}".format(z),
            compress
        ])

    def set_host_port(self, host_port):
        if not self.is_connected:
            self.settings.host = host_port.split(':')[0]
            self.settings.port = int(host_port.split(':')[1])
            self.settings.save_settings()


if __name__ == "__main__":
    print("Start sender mini program")

    robo = RobotSender()
    robo.connect()

    print("Wait for commands...")
    while True:

        p = input()

        if p == "1":
            robo.send_from_robot(1, 2, 3, True)
        elif p == "2":
            robo.send_from_robot(123, 456, 789, False)
        else:
            break

    robo.disconnect()
