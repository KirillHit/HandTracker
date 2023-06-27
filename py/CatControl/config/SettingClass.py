import os
import json


class Settings:
    __HOST_STRING = 'host'
    __PORT_STRING = 'port'
    __TIMEOUT_STRING = 'timeout'
    __DEBUG_MODE_STRING = 'debug_mode'
    __SEPARATOR_STRING = 'separator'

    def __init__(self, file_path):
        self.__config_path = file_path
        self.__config_dir, self.__config_file = os.path.split(file_path)
        if os.path.exists(self.__config_path):
            self.__load_settings()
        else:
            if not os.path.exists(self.__config_dir): os.makedirs(self.__config_dir)
            self.__load_defaults()

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
            print(f"Error of read setting from file {e}")
            self.__load_defaults()

    def __load_defaults(self):
        self.host = ''
        self.port = 48569
        self.timeout = 5
        self.debug_mode = True
        self.separator = ';'

        self.save_settings()

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
            json.dump(config, file)
