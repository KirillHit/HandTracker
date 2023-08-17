import os
import json
import socket


class Settings:
    def __init__(self, file_path='config/parameters.json'):
        self.__config_path = file_path
        self.__config_dir, self.__config_file = os.path.split(file_path)
        try:
            host = socket.gethostbyname(socket.gethostname())
            print(socket.gethostbyname_ex(socket.gethostname()))
        except Exception as e:
            host = ''
            print(e)
        self.DefaultsSetting = {"host": host,
                                "port": "48569",
                                "timeout": 30,
                                "CalibDist": "500",
                                "CalibCam": 100,
                                "TimeApprox": 30,
                                "FixedParam": 3,
                                "FixedParam_Z": 4,
                                "Change_XY": False,
                                "Inv_X": False,
                                "Inv_Y": False,
                                "game_time": "1:30",
                                "add_game_time": "2:30"}
  #123
    
    def __load_settings(self):
        if os.path.exists(self.__config_path):
            try:
                with open(self.__config_path, 'r') as file:
                    self.config = json.load(file)
            except Exception as e:
                print(f"Error of read setting from file {e}")
                self.__load_defaults()
        else:
            if not os.path.exists(self.__config_dir): os.makedirs(self.__config_dir)
            self.__load_defaults()

    def __load_defaults(self):
        self.save_settings(self.DefaultsSetting)
        self.config = self.DefaultsSetting

    def save_settings(self, NewConfig):
        try:
            with open(self.__config_path, 'r') as file:
                config = json.load(file)
        except Exception as e:
            print(e)
            config = {}

        config.update(NewConfig)

        with open(self.__config_path, 'w') as file:
            json.dump(config, file)

    def get_settings(self):
        self.__load_settings()
        return self.config

    def get_defaults_settings(self):
        return self.DefaultsSetting
