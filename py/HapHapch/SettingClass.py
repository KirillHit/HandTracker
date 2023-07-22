import os
import json


class Settings:
    def __init__(self, file_path='config/parameters.json'):
        self.__config_path = file_path
        self.__config_dir, self.__config_file = os.path.split(file_path)
        self.DefaultsSetting = {"host": '',
                                "port": 48569,
                                "timeout": 5}

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
