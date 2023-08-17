import time
import datetime
import os

class Logger(object):

    def __init__(self, file_path : str, log_type = None) -> None:
        self.path = file_path
        self.type = log_type
        self.runtime = time.time()
        if os.path.isdir(self.path):
            pass
        else:
            os.mkdir(self.path)
        self.log = open(self.path + "/" + str(log_type) + ".out", "w")
        if log_type:
            self.log = open(self.path + "/" + str(log_type) + ".out", "w")
        else:
            formatted_time = time.strftime('%Y_%m_%d_%H:%M:%S', time.localtime(self.runtime))
            self.log = open(formatted_time + "_" + str(log_type) + ".out", "w")

    def write(self, msg : str):
        self.log.write(msg)

    def __repr__(self):
        return str(self.log)