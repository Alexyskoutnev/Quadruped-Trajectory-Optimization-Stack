import time
import datetime
import os

class Logger(object):
    """A simple logger for writing messages to a log file.

    Attributes:
        file_path (str): The path to the directory where log files will be stored.
        log_type (str): The type of log (e.g., 'error', 'info', 'debug').
    """

    def __init__(self, file_path : str, log_type = None) -> None:
        """Initialize a Logger object.

        Args:
            file_path (str): The path to the directory where log files will be stored.
            log_type (str, optional): The type of log (e.g., 'error', 'info', 'debug'). Defaults to None.
        """
        self.path = file_path
        self.type = log_type
        self.runtime = time.time()
        if not os.path.isdir(self.path):
            os.mkdir(self.path)
        self.log = open(self.path + "/" + str(log_type) + ".out", "w")
        if log_type:
            self.log = open(self.path + "/" + str(log_type) + ".out", "w")
        else:
            formatted_time = time.strftime('%Y_%m_%d_%H:%M:%S', time.localtime(self.runtime))
            self.log = open(formatted_time + "_" + str(log_type) + ".out", "w")

    def write(self, msg : str):
        """Write a message to the log file.

        Args:
            msg (str): The message to be written to the log.
        """
        self.log.write(msg)

    def __repr__(self):
        """Return a string representation of the Logger object.

        Returns:
            str: A string representation of the Logger object.
        """
        return str(self.log)