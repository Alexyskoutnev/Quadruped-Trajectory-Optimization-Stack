#! /usr/bin/python3

import subprocess
import shlex


TOWR_SCRIPT = shlex.split("./test 2")

def _run(args):
    subprocess.run(TOWR_SCRIPT)

if __name__ == "__main__":
    args = {}
    _run(args)