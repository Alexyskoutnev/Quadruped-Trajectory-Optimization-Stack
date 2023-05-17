import sys

class ROBOT_CFG:
    robot = None
    linkWorldPosition = [0.0, 0, 0.24]
    linkWorldOrientation = [0, 0, 0]
    last_POSE = [0, 0, 0.24]
    robot_goal = [0, 0, 0]
    EE = {"FL_FOOT": [0.15,0.10,0], "FR_FOOT": [0.15,-0.10,0], "HL_FOOT": [-0.15,0.10,0] , "HR_FOOT": [-0.15, -0.10,0]}

class RUN:
    step = 0
    update = False
    _wait = False
    TOWR_POS = [0, 0, 0]

def print_vars(stream = sys.__stdout__):
    sys.stdout = stream
    print("=========ROBOT_CFG GLOBAL VARS==========")
    print("Global POSITION: ",  ROBOT_CFG.linkWorldPosition)
    print("GLOBAL ORIENTATION: ", ROBOT_CFG.linkWorldPosition)
    print("LAST POSE: ", ROBOT_CFG.last_POSE)
    print("ROBOT GOAL: ", ROBOT_CFG.robot_goal)
    print("EE: ", ROBOT_CFG.EE)

    print("=========RUN GLOBAL VARS==========")
    print("STEP: ",  RUN.step)
    print("UPDATE: ", RUN.update)
    print("WAIT: ", RUN._wait)
    print("TOWR POS: ", RUN.TOWR_POS)
    sys.stdout = sys.__stdout__