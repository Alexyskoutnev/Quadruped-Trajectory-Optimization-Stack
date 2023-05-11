class ROBOT_CFG:
    robot = None
    linkWorldPosition = [0.0, 0, 0]
    linkWorldOrientation = [0, 0, 0]
    last_POSE = [0, 0, 0]
    robot_goal = [0, 0, 0]
    EE = {"FL_FOOT": [0.15,0.10,0], "FR_FOOT": [0.15,-0.10,0], "HL_FOOT": [-0.15,0.10,0] , "HR_FOOT": [-0.15, -0.10,0]}

class RUN:
    step = 0
    update = False
    _wait = False

def print_vars():
    print("=========ROBOT_CFG GLOBAL VARS==========")
    print("Global POSITION: ",  ROBOT_CFG.linkWorldPosition)
    print("GLOBAL ORIENTATION: ", ROBOT_CFG.linkWorldPosition)
    print("LAST POSE: ", ROBOT_CFG.last_POSE)
    print("ROBOT GOAL: ", ROBOT_CFG.last_POSE)
    print("EE: ", ROBOT_CFG.EE)

    print("=========RUN GLOBAL VARS==========")
    print("STEP: ",  RUN.step)
    print("UPDATE: ", RUN.update)
    print("WAIT: ", RUN._wait)