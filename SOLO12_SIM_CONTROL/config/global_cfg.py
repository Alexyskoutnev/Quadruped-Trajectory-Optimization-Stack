class ROBOT_CFG:
    robot = None
    linkWorldPosition = [0, 0, 0]
    linkWorldOrientation = [0, 0, 0]
    last_POSE = [0, 0, 0]
    robot_goal = [0, 0, 0]
    EE = {"FL_FOOT": [0.09,0.07,0], "FR_FOOT": [0.09,-0.07,0], "HL_FOOT": [-0.09,0.07,0] , "HR_FOOT": [-0.09, -0.07,0]}

class RUN:
    update = False
