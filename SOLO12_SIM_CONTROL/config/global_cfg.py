class ROBOT_CFG:
    linkWorldPosition = [0, 0, 0.21]
    linkWorldOrientation = [0, 0, 0]
    last_POSE = [0, 0, 0]
    robot_goal = [0, 0, 0]
    EE = {"FL": [0.09,0.07,0], "FR": [0.09,-0.07,0], "HL": [-0.09,0.07,0] , "HR": [-0.09, -0.07,0]}

class RUN:
    update = False
