import time
import numpy as np
import matplotlib.pyplot as plt


import yaml 
import pybullet as p
import pybullet_data

from SOLO12_SIM_CONTROL.utils import trajectory_2_world_frame
from SOLO12_SIM_CONTROL.robot import SOLO12

def binomial_factor(n, k):
    return np.math.factorial(n) / (np.math.factorial(k)*np.math.factorial(n - k))

def bezier(t, k, p):
    n = 9
    return p*binomial_factor(n, k)*np.power(t, k)*np.power(1 - t, n - k)

def plot(t, val):
    plt.plot(t, val)



class Gait(object):

    def __init__(self, robot):
        self.robot = robot
        self.feetPose = {"FL": np.zeros(3), "FR": np.zeros(3), "BL": np.zeros(3), "BR": np.zeros(3)}
        self.t = 0
        self.phiStance = 0
        self.lastTime = time.time()
        self.alpha = 0
        self.s = False
        self.offset = np.array([0, 0.5, 0.5, 0]) #(FR, FL, BR, BL)
        self.gaitTraj = {'FL_FOOT': np.zeros(3), 'FR_FOOT': np.zeros(3), 'HL_FOOT': np.zeros(3), 'HR_FOOT': np.zeros(3)}
        self.cntTraj = 0

    def calculateStance(self, t, V, angle):
        c = np.cos(angle)
        s = np.sin(angle)
        A = 0.0005
        halfStance =0.05
        p_stance = halfStance*(1 - 2*t)
        stanceX = c * halfStance*(1 - 2*t) * np.abs(V)
        stanceY = -s * halfStance*(1 - 2*t) * np.abs(V)
        stanceZ = -A * np.cos(np.pi/(2 * halfStance)*p_stance)
        return stanceX, stanceY, stanceZ

    def calculateSwing(self, t, velocity, angle):
        
        X_pts = np.abs(velocity) * np.cos(angle) * np.array([-0.05 ,
                                  -0.06 ,
                                  -0.07 , 
                                  -0.07 ,
                                  0. ,
                                  0. , 
                                  0.07 ,
                                  0.07 ,
                                  0.06 ,
                                  0.05 ])
        Y_pts = np.abs(velocity) * np.sin(angle) * np.array([ 0.05 ,
                                   0.06 ,
                                   0.07 , 
                                   0.07 ,
                                   0. ,
                                   -0. , 
                                   -0.07 ,
                                   -0.07 ,
                                   -0.06 ,
                                   -0.05 ])
        Z_pts = np.abs(velocity) * np.array([0. ,
                                0. ,
                                0.05 , 
                                0.05 ,
                                0.05 ,
                                0.06 , 
                                0.06 ,
                                0.06 ,
                                0. ,
                                0. ])

        swingX = 0.0
        swingY = 0.0
        swingZ = 0.0

        for i in range(10): #Bezier Curve Computation
            swingX += bezier(t, i, X_pts[i])
            swingY += bezier(t, i, Y_pts[i])
            swingZ += bezier(t, i, Z_pts[i])
        
        return swingX, swingY, swingZ

    def stepTrajectory(self, t, velocity, angle, stepOffset): 
        if t >= 1.0:
            t = (t - 1.0)
        print(f"t: -> {t}")
        if t <= stepOffset:
            T_stance = t/stepOffset
            stepX_pos, stepY_pos, stepZ_pos = self.calculateStance(T_stance, velocity, angle)
            # stepX_pos, stepY_pos, stepZ_pos = 0, 0, 0

            # stepX_rot, stepY_rot, stepZ_rot = self.calculateStance(phiStance, angle_vel, )
        else:
            T_swing = (t - stepOffset)/(1 - stepOffset)
            stepX_pos, stepY_pos, stepZ_pos = self.calculateSwing(t, velocity, angle)
        # breakpoint()
        coord = np.empty(3)
        coord[0] = stepX_pos
        coord[1] = stepY_pos
        coord[2] = stepZ_pos
        return coord

    def runTrajectory(self, velocity, angle, offset, T, timestep=0.01):
        if T <= 0.001:
            T = 0.001
        if (abs(self.lastTime  - time.time()) < timestep):
            return self.gaitTraj
            # return None
        else:
            self.t += timestep
            self.lastTime = time.time()
            self.cntTraj += 1
        if (self.t >= 0.99):
            self.lastTime = time.time()
            self.t = 0   

        assert(self.t >= 0.0)
        assert(self.t < 1.0)

        #Front-left
        step_coord = self.stepTrajectory(self.t + offset[0], velocity, angle, T)
        self.gaitTraj['FL_FOOT'][0] = step_coord[0]
        # self.gaitTraj['FL_FOOT'][0] = 0.14
        self.gaitTraj['FL_FOOT'][1] = step_coord[1]
        # self.gaitTraj['FL_FOOT'][1] = step_coord[1]
        self.gaitTraj['FL_FOOT'][2] = step_coord[2] * 2

        #Front-right
        step_coord = self.stepTrajectory(self.t + offset[1], velocity, angle, T)
        self.gaitTraj['FR_FOOT'][0] = step_coord[0]
        # self.gaitTraj['FR_FOOT'][0] = 
        self.gaitTraj['FR_FOOT'][1] = step_coord[1]
        # self.gaitTraj['FR_FOOT'][1] = 0
        self.gaitTraj['FR_FOOT'][2] = step_coord[2]  * 2

        #Back-left
        step_coord = self.stepTrajectory(self.t + offset[2], velocity, angle, T)
        self.gaitTraj['HL_FOOT'][0] = step_coord[0] 
        self.gaitTraj['HL_FOOT'][1] = step_coord[1]
        # self.gaitTraj['HL_FOOT'][0] = 0
        # self.gaitTraj['HL_FOOT'][1] = 0
        self.gaitTraj['HL_FOOT'][2] = step_coord[2] * 2

        #Back-right
        step_coord = self.stepTrajectory(self.t + offset[3], velocity, angle, T)
        self.gaitTraj['HR_FOOT'][0] = step_coord[0] 
        # self.gaitTraj['HR_FOOT'][0] = 0
        self.gaitTraj['HR_FOOT'][1] = step_coord[1]
        # self.gaitTraj['HR_FOOT'][1] = 0
        self.gaitTraj['HR_FOOT'][2] = step_coord[2] * 2

        # breakpoint()
        # self.gaitTraj = self.gaitTraj.values()*10
        # breakpoint()
        # print(f"t: {self.t}")
        print(f"Before Transformation -> {self.gaitTraj}")
        self.gaitTraj = trajectory_2_world_frame(self.robot, self.gaitTraj)
        print(f"After Transformation -> {self.gaitTraj}")
        return self.gaitTraj




        
if __name__ == "__main__":
    itr = 5000
    t = np.linspace(0, 1, itr)
    velocity = 1
    X_FL_FOOT = list()
    Y_FL_FOOT = list()
    Z_FL_FOOT = list()
    X_FR_FOOT = list()
    Y_FR_FOOT = list()
    Z_FR_FOOT = list()
    X_HL_FOOT = list()
    Y_HL_FOOT = list()
    Z_HL_FOOT = list()
    X_HR_FOOT = list()
    Y_HR_FOOT = list()
    Z_HR_FOOT = list()
    URDF = "./data/urdf/solo12.urdf"
    config = "./data/config/solo12.yml"
    cfg = yaml.safe_load(open(config, 'r'))
    # py_client = p.connect(p.GUI)
    py_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,0)
    p.setTimeStep(0.001) 
    ROBOT = SOLO12(py_client, URDF, cfg)
    
    gait = Gait(ROBOT)
    angle = 0
    offsets = np.array([0.0, 0.0, 0.0, 0.0])
    # for _ in t:
    #     _X, _Y, _Z = gait.calculateSwing(_, 1, np.pi/3)
    #     X.append(_X)
    #     Y.append(_Y)
    #     Z.append(_Z)
    

    # T = np.linspace(0, 1, 100)
    while(True):
    # if
        gait_traj = gait.runTrajectory(velocity, angle, offsets, 0.50, timestep=0.001)
        if gait_traj is not None:
            X_FL_FOOT.append(gait_traj['FL_FOOT'][0])
            Y_FL_FOOT.append(gait_traj['FL_FOOT'][1])
            Z_FL_FOOT.append(gait_traj['FL_FOOT'][2])
            X_FR_FOOT.append(gait_traj['FR_FOOT'][0])
            Y_FR_FOOT.append(gait_traj['FR_FOOT'][1])
            Z_FR_FOOT.append(gait_traj['HL_FOOT'][2])
            X_HL_FOOT.append(gait_traj['HL_FOOT'][0])
            Y_HL_FOOT.append(gait_traj['HL_FOOT'][1])
            Z_HL_FOOT.append(gait_traj['HL_FOOT'][2])
            X_HR_FOOT.append(gait_traj['HR_FOOT'][0])
            Y_HR_FOOT.append(gait_traj['HR_FOOT'][1])
            Z_HR_FOOT.append(gait_traj['HR_FOOT'][2])
        else:
            continue
        if gait.cntTraj == itr:
            break
    breakpoint()