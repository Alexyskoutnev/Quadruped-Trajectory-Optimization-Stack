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
        angle = np.deg2rad(angle)
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
                                0.15 , 
                                0.15 ,
                                0.15 ,
                                0.18 , 
                                0.18 ,
                                0.18 ,
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

    def stepTrajectory(self, t, velocity, angle, angle_velocity, stepOffset, footID): 
        if t >= 1.0:
            t = (t - 1.0)


        #a circumference to rotate around
        r = np.sqrt(self.robot.shift[footID][0]**2 + self.robot.shift[footID][1]**2)
        footAngle = np.arctan2(self.robot.shift[footID][0], self.robot.shift[footID][1])
        if angle_velocity >= 0:
            circleTrajectory = 90. - np.rad2deg(footAngle - self.alpha)
        else:
            circleTrajectory = 270. - np.rad2deg(footAngle - self.alpha)

        if t <= stepOffset: #stance phase
            T_stance = t/stepOffset
            stepX_pos, stepY_pos, stepZ_pos = self.calculateStance(T_stance, velocity, angle)
            stepX_rot, stepY_rot, stepZ_rot = self.calculateStance(T_stance, angle_velocity, circleTrajectory)
        else: #swing phase
            T_swing = (t - stepOffset)/(1 - stepOffset)
            stepX_pos, stepY_pos, stepZ_pos = self.calculateSwing(T_swing, velocity, angle)
            stepX_rot, stepY_rot, stepZ_rot = self.calculateSwing(T_swing, angle_velocity, circleTrajectory)

        if (self.robot.shift[footID][1] > 0):
            if (stepX_rot < 0):
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2), r)
            else:
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2), r)
        else:
            if (stepX_rot < 0):
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2), r)
            else:
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2), r)

        coord = np.empty(3)
        coord[0] = stepX_pos + stepX_rot
        coord[1] = stepY_pos + stepY_rot
        coord[2] = stepZ_pos + stepY_rot
        return coord

    def runTrajectory(self, velocity, angle, angle_velocity, offset, T, step_stance_ratio, hz = 1000):
        
        # if T <= 0.001:
        #     T = 0.001
        # if (abs(self.lastTime  - time.time()) < timestep):
        #     return self.gaitTraj
        #     # return None
        # else:
        #     self.t += timestep
        #     self.lastTime = time.time()
        #     self.cntTraj += 1
        # if (self.t >= 0.99):
        #     self.lastTime = time.time()
        #     self.t = 0   
        diff_t = (time.time() - self.lastTime)
        if (diff_t < T / hz):
            return self.gaitTraj
        else:
            time_itr = (time.time() - self.lastTime)/T
            self.t += time_itr
            self.lastTime = time.time()
            self.cntTraj += 1
        if (self.t >= 1.00):
            self.lastTime = time.time()
            self.t = 0.0

        assert(self.t >= 0.0)
        assert(self.t <= 1.0)

        #Front-left
        step_coord = self.stepTrajectory(self.t + offset[0], velocity, angle, angle_velocity,  step_stance_ratio, 'FL_FOOT')
        self.gaitTraj['FL_FOOT'][0] = step_coord[0]
        self.gaitTraj['FL_FOOT'][1] = step_coord[1]
        self.gaitTraj['FL_FOOT'][2] = step_coord[2]

        #Front-right
        step_coord = self.stepTrajectory(self.t + offset[1], velocity, angle, angle_velocity, step_stance_ratio, 'FR_FOOT')
        self.gaitTraj['FR_FOOT'][0] = step_coord[0]
        self.gaitTraj['FR_FOOT'][1] = step_coord[1]
        self.gaitTraj['FR_FOOT'][2] = step_coord[2]

        #Back-left
        step_coord = self.stepTrajectory(self.t + offset[2], velocity, angle, angle_velocity, step_stance_ratio, 'HL_FOOT')
        self.gaitTraj['HL_FOOT'][0] = step_coord[0] 
        self.gaitTraj['HL_FOOT'][1] = step_coord[1]
        self.gaitTraj['HL_FOOT'][2] = step_coord[2]

        #Back-right
        step_coord = self.stepTrajectory(self.t + offset[3], velocity, angle, angle_velocity, step_stance_ratio, 'HR_FOOT')
        self.gaitTraj['HR_FOOT'][0] = step_coord[0] 
        self.gaitTraj['HR_FOOT'][1] = step_coord[1]
        self.gaitTraj['HR_FOOT'][2] = step_coord[2]

        self.gaitTraj = trajectory_2_world_frame(self.robot, self.gaitTraj)
        return self.gaitTraj




        
if __name__ == "__main__":
    # pass
    # print("in Gait")
    # itr = 10000
    # t = np.linspace(0, 1, itr)
    # velocity = 1
    # X_FL_FOOT = list()
    # Y_FL_FOOT = list()
    # Z_FL_FOOT = list()
    # X_FR_FOOT = list()
    # Y_FR_FOOT = list()
    # Z_FR_FOOT = list()
    # X_HL_FOOT = list()
    # Y_HL_FOOT = list()
    # Z_HL_FOOT = list()
    # X_HR_FOOT = list()
    # Y_HR_FOOT = list()
    # Z_HR_FOOT = list()
    # URDF = "./data/urdf/solo12.urdf"
    # config = "./data/config/solo12.yml"
    # cfg = yaml.safe_load(open(config, 'r'))
    # # py_client = p.connect(p.GUI)
    # py_client = p.connect(p.DIRECT)
    # p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # p.setGravity(0,0,0)
    # p.setTimeStep(0.001) 
    # ROBOT = SOLO12(py_client, URDF, cfg)
    
    # gait = Gait(ROBOT)
    # angle = 30
    # offsets = np.array([0.5, 0.0, 0.0, 0.5])
    # angle_velocity = 0.0
    # T = 1.0
    # step_ratio = 0.5
    # while(True):
    #     # breakpoint()
    #     gait_traj = gait.runTrajectory(velocity, angle, angle_velocity, offsets, T, step_ratio)
    #     if gait_traj is not None:
    #         X_FL_FOOT.append(gait_traj['FL_FOOT'][0])
    #         Y_FL_FOOT.append(gait_traj['FL_FOOT'][1])
    #         Z_FL_FOOT.append(gait_traj['FL_FOOT'][2])
    #         X_FR_FOOT.append(gait_traj['FR_FOOT'][0])
    #         Y_FR_FOOT.append(gait_traj['FR_FOOT'][1])
    #         Z_FR_FOOT.append(gait_traj['HL_FOOT'][2])
    #         X_HL_FOOT.append(gait_traj['HL_FOOT'][0])
    #         Y_HL_FOOT.append(gait_traj['HL_FOOT'][1])
    #         Z_HL_FOOT.append(gait_traj['HL_FOOT'][2])
    #         X_HR_FOOT.append(gait_traj['HR_FOOT'][0])
    #         Y_HR_FOOT.append(gait_traj['HR_FOOT'][1])
    #         Z_HR_FOOT.append(gait_traj['HR_FOOT'][2])
    #     else:
    #         continue
    #     if gait.cntTraj == itr:
    #         break
    # breakpoint()
    # plt.plot(t, Z_FL_FOOT)
    # plt.show()
    pass