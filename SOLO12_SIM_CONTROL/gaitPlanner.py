import time
import numpy as np
import matplotlib.pyplot as plt

import yaml 
import pybullet as p
import pybullet_data

from SOLO12_SIM_CONTROL.utils import trajectory_2_world_frame
from SOLO12_SIM_CONTROL.robot.robot import SOLO12

def binomial_factor(n, k):
    return np.math.factorial(n) / (np.math.factorial(k)*np.math.factorial(n - k))

def bezier(t, k, p):
    n = 9
    return p*binomial_factor(n, k)*np.power(t, k)*np.power(1 - t, n - k)

def bezier_d_1(t, k, p_i, p_i_1):
    n = 8
    b_i = binomial_factor(n, k)*np.power(t, k)*np.power(1 - t, n - k)
    return (n + 1)*b_i *(p_i_1 - p_i)

def bezier_d_2(t, k, p):
    pass

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
        self.gaitTraj = {'FL_FOOT': {"P": np.zeros(3), "D": np.zeros(3), "T": np.zeros(3)}, 'FR_FOOT': {"P": np.zeros(3), "D": np.zeros(3), "T": np.zeros(3)},
                        "HL_FOOT": {"P": np.zeros(3), "D": np.zeros(3), "T": np.zeros(3)}, "HR_FOOT": {"P": np.zeros(3), "D": np.zeros(3), "T": np.zeros(3)}}
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
        return stanceX, stanceY, stanceZ, 0, 0, 0   #might need to change the derivative of stance

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
        swingX_d = 0.0
        swingY_d = 0.0
        swingZ_d = 0.0

        for i in range(10): #Bezier Curve Computation
            swingX += bezier(t, i, X_pts[i])
            swingY += bezier(t, i, Y_pts[i])
            swingZ += bezier(t, i, Z_pts[i])
        
        for i in range(9): #Derivative Bezier Curve
            swingX_d += bezier_d_1(t, i, X_pts[i + 1], X_pts[i])
            swingY_d += bezier_d_1(t, i, Y_pts[i + 1], Y_pts[i])
            swingZ_d += bezier_d_1(t, i, Z_pts[i + 1], Z_pts[i])
        
        return swingX, swingY, swingZ, swingX_d, swingY_d, swingZ_d

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
            stepX_pos, stepY_pos, stepZ_pos, stepX_pos_d, stepY_pos_d, stepZ_pos_d = self.calculateStance(T_stance, velocity, angle)
            stepX_rot, stepY_rot, stepZ_rot, stepX_rot_d, stepY_rot_d, stepZ_rot_d = self.calculateStance(T_stance, angle_velocity, circleTrajectory)
        else: #swing phase
            T_swing = (t - stepOffset)/(1 - stepOffset)
            stepX_pos, stepY_pos, stepZ_pos, stepX_pos_d, stepY_pos_d, stepZ_pos_d = self.calculateSwing(T_swing, velocity, angle)
            stepX_rot, stepY_rot, stepZ_rot, stepX_rot_d, stepY_rot_d, stepZ_rot_d = self.calculateSwing(T_swing, angle_velocity, circleTrajectory)

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

        coord = np.empty(6)
        coord[0] = stepX_pos + stepX_rot
        coord[1] = stepY_pos + stepY_rot
        coord[2] = stepZ_pos + stepY_rot
        coord[3] = stepX_pos_d + stepX_rot_d
        coord[4] = stepY_pos_d + stepY_rot_d
        coord[5] = stepZ_pos_d + stepZ_rot_d
        return coord

    def runTrajectory(self, velocity, angle, angle_velocity, offset, T, step_stance_ratio, hz = 1000, mode = "sim"): 
        if mode == "collect":
            self.t += T/hz
            if (self.t >= 1.00):
                self.t = 0.0
        elif mode == "sim":
            diff_t = (time.time() - self.lastTime)
            if (diff_t < T / hz):
                return self.gaitTraj, False
            else:
                time_itr = (time.time() - self.lastTime)/T
                self.t += time_itr
                self.t += T/hz
                self.lastTime = time.time()
                self.cntTraj += 1
            if (self.t >= 1.00):
                self.lastTime = time.time()
                self.t = 0.0

        assert(self.t >= 0.0)
        assert(self.t <= 1.0)

        #Front-left
        step_coord = self.stepTrajectory(self.t + offset[0], velocity, angle, angle_velocity,  step_stance_ratio, 'FL_FOOT')
        self.gaitTraj['FL_FOOT']['P'][0] = step_coord[0]
        self.gaitTraj['FL_FOOT']['P'][1] = step_coord[1]
        self.gaitTraj['FL_FOOT']['P'][2] = step_coord[2]
        self.gaitTraj['FL_FOOT']['D'][0] = step_coord[3]
        self.gaitTraj['FL_FOOT']['D'][1] = step_coord[4]
        self.gaitTraj['FL_FOOT']['D'][2] = step_coord[5]

        #Front-right
        step_coord = self.stepTrajectory(self.t + offset[1], velocity, angle, angle_velocity, step_stance_ratio, 'FR_FOOT')
        self.gaitTraj['FR_FOOT']['P'][0] = step_coord[0]
        self.gaitTraj['FR_FOOT']['P'][1] = step_coord[1]
        self.gaitTraj['FR_FOOT']['P'][2] = step_coord[2]
        self.gaitTraj['FR_FOOT']['D'][0] = step_coord[3]
        self.gaitTraj['FR_FOOT']['D'][1] = step_coord[4]
        self.gaitTraj['FR_FOOT']['D'][2] = step_coord[5]

        #Back-left
        step_coord = self.stepTrajectory(self.t + offset[2], velocity, angle, angle_velocity, step_stance_ratio, 'HL_FOOT')
        self.gaitTraj['HL_FOOT']['P'][0] = step_coord[0] 
        self.gaitTraj['HL_FOOT']['P'][1] = step_coord[1]
        self.gaitTraj['HL_FOOT']['P'][2] = step_coord[2]
        self.gaitTraj['HL_FOOT']['D'][0] = step_coord[3]
        self.gaitTraj['HL_FOOT']['D'][1] = step_coord[4]
        self.gaitTraj['HL_FOOT']['D'][2] = step_coord[5]

        #Back-right
        step_coord = self.stepTrajectory(self.t + offset[3], velocity, angle, angle_velocity, step_stance_ratio, 'HR_FOOT')
        self.gaitTraj['HR_FOOT']['P'][0] = step_coord[0] 
        self.gaitTraj['HR_FOOT']['P'][1] = step_coord[1]
        self.gaitTraj['HR_FOOT']['P'][2] = step_coord[2]
        self.gaitTraj['HR_FOOT']['D'][0] = step_coord[3]
        self.gaitTraj['HR_FOOT']['D'][1] = step_coord[4]
        self.gaitTraj['HR_FOOT']['D'][2] = step_coord[5]
        self.gaitTraj = trajectory_2_world_frame(self.robot, self.gaitTraj, shift_bezier=True)

        return self.gaitTraj, True