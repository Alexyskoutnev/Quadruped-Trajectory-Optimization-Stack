
import time
import os
import sys
import math
import csv

#third party
import pybullet as p
import pybullet_data
import yaml
import numpy as np

#project
from SOLO12_SIM_CONTROL.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv
from SOLO12_SIM_CONTROL.gaitPlanner import Gait
from SOLO12_SIM_CONTROL.simulation import Simulation
from SOLO12_SIM_CONTROL.utils import combine

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
TRAJ = "./data/traj/gait.csv"
NUM_TIME_STEPS = 10000
TIMESTEPS = 0.001

if __name__ == "__main__":
    type = "traj"
    sim = Simulation(type)
    py_client = sim.setup(type)
    cfg = yaml.safe_load(open(config, 'r'))
    ROBOT = SOLO12(URDF, cfg)
    gait = Gait(ROBOT)
    velocity, angle_velocity , angle,  stepPeriod = 1, 0, 0, 2.0
    offsets = np.array([0.5, 0.0, 0.0, 0.5])
    trot_2_stance_ratio = 0.5
    cmd = np.zeros((12, 1))
    t = 0
    with open(TRAJ, 'w', newline='') as file:
        writer = csv.writer(file) 
        for i in range (NUM_TIME_STEPS):
            gait_traj = gait.runTrajectory(velocity, angle, angle_velocity, offsets, stepPeriod, trot_2_stance_ratio)
            joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(gait_traj['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
            ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_ang_FL[0:3])
            joints_ang_FR, joints_vel_FR, joints_toq_FR = ROBOT.control(gait_traj['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
            ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joints_ang_FR[3:6])
            joints_ang_HL, joints_vel_HL, joints_toq_HL = ROBOT.control(gait_traj['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
            ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joints_ang_HL[6:9])
            joints_ang_HR, joints_vel_HR, joints_toq_HR = ROBOT.control(gait_traj['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
            ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joints_ang_HR[9:12])
            joint_ang = combine(joint_ang_FL, joints_ang_FR, joints_ang_HL, joints_ang_HR)
            joint_vel = combine(joint_vel_FL, joints_vel_FR, joints_vel_HL, joints_vel_HR)
            joint_toq = combine(joint_toq_FL, joints_toq_FR, joints_toq_HL, joints_toq_HR)
            csv_entry = np.hstack([t, joint_ang, joint_vel, joint_toq])
            writer.writerow(csv_entry)
            t += TIMESTEPS
    p.disconnect()
    file.close()