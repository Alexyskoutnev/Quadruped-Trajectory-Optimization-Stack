
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
import matplotlib.pyplot as plt

#project
from SOLO12_SIM_CONTROL.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv
from SOLO12_SIM_CONTROL.gaitPlanner import Gait
from SOLO12_SIM_CONTROL.simulation import Simulation
from SOLO12_SIM_CONTROL.utils import combine
from SOLO12_SIM_CONTROL.pybulletInterface import PybulletInterface

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
TRAJ = "./data/traj/gait.csv"
NUM_TIME_STEPS = 10000
TIMESTEPS = 0.001
HZ = 1000


def plot(t, joint):
    plt.plot(t, joint)
    plt.savefig("./traj_EE_pos.png")
    # plt.show()


if __name__ == "__main__":
    type = "traj"
    sim = Simulation(type)
    py_client = sim.setup(type)
    cfg = yaml.safe_load(open(config, 'r'))
    ROBOT = SOLO12(URDF, cfg, fixed=1)
    gait = Gait(ROBOT)
    # pybullet_interface = PybulletInterface()
    velocity, angle_velocity , angle, steps_per_sec = 0.5, 0, 0, 0.5
    offsets = np.array([0.5, 0.0, 0.0, 0.5])
    trot_2_stance_ratio = 0.5
    cmd = np.zeros((12, 1))
    itr = 0

    traj = []
    _t = np.linspace(0, NUM_TIME_STEPS/HZ, NUM_TIME_STEPS)

    with open(TRAJ, 'w', newline='') as file:
        writer = csv.writer(file) 
        while (itr < NUM_TIME_STEPS):
            gait_traj, updated = gait.runTrajectory(velocity, angle, angle_velocity, offsets, steps_per_sec, trot_2_stance_ratio, HZ, mode = "collect")
            if updated:
                traj.append(gait_traj['FL_FOOT']['P'][2])
                joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(gait_traj['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_ang_FL[0:3])
                joints_ang_FR, joints_vel_FR, joints_toq_FR = ROBOT.control(gait_traj['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joints_ang_FR[3:6])
                joints_ang_HL, joints_vel_HL, joints_toq_HL = ROBOT.control(gait_traj['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joints_ang_HL[6:9])
                joints_ang_HR, joints_vel_HR, joints_toq_HR = ROBOT.control(gait_traj['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joints_ang_HR[9:12])
                joint_ang = combine(joint_ang_FL, joints_ang_FR, joints_ang_HL, joints_ang_HR)
                joint_vel = combine(tuple(joint_vel_FL), tuple(joints_vel_FR), tuple(joints_vel_HL), tuple(joints_vel_HR))
                joint_toq = combine(joint_toq_FL, joints_toq_FR, joints_toq_HL, joints_toq_HR)
                csv_entry = np.hstack([joint_ang, joint_vel, joint_toq])
                writer.writerow(csv_entry)
                itr += 1
            else:
                continue
            p.stepSimulation()
    plot(_t, traj)
    p.disconnect()
    file.close()