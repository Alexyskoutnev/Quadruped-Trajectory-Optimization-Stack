
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
from SOLO12_SIM_CONTROL.robot.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv
from SOLO12_SIM_CONTROL.gaitPlanner import Gait
from SOLO12_SIM_CONTROL.simulation import Simulation
from SOLO12_SIM_CONTROL.utils import combine
from SOLO12_SIM_CONTROL.pybulletInterface import PybulletInterface

config = "./data/config/solo12.yml"
config_sim = "./data/config/simulation.yml"
cfg = yaml.safe_load(open(config, 'r'))
sim_cfg = yaml.safe_load(open(config_sim, 'r'))

URDF = "./data/urdf/solo12.urdf"
PLOTS = "../plots"
TRAJ = "./data/traj/gait.csv"
TOWR = "./data/traj/towr.csv"
NUM_TIME_STEPS = sim_cfg['NUM_TIME_STEPS']
TIMESTEPS = sim_cfg['TIMESTEPS']
HZ = sim_cfg['HZ']

def plot(t, *joints):
    str_ = {0: "FL", 1: "FR", 2: "HL", 3: "HL"}
    for i in range(len(joints)):
        breakpoint()
        plt.plot(t, joints[i])
        plt.savefig("/traj_" + str_[i] +  "_pos_FL.png")
        plt.close()
    # plt.show()


if __name__ == "__main__":
    Simulation(sim_cfg['enviroment'], timestep=sim_cfg['TIMESTEPS'])
    ROBOT = SOLO12(URDF, cfg, fixed=sim_cfg['fix-base'], sim_cfg=sim_cfg)
    gait = Gait(ROBOT)
    velocity, angle_velocity , angle, steps_per_sec = sim_cfg['velocity'], sim_cfg['angle_velocity'], sim_cfg['angle'], sim_cfg['step_per_sec']
    offsets = np.array(sim_cfg['offsets'])
    trot_2_stance_ratio = sim_cfg['trot_2_stance_ratio']
    itr = 0
    traj_FL, traj_FR, traj_HL, traj_HR = [], [], [], []

    if sim_cfg['mode'] == "towr":
        csv_file = open(TOWR, 'r', newline='')
        reader = csv.reader(csv_file, delimiter=',')
        NUM_TIME_STEPS = sum(1 for row in reader)
        csv_file = open(TOWR, 'r', newline='')
        reader = csv.reader(csv_file, delimiter=',')
        _t = np.linspace(0, NUM_TIME_STEPS/HZ, NUM_TIME_STEPS)
    if sim_cfg['mode'] == 'bezier':
        _t = np.linspace(0, NUM_TIME_STEPS/HZ, NUM_TIME_STEPS)

    with open(TRAJ, 'w', newline='') as file:
        writer = csv.writer(file) 
        while (itr < NUM_TIME_STEPS):
            if sim_cfg['mode'] == "bezier":
                gait_traj, updated = gait.runTrajectory(velocity, angle, angle_velocity, offsets, steps_per_sec, trot_2_stance_ratio, HZ, mode = "sim")
                if updated:
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(gait_traj['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    if ROBOT.mode == 'P' or ROBOT.mode == 'PD':
                        ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_ang_FL[0:3])
                    elif ROBOT.mode == 'torque':
                        ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_toq_FL[0:3])
                    joints_ang_FR, joints_vel_FR, joints_toq_FR = ROBOT.control(gait_traj['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    if ROBOT.mode == 'P' or ROBOT.mode == 'PD':
                        ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joints_ang_FR[3:6])
                    elif ROBOT.mode == 'torque':
                        ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joints_toq_FR[3:6])
                    joints_ang_HL, joints_vel_HL, joints_toq_HL = ROBOT.control(gait_traj['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    if ROBOT.mode == 'P' or ROBOT.mode == 'PD':
                        ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joints_ang_HL[6:9])
                    elif ROBOT.mode == 'torque':
                            ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joints_toq_HL[6:9])
                    joints_ang_HR, joints_vel_HR, joints_toq_HR = ROBOT.control(gait_traj['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
                    if ROBOT.mode == 'P' or ROBOT.mode == 'PD':
                        ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joints_ang_HR[9:12])
                    elif ROBOT.mode == 'torque':
                        ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joints_toq_HR[9:12])
                    joint_ang = combine(joint_ang_FL, joints_ang_FR, joints_ang_HL, joints_ang_HR)
                    if ROBOT.mode == 'P':
                        joint_vel = np.zeros(12)
                        joint_toq = np.zeros(12)
                    elif ROBOT.mode == 'PD':
                        joint_vel = combine(tuple(joint_vel_FL), tuple(joints_vel_FR), tuple(joints_vel_HL), tuple(joints_vel_HR))
                        joint_toq = combine(joint_toq_FL, joints_toq_FR, joints_toq_HL, joints_toq_HR)
                    csv_entry = np.hstack([joint_ang, joint_vel, joint_toq])
                    writer.writerow(csv_entry)
                    itr += 1
                    p.stepSimulation()
                ROBOT.time_step += 1
            elif sim_cfg['mode'] == "towr":
                with open(TOWR, 'r', newline='') as csv_file:
                    try: 
                        EE_POSE = [float(x) for x in next(reader)]
                    except StopIteration:
                        break
                    towr = {"FL_FOOT": {'P' : EE_POSE[0:3], 'D': np.zeros(3)}, "FR_FOOT": {'P': EE_POSE[3:6], 'D': np.zeros(3)}, 
                            "HL_FOOT": {'P': EE_POSE[6:9], 'D' : np.zeros(3)}, "HR_FOOT": {'P': EE_POSE[9:12], 'D': np.zeros(3)}}
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(towr['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_ang_FL[0:3])
                    joints_ang_FR, joints_vel_FR, joints_toq_FR = ROBOT.control(towr['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joints_ang_FR[3:6])
                    joints_ang_HL, joints_vel_HL, joints_toq_HL = ROBOT.control(towr['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joints_ang_HL[6:9])
                    joints_ang_HR, joints_vel_HR, joints_toq_HR = ROBOT.control(towr['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
                    ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joints_ang_HR[9:12])
                    joint_ang = combine(joint_ang_FL, joints_ang_FR, joints_ang_HL, joints_ang_HR)
                    if ROBOT.mode == 'P':
                        joint_vel = np.zeros(12)
                        joint_toq = np.zeros(12)
                    elif ROBOT.mode == 'PD':
                        joint_vel = combine(tuple(joint_vel_FL), tuple(joints_vel_FR), tuple(joints_vel_HL), tuple(joints_vel_HR))
                        joint_toq = combine(joint_toq_FL, joints_toq_FR, joints_toq_HL, joints_toq_HR)
                    csv_entry = np.hstack([joint_ang, joint_vel, joint_toq])
                    writer.writerow(csv_entry)
                    itr += 1
    # plot(_t, traj_FL, traj_FR, traj_HL, traj_HR)
    p.disconnect()
    file.close()