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
config_sim = "./data/config/simulation_traj_record.yml"
cfg = yaml.safe_load(open(config, 'r'))
sim_cfg = yaml.safe_load(open(config_sim, 'r'))

URDF = "./data/urdf/solo12.urdf"
PLOTS = "../plots"
TRAJ = "./data/traj/gait.csv"
BEZIER = "./data/traj/bezier"
NUM_TIME_STEPS = sim_cfg['NUM_TIME_STEPS']
TIMESTEPS = sim_cfg['TIMESTEPS']
HZ = sim_cfg['HZ']

def update_file_name(file, cfg, sim_cfg):
    step_period = str(sim_cfg['step_period'])
    velocity = str(sim_cfg['velocity'])
    MODE = str(cfg['mode'])
    file_name = file + "_velocity_" + velocity + "_step_period_" + step_period + "_cmode_" + MODE + ".csv"
    return file_name

if __name__ == "__main__":

    itr = 0
    Simulation(sim_cfg['enviroment'], timestep=sim_cfg['TIMESTEPS'])
    ROBOT = SOLO12(URDF, cfg, fixed=sim_cfg['fix-base'], sim_cfg=sim_cfg)
    gait = Gait(ROBOT)
    _t = np.linspace(0, NUM_TIME_STEPS/HZ, NUM_TIME_STEPS)
    offsets = np.array(cfg['offsets'])
    trot_2_stance_ratio = cfg['trot_2_stance_ratio']
    velocity, angle, angle_velocity, step_period = sim_cfg['velocity'], sim_cfg['angle_velocity'], sim_cfg['angle'], sim_cfg['step_period']
    FILE = update_file_name(BEZIER, cfg, sim_cfg)

    with open(FILE, 'w', newline='') as file:
        writer = csv.writer(file) 
        while (itr < NUM_TIME_STEPS):
            if sim_cfg['mode'] == "bezier":
                gait_traj, updated = gait.runTrajectory(velocity, angle, angle_velocity, offsets, step_period, trot_2_stance_ratio, HZ, mode = "sim")
                if updated:
                    gait_traj, newCmd = gait.runTrajectory(velocity, angle, angle_velocity, offsets, step_period, trot_2_stance_ratio)
                    joint_ang, joint_vel, joint_toq = ROBOT.control_multi(gait_traj, ROBOT.EE_index['all'], mode=ROBOT.mode)
                    ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
                    if ROBOT.mode == 'P':
                        joint_vel = np.zeros(12)
                        joint_toq = np.zeros(12)
                    elif ROBOT.mode == 'PD':
                        joint_toq = np.zeros(12)
                    elif ROBOT.mode == "torque":
                        pass
                    csv_entry = np.hstack([joint_ang, joint_vel, joint_toq])
                    assert(csv_entry.shape[0] == 36)
                    writer.writerow(csv_entry)
                    itr += 1
                    p.stepSimulation()
                ROBOT.time_step += 1
                if (itr % 1000 == 0):
                    print(f"Collected [{itr}] / [{NUM_TIME_STEPS}]")

    print("DONE")
    p.disconnect()
    file.close()