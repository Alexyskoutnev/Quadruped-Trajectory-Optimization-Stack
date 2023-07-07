
import time
import os
import sys
import math
import csv
from threading import Thread, Lock

#third party
import pybullet as p
import pybullet_data
import yaml
import numpy as np
import matplotlib.pyplot as plt

#project
from SOLO12_SIM_CONTROL.robot.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import *
from SOLO12_SIM_CONTROL.pybulletInterface import PybulletInterface
from SOLO12_SIM_CONTROL.simulation import Simulation
from SOLO12_SIM_CONTROL.logger import Logger
import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg

config = "./data/config/solo12.yml"
config_sim = "./data/config/simulation_traj_record.yml"
cfg = yaml.safe_load(open(config, 'r'))
sim_cfg = yaml.safe_load(open(config_sim, 'r'))

URDF = "./data/urdf/solo12.urdf"
PLOTS = "../plots"
TRAJ = "./data/traj/gait.csv"
TOWR = "./data/traj/towr"
BEZIER = "./data/traj/bezier"
NUM_TIME_STEPS = sim_cfg['NUM_TIME_STEPS']
TIMESTEPS = sim_cfg['TIMESTEPS']
HZ = sim_cfg['HZ']

mutex = Lock()

def plot(t, *joints):
    str_ = {0: "FL", 1: "FR", 2: "HL", 3: "HL"}
    for i in range(len(joints)):
        breakpoint()
        plt.plot(t, joints[i])
        plt.savefig("/traj_" + str_[i] +  "_pos_FL.png")
        plt.close()
    # plt.show()

def update_file_name(file, cfg, sim_cfg):
    step_period = str(sim_cfg['step_period'])
    velocity = str(sim_cfg['velocity'])
    MODE = str(cfg['mode'])
    file_name = file + "_velocity_" + velocity + "_step_period_" + step_period + "_cmode_" + MODE + ".csv"
    return file_name

def _global_update(ROBOT, kwargs):
    global_cfg.ROBOT_CFG.robot = ROBOT
    global_cfg.ROBOT_CFG.linkWorldPosition = list(kwargs['COM'])
    global_cfg.ROBOT_CFG.linkWorldOrientation = list(p.getEulerFromQuaternion(kwargs['linkWorldOrientation']))
    global_cfg.ROBOT_CFG.EE['FL_FOOT'] = list(kwargs['FL_FOOT'])
    global_cfg.ROBOT_CFG.EE['FR_FOOT'] = list(kwargs['FR_FOOT'])
    global_cfg.ROBOT_CFG.EE['HL_FOOT'] = list(kwargs['HL_FOOT'])
    global_cfg.ROBOT_CFG.EE['HR_FOOT'] = list(kwargs['HR_FOOT'])
    global_cfg.ROBOT_CFG.runtime = ROBOT.time
    global_cfg.RUN.step += 1
    global_cfg.ROBOT_CFG.joint_state = ROBOT.jointstate

def simulation():
    itr = 0
    Simulation(sim_cfg['enviroment'], timestep=sim_cfg['TIMESTEPS'])
    ROBOT = SOLO12(URDF, cfg, fixed=sim_cfg['fix-base'], sim_cfg=sim_cfg)
    csv_file = open(TOWR, 'r', newline='')
    reader = csv.reader(csv_file, delimiter=',')
    NUM_TIME_STEPS = sum(1 for row in reader)
    csv_file = open(TOWR, 'r', newline='')
    reader = csv.reader(csv_file, delimiter=',')
    _t = np.linspace(0, NUM_TIME_STEPS/HZ, NUM_TIME_STEPS)
    FILE = update_file_name(TRAJ, cfg, sim_cfg)

    with open(FILE, 'w', newline='') as file:
        writer = csv.writer(file) 
        while (itr < NUM_TIME_STEPS):
            with open(TOWR, 'r', newline='') as csv_file:
                try:
                    if global_cfg.RUN._wait: #Waits for towr thread to copy over the trajectory
                        time.sleep(0.001)
                        continue
                    elif global_cfg.RUN._update:
                        print("============UPDATE STATE============")
                        mutex.acquire()
                        reader = csv.reader(open(TOWR, 'r', newline=''))
                        mutex.release()
                        global_cfg.RUN._update = False 
                        global_cfg.RUN.step = 0
                    EE_POSE = np.array([float(x) for x in next(reader)])[1:]
                    global_cfg.ROBOT_CFG.last_POSE = EE_POSE[0:3]
                    global_cfg.RUN.TOWR_POS = EE_POSE[0:3]
                    towr_traj = towr_transform(ROBOT, vec_to_cmd_pose(EE_POSE))
                except StopIteration:
                    pass
                    global_cfg.RUN._stance = True

                if global_cfg.RUN._stance:
                    _, _, joint_toq = ROBOT.default_stance_control()
                    p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=joint_toq)
                else:
                    joint_ang, joint_vel, joint_toq = ROBOT.control_multi(towr_traj, ROBOT.EE_index['all'], mode=ROBOT.mode)
                    ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
    
                csv_entry = np.hstack([joint_ang, joint_vel, joint_toq])
                writer.writerow(csv_entry)
                itr += 1

                p.stepSimulation()
                ROBOT.time_step += 1
                _global_update(ROBOT, ROBOT.state)

            if (itr % 1000 == 0):
                print(f"Collected [{itr}] / [{NUM_TIME_STEPS}]")


    print("DONE")
    p.disconnect()
    file.close()

if __name__ == "__main__":
    pass