#! /opt/homebrew/Caskroom/miniforge/base/envs/soloSim/bin/python3

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

#project
from SOLO12_SIM_CONTROL.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv, towr_transform, vec_to_cmd, vec_to_cmd_pose, nearestPoint, trajectory_2_world_frame
from SOLO12_SIM_CONTROL.gaitPlanner import Gait
from SOLO12_SIM_CONTROL.pybulletInterface import PybulletInterface
from SOLO12_SIM_CONTROL.simulation import Simulation
from SOLO12_SIM_CONTROL.logger import Logger
import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
config_sim = "./data/config/simulation.yml"
cfg = yaml.safe_load(open(config, 'r'))
sim_cfg = yaml.safe_load(open(config_sim, 'r'))
TOWR = "./data/traj/towr.csv"
TRACK = "./data/traj/traj_thirdparty/jointStates.csv"
HZ = sim_cfg['HZ']

# global keypressed
key_press_init_phase = True
mutex = Lock()


def setup_enviroment():
    py_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    return py_client 

def _global_update(ROBOT, kwargs):
    global_cfg.ROBOT_CFG.robot = ROBOT
    global_cfg.ROBOT_CFG.linkWorldPosition = list(kwargs['COM'])
    global_cfg.ROBOT_CFG.linkWorldOrientation = list(p.getEulerFromQuaternion(kwargs['linkWorldOrientation']))
    global_cfg.ROBOT_CFG.EE['FL_FOOT'] = list(kwargs['FL_FOOT'])
    global_cfg.ROBOT_CFG.EE['FR_FOOT'] = list(kwargs['FR_FOOT'])
    global_cfg.ROBOT_CFG.EE['HL_FOOT'] = list(kwargs['HL_FOOT'])
    global_cfg.ROBOT_CFG.EE['HR_FOOT'] = list(kwargs['HR_FOOT'])

def keypress():
    global key_press_init_phase
    while True:
        print("Press q to exit")
        val = input('Enter your input: ')
        if val == 'q':
            print("Moving to trajectory")
            key_press_init_phase = False
            break

def swtich(timestep):
    FL, FR, HL, HR = False, False, False, False
    balance = True
    if timestep > 250 and timestep < 500:
        balance = False
    elif timestep >= 750 and timestep < 1000:
        balance = False
    elif timestep >= 1250 and timestep < 1500:
        balance = False
    elif timestep >= 1750 and timestep < 2000:
        balance = False
    if timestep > 0 and timestep < 500:
        FL = True
        FR = False
        HL = False 
        HR = False
    elif timestep >= 500 and timestep < 1000:
        FL = False
        FR = True
        HL = False 
        HR = False
    elif timestep >= 1000 and timestep < 1500:
        FL = False
        FR = False
        HL = True 
        HR = False
    elif timestep >= 1500 and timestep < 2000:
        FL = False
        FR = False
        HL = False 
        HR = True
    return FL, FR, HL, HR, balance

def simulation():
    log = Logger("./logs", "simulation_log")
    global key_press_init_phase
    Simulation(sim_cfg['enviroment'], timestep=sim_cfg['TIMESTEPS'])
    ROBOT = SOLO12(URDF, cfg, fixed=sim_cfg['fix-base'], sim_cfg=sim_cfg)
    gait = Gait(ROBOT)
    init_phase = sim_cfg['stance_phase']
    keypress_io = Thread(target=keypress)
    stance = False
    # flip_pos = trajectory_2_world_frame(ROBOT, {"FL_FOOT": {"P": [0.2, 0.25, 0.25], "D": np.zeros(3)}, "FR_FOOT": {"P": [0.2, -0.25, 0.25], "D": np.zeros(3)}, 
    #                 "HL_FOOT": {"P": [-0.2, 0.25, 0.25], "D": np.zeros(3)}, "HR_FOOT": {"P": [-0.2, -0.25, 0.25], "D": np.zeros(3)}})
    lift_pos = trajectory_2_world_frame(ROBOT, {"FL_FOOT": {"P": [0.2, 0.15, -0.05], "D": np.zeros(3)}, "FR_FOOT": {"P": [0.2, -0.15, -0.05], "D": np.zeros(3)}, 
                    "HL_FOOT": {"P": [-0.2, 0.15, -0.05], "D": np.zeros(3)}, "HR_FOOT": {"P": [-0.2, -0.15, -0.05], "D": np.zeros(3)}})
    lift_pos_back = trajectory_2_world_frame(ROBOT, {"FL_FOOT": {"P": [0.2, 0.15, -0.20], "D": np.zeros(3)}, "FR_FOOT": {"P": [0.22, -0.13, -0.22], "D": np.zeros(3)}, 
                    "HL_FOOT": {"P": [-0.18, 0.18, -0.22], "D": np.zeros(3)}, "HR_FOOT": {"P": [-0.2, -0.15, -0.20], "D": np.zeros(3)}})
    lift_pos_inner = trajectory_2_world_frame(ROBOT, {"FL_FOOT": {"P": [0.2, 0.15, -0.20], "D": np.zeros(3)}, "FR_FOOT": {"P": [0.2, -0.15, -0.20], "D": np.zeros(3)}, 
                    "HL_FOOT": {"P": [-0.2, 0.15, -0.20], "D": np.zeros(3)}, "HR_FOOT": {"P": [-0.2, -0.15, -0.20], "D": np.zeros(3)}})
    stand_pos = trajectory_2_world_frame(ROBOT, {"FL_FOOT": {"P": [0.2, 0.15, -0.20], "D": np.zeros(3)}, "FR_FOOT": {"P": [0.2, -0.15, -0.24], "D": np.zeros(3)}, 
                    "HL_FOOT": {"P": [-0.2, 0.15, -0.24], "D": np.zeros(3)}, "HR_FOOT": {"P": [-0.2, -0.15, -0.24], "D": np.zeros(3)}})
    # breakpoint()
    FL, FR, HL, HR = False, False, False, False,
    if init_phase:
        keypress_io.start()
        key_press_init_phase = True
    else:
        key_press_init_phase = False
    
    while (key_press_init_phase):
        if init_phase and key_press_init_phase:
                jointTorques = ROBOT.default_stance_control(ROBOT.q_init, p.TORQUE_CONTROL)
                p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=jointTorques)
                p.stepSimulation()
    for i in range (10000):
        lift_pos = trajectory_2_world_frame(ROBOT, {"FL_FOOT": {"P": [0.2, 0.15, -0.05], "D": np.zeros(3)}, "FR_FOOT": {"P": [0.2, -0.15, -0.05], "D": np.zeros(3)}, 
                    "HL_FOOT": {"P": [-0.2, 0.15, -0.05], "D": np.zeros(3)}, "HR_FOOT": {"P": [-0.2, -0.15, -0.05], "D": np.zeros(3)}})
        lift_pos_back = trajectory_2_world_frame(ROBOT, {"FL_FOOT": {"P": [0.2, 0.15, -0.20], "D": np.zeros(3)}, "FR_FOOT": {"P": [0.2, -0.15, -0.20], "D": np.zeros(3)}, 
                        "HL_FOOT": {"P": [-0.2, 0.15, -0.20], "D": np.zeros(3)}, "HR_FOOT": {"P": [-0.2, -0.15, -0.20], "D": np.zeros(3)}})
        lift_pos_inner = trajectory_2_world_frame(ROBOT, {"FL_FOOT": {"P": [0.2, 0.10, -0.24], "D": np.zeros(3)}, "FR_FOOT": {"P": [0.20, -0.10, -0.24], "D": np.zeros(3)}, 
                    "HL_FOOT": {"P": [-0.2, 0.25, -0.24], "D": np.zeros(3)}, "HR_FOOT": {"P": [-0.2, -0.25, -0.24], "D": np.zeros(3)}})
        stand_pos = trajectory_2_world_frame(ROBOT, {"FL_FOOT": {"P": [0.2, 0.15, -0.20], "D": np.zeros(3)}, "FR_FOOT": {"P": [0.2, -0.15, -0.24], "D": np.zeros(3)}, 
                        "HL_FOOT": {"P": [-0.2, 0.15, -0.24], "D": np.zeros(3)}, "HR_FOOT": {"P": [-0.2, -0.15, -0.24], "D": np.zeros(3)}})
        if sim_cfg['mode'] == "hold_leg":
            FL, FR, HL, HR, balance  = swtich(ROBOT.time_step)
            print(f"timestep {ROBOT.time_step}, balance : {balance}")
            if FL:
                print("Flipping Leg {FL}")
            elif FR:
                print("Flipping Leg {FR}")
            elif HL:
                print("Flipping Leg {HL}")
            elif HR:
                print("Flipping Leg {HR}")
            
            
            
            joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(stand_pos['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
            joint_ang_FR, joint_vel_FR, joint_toq_FR = ROBOT.control(stand_pos['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
            joint_ang_HL, joint_vel_HL, joint_toq_HL = ROBOT.control(stand_pos['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
            joint_ang_HR, joint_vel_HR, joint_toq_HR = ROBOT.control(stand_pos['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)


            if FL:
                if balance:
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(lift_pos['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    joint_ang_FR, joint_vel_FR, joint_toq_FR = ROBOT.control(lift_pos_inner['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    joint_ang_HL, joint_vel_HL, joint_toq_HL = ROBOT.control(lift_pos_inner['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    joint_ang_HR, joint_vel_HR, joint_toq_HR = ROBOT.control(lift_pos_back['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
                else:
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(lift_pos['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    joint_ang_FR, joint_vel_FR, joint_toq_FR = ROBOT.control(lift_pos_inner['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    joint_ang_HL, joint_vel_HL, joint_toq_HL = ROBOT.control(lift_pos_inner['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    joint_ang_HR, joint_vel_HR, joint_toq_HR = ROBOT.control(lift_pos_back['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)

            elif FR:
                if balance:
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(lift_pos_inner['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    joint_ang_FR, joint_vel_FR, joint_toq_FR = ROBOT.control(lift_pos['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    joint_ang_HL, joint_vel_HL, joint_toq_HL = ROBOT.control(lift_pos_back['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    joint_ang_HR, joint_vel_HR, joint_toq_HR = ROBOT.control(lift_pos_inner['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
                else:
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(lift_pos_inner['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    joint_ang_FR, joint_vel_FR, joint_toq_FR = ROBOT.control(lift_pos['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    joint_ang_HL, joint_vel_HL, joint_toq_HL = ROBOT.control(lift_pos_back['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    joint_ang_HR, joint_vel_HR, joint_toq_HR = ROBOT.control(lift_pos_inner['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
            elif HL:
                if balance:
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(stand_pos['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    joint_ang_FR, joint_vel_FR, joint_toq_FR = ROBOT.control(lift_pos_back['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    joint_ang_HL, joint_vel_HL, joint_toq_HL = ROBOT.control(lift_pos['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    joint_ang_HR, joint_vel_HR, joint_toq_HR = ROBOT.control(stand_pos['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
                else:
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(stand_pos['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    joint_ang_FR, joint_vel_FR, joint_toq_FR = ROBOT.control(lift_pos_back['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    joint_ang_HL, joint_vel_HL, joint_toq_HL = ROBOT.control(lift_pos['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    joint_ang_HR, joint_vel_HR, joint_toq_HR = ROBOT.control(stand_pos['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
            elif HR:
                if balance:
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(lift_pos_back['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    joint_ang_FR, joint_vel_FR, joint_toq_FR = ROBOT.control(stand_pos['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    joint_ang_HL, joint_vel_HL, joint_toq_HL = ROBOT.control(stand_pos['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    joint_ang_HR, joint_vel_HR, joint_toq_HR = ROBOT.control(lift_pos['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
                else:
                    joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(lift_pos_back['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                    joint_ang_FR, joint_vel_FR, joint_toq_FR = ROBOT.control(stand_pos['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                    joint_ang_HL, joint_vel_HL, joint_toq_HL = ROBOT.control(stand_pos['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                    joint_ang_HR, joint_vel_HR, joint_toq_HR = ROBOT.control(lift_pos['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
            # breakpoint()
            # breakpoint()

            ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_ang_FL[0:3])
            ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joint_ang_FR[3:6])
            ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joint_ang_HL[6:9])
            ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joint_ang_HR[9:12])
            p.stepSimulation()
            ROBOT.time_step += 1
            if ROBOT.time_step >= 2000:
                ROBOT.time_step = 0

            # time.sleep(0.01)
    p.disconnect()


if __name__ == "__main__":
    simulation()