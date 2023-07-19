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
from SOLO12_SIM_CONTROL.robot.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import *
from SOLO12_SIM_CONTROL.gaitPlanner import Gait
from SOLO12_SIM_CONTROL.pybulletInterface import PybulletInterface
from SOLO12_SIM_CONTROL.simulation import Simulation
from SOLO12_SIM_CONTROL.logger import Logger
from SOLO12_SIM_CONTROL.tracking import Tracking
import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12_debug.yml"
config_sim = "./data/config/debug.yml"
cfg = yaml.safe_load(open(config, 'r'))
sim_cfg = yaml.safe_load(open(config_sim, 'r'))
TOWR = "./data/traj/towr.csv"
TOWR_TRAJ = "./data/traj/towr_traj"

def update_file_name(file, cfg, sim_cfg):
    MODE = str(cfg['mode'])
    file_name = file +  "_cmode_" + MODE + ".csv"
    return file_name

def setup_enviroment():
    py_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    return py_client 

def importRobot(file=URDF, POSE=([0,0,1], (0.0,0.0,0.0,1.0))):
    solo12 = p.loadURDF(URDF, *POSE)
    return solo12

def _global_update(ROBOT, kwargs):
    global_cfg.ROBOT_CFG.robot = ROBOT
    global_cfg.ROBOT_CFG.linkWorldPosition = list(kwargs['COM'])
    global_cfg.ROBOT_CFG.linkWorldOrientation = list(p.getEulerFromQuaternion(kwargs['linkWorldOrientation']))
    global_cfg.ROBOT_CFG.EE['FL_FOOT'] = list(kwargs['FL_FOOT'])
    global_cfg.ROBOT_CFG.EE['FR_FOOT'] = list(kwargs['FR_FOOT'])
    global_cfg.ROBOT_CFG.EE['HL_FOOT'] = list(kwargs['HL_FOOT'])
    global_cfg.ROBOT_CFG.EE['HR_FOOT'] = list(kwargs['HR_FOOT'])
    global_cfg.ROBOT_CFG.runtime = ROBOT.time
    if sim_cfg['enviroment'] == "plane":
        global_cfg.RUN.step += sim_cfg['skip_forward_idx'] + 1
    elif sim_cfg['enviroment'] == "towr_no_gui":
        global_cfg.RUN.step += 1
    global_cfg.ROBOT_CFG.joint_state = ROBOT.jointstate

def keypress():
    """thread to handle keyboard I/O
    """
    global key_press_init_phase
    while True:
        print("Press q to exit")
        val = input('Enter your input: ')
        if val == 'q':
            print("Moving to trajectory")
            key_press_init_phase = False
            break

def simulation(args={}):
    """Main simulation interface that runs the bullet engine
    
    """
    log = Logger("./logs", "simulation_log")
    Simulation(sim_cfg['enviroment'])
    ROBOT = SOLO12(URDF, cfg, fixed=sim_cfg['fix-base'], sim_cfg=sim_cfg)
    last_loop_time = time.time()
    sim_step = 0
    """============SIM-CONFIGURATION============"""
    if sim_cfg['enviroment'] == "testing":
        velocity, angle_velocity , angle, step_period = sim_cfg['velocity'], sim_cfg['angle_velocity'], sim_cfg['angle'], sim_cfg['step_period']
    if sim_cfg['mode'] == "towr_track_no_contact":
        csv_file = open(TOWR, 'r', newline='')
        reader = csv.reader(csv_file, delimiter=',')
        if args.get('record') or sim_cfg.get('record'):
            FILE = update_file_name(TOWR_TRAJ, cfg, sim_cfg)
            file = open(FILE, 'w', newline='')
            NUM_TIME_STEPS = sum(1 for row in reader)
            csv_file = open(TOWR, 'r', newline='')
            reader = csv.reader(csv_file, delimiter=',')
            writer = csv.writer(file) 
            record_timestep = 0
            TRACK_RECORD = Tracking(ROBOT, NUM_TIME_STEPS)

    """=========================================="""

    while (sim_step < NUM_TIME_STEPS):
        loop_time = time.time() - last_loop_time
        time_loop = time.time()
        if loop_time > sim_cfg['TIMESTEPS']:
            if sim_cfg['mode'] == "towr_track_no_contact":
                try:
                    if sim_cfg['skip_forward_idx'] > 1:
                        for _ in range(sim_cfg['skip_forward_idx']):
                                next(reader)
                                sim_step += 1
                    traj = np.array([float(x) for x in next(reader)])
                    time_step, cmds = traj[0], vec_to_cmd_pose(traj[1:])
                    COM = cmds['COM']
                except StopIteration:
                    break

                joint_ang, joint_vel, joint_toq = ROBOT.control_multi(cmds, ROBOT.EE_index['all'], mode=ROBOT.mode)
                if sim_cfg.get('record'):
                    csv_entry = ROBOT.csv_entry
                    for i in range(5):
                        writer.writerow(csv_entry)
                ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
                p.resetBasePositionAndOrientation(ROBOT.robot, COM[0:3], p.getQuaternionFromEuler(COM[3:6]))
                p.stepSimulation()
                TRACK_RECORD.update(cmds, time_step)
                print(f"Time [{time_step:.3f}] || COM [{[round(i, 3) for i in COM[0:3].tolist()]}]")
                ROBOT.time_step += 1
                sim_step += 1
                # if sim_step == 1000:
                #     TRACK_RECORD.plot_realized_vs_sim()
        else:
            continue
    
    TRACK_RECORD.plot_realized_vs_sim()
        
    p.disconnect()

if __name__ == "__main__":
    simulation()

