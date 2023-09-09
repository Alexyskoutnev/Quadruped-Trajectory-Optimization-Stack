#! /opt/homebrew/Caskroom/miniforge/base/envs/soloSim/bin/python3

import time
import os
import sys
import math
import csv
import argparse
from threading import Thread, Lock

#third party
import pybullet as p
import pybullet_data as pd
import yaml
import numpy as np

#project
from QTOS.robot.robot import SOLO12
from QTOS.utils import *
from QTOS.pybulletInterface import PybulletInterface
from QTOS.simulation import Simulation
from QTOS.logger import Logger
from QTOS.tracking import Tracking
from QTOS.visual import Visual_Planner
from QTOS.builder import builder
import QTOS.config.global_cfg as global_cfg

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
config_sim = "./data/config/simulation.yml"
cfg = yaml.safe_load(open(config, 'r'))
# sim_cfg = None
TOWR = "./data/traj/towr.csv"
TOWR_TRAJ = "./data/traj/towr_traj"
BEZIER_TRAJ = "./data/traj/bezier_traj"

# global keypressed
key_press_init_phase = True
mutex = Lock()

def save_traj(writer, entries):
    print(f"size of trajectory {len(entries)}")
    for csv_entry in entries:
        writer.writerow(csv_entry)

def make_even(number):
    if number % 2 != 0:  # Check if the number is odd
        number += 1      # Increment the number by 1 to make it even
    return number

def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--test', action="store_true", help="Sets testing flag for CI")
    args = parser.parse_args()
    return args

def update_file_name(file, cfg, sim_cfg):
    step_period = str(sim_cfg['step_period'])
    velocity = str(sim_cfg['velocity'])
    MODE = str(cfg['mode'])
    file_name = file +  "_cmode_" + MODE + ".csv"
    return file_name

def _global_update(ROBOT, kwargs, sim_cfg):
    global_cfg.ROBOT_CFG.robot = ROBOT
    global_cfg.ROBOT_CFG.linkWorldPosition = list(kwargs['COM'])
    global_cfg.ROBOT_CFG.linkWorldOrientation = list(p.getEulerFromQuaternion(kwargs['linkWorldOrientation']))
    global_cfg.ROBOT_CFG.EE['FL_FOOT'] = list(kwargs['FL_FOOT'])
    global_cfg.ROBOT_CFG.EE['FR_FOOT'] = list(kwargs['FR_FOOT'])
    global_cfg.ROBOT_CFG.EE['HL_FOOT'] = list(kwargs['HL_FOOT'])
    global_cfg.ROBOT_CFG.EE['HR_FOOT'] = list(kwargs['HR_FOOT'])
    global_cfg.ROBOT_CFG.runtime = ROBOT.time
    if sim_cfg['skip_forward_idx'] > 1:
        global_cfg.RUN.step += sim_cfg['skip_forward_idx'] + 1
    else:
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

def record_simulation(args):
    """Main simulation interface that runs the bullet engine
    
    """
    log = Logger("./logs", "simulation_log")
    global key_press_init_phase
    ROBOT = args['robot']
    sim_cfg = args["sim_cfg"]
    init_phase = sim_cfg['stance_phase']
    last_loop_time = time.time()
    sim_step = 1
    RECORD_TRAJ = False

    csv_entries = []

    """============Record-CONFIGURATION============"""
    if sim_cfg['mode'] == "towr":
        csv_file = open(TOWR, 'r', newline='')
        reader = csv.reader(csv_file, delimiter=',')
        TRAJ_SIZE = sum(1 for row in reader)
        traj = np.genfromtxt(TOWR, delimiter=',')
        first_traj_point = traj[0]
        if sim_cfg['stance_phase']:
            time_step, EE_POSE = first_traj_point[0], first_traj_point[1:]
            ref_start_cmd = vec_to_cmd_pose(EE_POSE)
            q_init, _, _ = ROBOT.control_multi(ref_start_cmd, ROBOT.EE_index['all'], mode=ROBOT.mode)
        FILE = update_file_name(TOWR_TRAJ, cfg, sim_cfg)
        record_file = open(FILE, 'w', newline='')
        print(f"RECORDING FILE AT -> {record_file}")
        writer = csv.writer(record_file) 
        record_timestep = 0
        RECORD_TRAJ = True
    """=========================================="""

    cmd = np.zeros((12, 1))
    keypress_io = Thread(target=keypress)
    if init_phase:
        keypress_io.start()
        key_press_init_phase = True
    else:
        key_press_init_phase = False
    
    stance_step = 0

    while (key_press_init_phase):
        loop_time = time.time() - last_loop_time
        if stance_step >= sim_cfg['stance_period']:
            key_press_init_phase = False
            break
        if init_phase and key_press_init_phase:
                _, _, joint_toq = ROBOT.default_stance_control(q_init)
                p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=joint_toq)
                p.stepSimulation()
        if loop_time > sim_cfg['TIMESTEPS'] and RECORD_TRAJ:
            csv_entry = ROBOT.csv_entry
            writer.writerow(csv_entry)
            last_loop_time = time.time()
            stance_step += 1

    while (sim_step < sim_cfg["SIM_STEPS"]):
        if sim_step < sim_cfg["TRAJ_SIZE"]:
            loop_time = time.time() - last_loop_time
            time_loop = time.time()
            if loop_time > sim_cfg['TIMESTEPS']:
                if sim_cfg['mode'] == "towr":
                    try:
                        if global_cfg.RUN._done:
                            print("ROBOT HAS REACHED THE GOAL")
                            break
                        if global_cfg.RUN._wait: #Waits for towr thread to copy over the trajectory
                            time.sleep(0.0001)
                            continue
                        elif global_cfg.RUN._update:
                            print("============UPDATE STATE============")
                            mutex.acquire()
                            reader = csv.reader(open(TOWR, 'r', newline=''))
                            global_cfg.RUN._update = False 
                            global_cfg.RUN.step = 0
                            mutex.release()
                        traj = np.array([float(x) for x in next(reader)])
                        time_step, EE_POSE = traj[0], traj[1:]
                        global_cfg.ROBOT_CFG.last_POSE = EE_POSE[0:3]
                        global_cfg.RUN.TOWR_POS = EE_POSE[0:3]
                        towr_traj = towr_transform(ROBOT, vec_to_cmd_pose(EE_POSE))
                        ref_cmd = vec_to_cmd_pose(EE_POSE)
                        COM = EE_POSE[0:6]
                        if sim_cfg['skip_forward_idx'] > 1:
                            for _ in range(sim_cfg['skip_forward_idx']):
                                next(reader)
                    except StopIteration:
                        log.write("==========STANCE==========")
                        global_cfg.RUN._stance = True
                    ##====================Logging====================##
                    log.write(f"TIME STEP ==> {global_cfg.RUN.step}\n")
                    log.write(f"Towr CoM POS -> {EE_POSE[0:3]}\n")
                    log.write(f"Global POS -> {global_cfg.ROBOT_CFG.linkWorldPosition}\n")
                    log.write(f"=========Global Vars=========\n")
                    log.write(f"{global_cfg.print_vars(log.log)}\n")
                    ##===============================================##
                    if global_cfg.RUN._stance:
                        _, _, joint_toq = ROBOT.default_stance_control()
                        p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=joint_toq)
                    else:
                        joint_ang, joint_vel, joint_toq = ROBOT.control_multi(towr_traj, ROBOT.EE_index['all'], mode=ROBOT.mode)
                        ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)

                    

                    if record_timestep >= sim_cfg['TRAJ_SIZE']:
                        global_cfg.RUN._run_update_thread = False
                        break

                    if sim_cfg['skip_forward_idx'] > 1:
                        for _ in range(sim_cfg['skip_forward_idx'] + 1):
                            ROBOT.time_step += 1
                            sim_step += 1
                        sim_step -= 1
                    else:
                        ROBOT.time_step += 1


                    p.stepSimulation()
                    _global_update(ROBOT, ROBOT.state, sim_cfg)
                    ROBOT.update()
                    ROBOT.time_step += 1
                # print(f"entered: {sim_step}")
                # print(f"values: {ROBOT.csv_entry}")
                # with mutex:
                #     csv_entry = ROBOT.csv_entry
                #     global_cfg.ROBOT_CFG.robot_states.append(csv_entry)
                #     csv_entries.append(csv_entry)
                # writer.writerow(csv_entry)
                record_timestep += 1
                last_loop_time = time.time()
                sim_step += 1
                with mutex:
                    csv_entry = ROBOT.csv_entry
                    global_cfg.ROBOT_CFG.robot_states.append(csv_entry)
                    csv_entries.append(csv_entry)
        else:
            loop_time = time.time() - last_loop_time
            time_loop = time.time()
            if loop_time > sim_cfg['TIMESTEPS']:
                print("DONE")
                if sim_cfg['mode'] == "towr":
                    towr_traj = towr_transform(ROBOT, vec_to_cmd_pose(EE_POSE))
                    joint_ang, joint_vel, joint_toq = ROBOT.control_multi(towr_traj, ROBOT.EE_index['all'], mode=ROBOT.mode)
                    ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
                    p.stepSimulation()
                    last_loop_time = time.time()
                    sim_step += 1
                else:
                    break

        if make_even(sim_step) % 500 == 0:
            pass
            print(f"step [{sim_step} / {sim_cfg['TRAJ_SIZE']}]")

    save_traj(writer, csv_entries)
    
    # print(f"TRAJ RECORD PATH -> {record_file}")
    p.disconnect()
