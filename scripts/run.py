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
from SOLO12_SIM_CONTROL.robot.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import *
from SOLO12_SIM_CONTROL.gaitPlanner import Gait
from SOLO12_SIM_CONTROL.pybulletInterface import PybulletInterface, RecordInterface
from SOLO12_SIM_CONTROL.simulation import Simulation
from SOLO12_SIM_CONTROL.logger import Logger
from SOLO12_SIM_CONTROL.tracking import Tracking
from SOLO12_SIM_CONTROL.visual import Visual_Planner
from SOLO12_SIM_CONTROL.builder import builder
import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
config_sim = "./data/config/simulation.yml"
cfg = yaml.safe_load(open(config, 'r'))
sim_cfg = yaml.safe_load(open(config_sim, 'r'))
TOWR = "./data/traj/towr.csv"
TOWR_TRAJ = "./data/traj/towr_traj"
BEZIER_TRAJ = "./data/traj/bezier_traj"

# global keypressed
key_press_init_phase = True
mutex = Lock()

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

def _global_update(ROBOT, kwargs):
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

def simulation(args):
    """Main simulation interface that runs the bullet engine
    
    """
    log = Logger("./logs", "simulation_log")
    global key_press_init_phase
    ROBOT = args['robot']
    gait = Gait(ROBOT)
    init_phase = sim_cfg['stance_phase']
    last_loop_time = time.time()
    sim_step = 1
    RECORD_TRAJ = False
    
    """============SIM-CONFIGURATION============"""
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
        v_planner = Visual_Planner(TOWR, sim_cfg)
        if sim_cfg.get('track'):
            TRACK_RECORD = Tracking(ROBOT, TRAJ_SIZE, sim_cfg)
        if args.get('record') or sim_cfg.get('record'):
            FILE = update_file_name(TOWR_TRAJ, cfg, sim_cfg)
            record_file = open(FILE, 'w', newline='')
            writer = csv.writer(record_file) 
            record_timestep = 0
            RECORD_TRAJ = True
    elif sim_cfg['mode'] == 'bezier':
        trot_2_stance_ratio = cfg['trot_2_stance_ratio']
        velocity, angle, angle_velocity, step_period, offsets = sim_cfg['velocity'], sim_cfg['angle'], sim_cfg['angle_velocity'], sim_cfg['step_period'], np.array(cfg['offsets'])
        TRAJ_SIZE = sim_cfg['TRAJ_SIZE']
        if sim_cfg.get('track'):
            TRACK_RECORD = Tracking(ROBOT, TRAJ_SIZE, sim_cfg)
        if args.get('record') or sim_cfg.get('record'):
            FILE = update_file_name(BEZIER_TRAJ, cfg, sim_cfg)
            record_file = open(FILE, 'w', newline='')
            writer = csv.writer(record_file) 
            record_timestep = 0
            RECORD_TRAJ = True
    if sim_cfg['py_interface']:
        pybullet_interface = PybulletInterface()
    elif sim_cfg['custom_camera_view']:
        pybullet_interface = RecordInterface(args, ROBOT.robot)
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
                if sim_cfg['mode'] == "bezier":
                    if sim_cfg['py_interface']:
                        pos, angle, velocity, angle_velocity, step_period = pybullet_interface.robostates(ROBOT.robot)
                    gait_traj, newCmd = gait.runTrajectory(velocity, angle, angle_velocity, offsets, step_period, trot_2_stance_ratio)
                    joint_ang, joint_vel, joint_toq = ROBOT.control_multi(gait_traj, ROBOT.EE_index['all'], mode=ROBOT.mode, usePin=cfg['use_pinocchio'])
                    ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
                    p.stepSimulation()
                    ROBOT.time_step += 1
                    if sim_cfg.get('track'):
                        TRACK_RECORD.update(gait_traj, ROBOT.time_step)
                    if RECORD_TRAJ:
                        csv_entry = ROBOT.csv_entry
                        writer.writerow(csv_entry)
                        record_timestep += 1
                        if record_timestep >= sim_cfg['TRAJ_SIZE']:
                            break
                elif sim_cfg['mode'] == "towr":
                    try:
                        if global_cfg.RUN._done:
                            print("ROBOT HAS REACHED THE GOAL")
                            break
                        if global_cfg.RUN._wait: #Waits for towr thread to copy over the trajectory
                            time.sleep(0.001)
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

                    if RECORD_TRAJ:
                        csv_entry = ROBOT.csv_entry
                        writer.writerow(csv_entry)
                        record_timestep += 1
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

                    if sim_cfg.get('track'):
                        TRACK_RECORD.update(ref_cmd, ROBOT.time_step)
                    p.stepSimulation()
                    _global_update(ROBOT, ROBOT.state)
                    v_planner.step(sim_step, ROBOT.time)
                    ROBOT.update()

                last_loop_time = time.time()
                sim_step += 1
                if sim_cfg['custom_camera_view']:
                    pybullet_interface.update()

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

        

    if sim_cfg.get('track'):
        TRACK_RECORD.plot()
    if RECORD_TRAJ:
        print(f"TRAJ RECORD PATH -> {record_file}")

    p.disconnect()

if __name__ == "__main__":
    args = parser()
    if args.test:
        print("End-to-End Test")
        config = "./test/data/config/solo12_test.yml"
        config_sim = "./test/data/config/simulation_towr_test.yml"
        cfg = yaml.safe_load(open(config, 'r'))
        sim_cfg = yaml.safe_load(open(config_sim, 'r'))
        TOWR = "./test/data/traj/towr.csv"
        args = builder(cfg=cfg, sim_cfg=sim_cfg)
        simulation(args)
    else:
        args = builder()
        simulation(args)

