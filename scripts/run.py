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
import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
config_sim = "./data/config/simulation.yml"
config_sim_record= "./data/config/simulation_towr_record.yml"
cfg = yaml.safe_load(open(config, 'r'))
sim_cfg = yaml.safe_load(open(config_sim, 'r'))
TOWR = "./data/traj/towr.csv"
TOWR_TRAJ = "./data/traj/towr_traj"
TRACK = "./data/traj/traj_thirdparty/jointStates.csv"
TRACK_imitation = "./data/traj/testing_gait.csv"
HZ = sim_cfg['HZ']

# global keypressed
key_press_init_phase = True
mutex = Lock()

def update_file_name(file, cfg, sim_cfg):
    step_period = str(sim_cfg['step_period'])
    velocity = str(sim_cfg['velocity'])
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
        global_cfg.RUN.step += sim_cfg['skip_forward_idx']
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
    global key_press_init_phase
    Simulation(sim_cfg['enviroment'], timestep=sim_cfg['TIMESTEPS'])
    ROBOT = SOLO12(URDF, cfg, fixed=sim_cfg['fix-base'], sim_cfg=sim_cfg)
    gait = Gait(ROBOT)
    init_phase = sim_cfg['stance_phase']
    last_loop_time = time.time()
    sim_step = 0
    """============SIM-CONFIGURATION============"""
    if sim_cfg['enviroment'] == "testing":
        velocity, angle_velocity , angle, step_period = sim_cfg['velocity'], sim_cfg['angle_velocity'], sim_cfg['angle'], sim_cfg['step_period']
    if sim_cfg['mode'] == "towr" or sim_cfg['mode'] == "visual_track":
        csv_file = open(TOWR, 'r', newline='')
        reader = csv.reader(csv_file, delimiter=',')
        NUM_TIME_STEPS = sum(1 for row in reader)
        csv_file = open(TOWR, 'r', newline='')
        reader = csv.reader(csv_file, delimiter=',')
        _t = np.linspace(0, NUM_TIME_STEPS/HZ, NUM_TIME_STEPS)
        if args.get('record'):
            FILE = update_file_name(TOWR_TRAJ, cfg, sim_cfg)
            file = open(FILE, 'w', newline='')
            writer = csv.writer(file) 
            record_timestep = 0
            
    elif sim_cfg['mode'] == "track":
        csv_file = open(TRACK, 'r', newline='')
        reader =csv.reader(csv_file, delimiter=' ')
    elif sim_cfg['mode'] == "track_imitation":
        csv_file = open(TRACK_imitation, 'r', newline='')
        reader = csv.reader(csv_file, delimiter=',')
    elif sim_cfg['mode'] == 'bezier':
        offsets = np.array(cfg['offsets'])
        trot_2_stance_ratio = cfg['trot_2_stance_ratio']
        velocity, angle, angle_velocity, step_period = sim_cfg['velocity'], sim_cfg['angle_velocity'], sim_cfg['angle'], sim_cfg['step_period']
    if sim_cfg['py_interface']:
        pybullet_interface = PybulletInterface()
        pos, angle, velocity, angle_velocity , angle, step_period = pybullet_interface.robostates(ROBOT.robot)
    """=========================================="""
    cmd = np.zeros((12, 1))
    keypress_io = Thread(target=keypress)
    if init_phase:
        keypress_io.start()
        key_press_init_phase = True
    else:
        key_press_init_phase = False
    
    while (key_press_init_phase):
        loop_time = time.time() - last_loop_time
        if init_phase and key_press_init_phase:
                _, _, joint_toq = ROBOT.default_stance_control()
                p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=joint_toq)
                p.stepSimulation()
        if loop_time > sim_cfg['TIMESTEPS'] and args.get('record'):
            csv_entry = ROBOT.csv_entry
            writer.writerow(csv_entry)
            last_loop_time = time.time()

    while (sim_step < sim_cfg["NUM_TIME_STEPS"]):
        loop_time = time.time() - last_loop_time
        time_loop = time.time()
        if loop_time > sim_cfg['TIMESTEPS']:
            if sim_cfg['mode'] == "bezier":
                if sim_cfg['py_interface']:
                    pos, angle, velocity, angle_velocity , angle,  step_period = pybullet_interface.robostates(ROBOT.robot)
                gait_traj, newCmd = gait.runTrajectory(velocity, angle, angle_velocity, offsets, step_period, trot_2_stance_ratio)
                joint_ang, joint_vel, joint_toq = ROBOT.control_multi(gait_traj, ROBOT.EE_index['all'], mode=ROBOT.mode)
                ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
                p.stepSimulation()
                ROBOT.time_step += 1

            elif sim_cfg['mode'] == "towr":
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
                    if sim_cfg['enviroment'] == "plane":
                        for _ in range(sim_cfg['skip_forward_idx']):
                            next(reader)
                    EE_POSE = np.array([float(x) for x in next(reader)])[1:]
                    global_cfg.ROBOT_CFG.last_POSE = EE_POSE[0:3]
                    global_cfg.RUN.TOWR_POS = EE_POSE[0:3]
                    towr_traj = towr_transform(ROBOT, vec_to_cmd_pose(EE_POSE))
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

                if args.get('record'):
                    csv_entry = ROBOT.csv_entry
                    writer.writerow(csv_entry)
                    record_timestep += 1
                    if record_timestep >= sim_cfg['NUM_TIME_STEPS']:
                        global_cfg.RUN._run_update_thread = False
                        break
                
                print("STATE +++ ", ROBOT.state)
                # print("CMD ->" , towr_traj)
                # breakpoint()

                p.stepSimulation()
                ROBOT.time_step += 1
                _global_update(ROBOT, ROBOT.state)

            elif sim_cfg['mode'] == "track_imitation":
                try:
                    ee_pose = np.array([float(x) for x in next(reader)])[3:]
                except StopIteration:
                    break
                traj = trajectory_2_world_frame(ROBOT, create_cmd(ee_pose))
                joint_ang, joint_vel, joint_toq = ROBOT.control_multi(traj, ROBOT.EE_index['all'], mode=ROBOT.mode)
                ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
                global_cfg.RUN.step += 1  
                p.stepSimulation()
                ROBOT.time_step += 1

            last_loop_time = time.time()
            sim_step += 1
            # print(f"runtime: {time.time() - time_loop}")
        else:
            continue

        
    p.disconnect()

if __name__ == "__main__":
    simulation()

