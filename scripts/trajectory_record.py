#! /usr/bin/python3

import time
import csv
import argparse
from threading import Thread, Lock

#third party
import pybullet as p
import pybullet_data as pd
import yaml
import numpy as np

#project modules
from QTOS.utils import *
import QTOS.config.global_cfg as global_cfg

config = "./data/config/solo12.yml"
cfg = yaml.safe_load(open(config, 'r'))
TOWR = "./data/traj/towr.csv"
TOWR_TRAJ = "./data/traj/towr_traj"

mutex = Lock()

def save_traj(writer, entries):
    """
    Save a trajectory to a CSV file.

    This function takes a CSV writer object and a list of trajectory entries and writes the entries to the CSV file.
    
    Parameters:
        writer (csv.writer): A CSV writer object open for writing to the target file.
        entries (list): A list of trajectory entries, where each entry is a list representing a single row of data.

    Notes:
        This function iterates through the list of trajectory entries and writes each entry as a row to the CSV file
        using the provided CSV writer object. The length of the trajectory (number of entries) is printed to the console.

    Args:
        writer (csv.writer): A CSV writer object open for writing to the target file.
        entries (list): A list of trajectory entries, where each entry is a list representing a single row of data.
    """
    print(f"size of trajectory {len(entries)}")
    for csv_entry in entries:
        writer.writerow(csv_entry)

def make_even(number):
    """
    Ensure a number is even.

    This function takes an integer 'number' and ensures that it is even. If the input number is already even,
    it is returned unchanged. If the input number is odd, it is incremented by 1 to make it even.

    Parameters:
        number (int): The input integer.

    Returns:
        int: An even integer, either the input 'number' (if it's already even) or 'number' + 1 (if it's odd).
    """
    if number % 2 != 0: 
        number += 1 
    return number

def parser():
    """
    Create an argument parser for command-line options.

    This function creates an argument parser using the argparse library, which allows you to define and parse
    command-line options for your script.

    Returns:
        argparse.Namespace: A namespace containing the parsed command-line arguments.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--test', action="store_true", help="Sets testing flag for CI")
    args = parser.parse_args()
    return args

def update_file_name(file, cfg, sim_cfg):
    """
    Update a file name based on configuration parameters.

    This function takes a base file name, a configuration dictionary 'cfg', and a simulation configuration dictionary 'sim_cfg'.
    It generates a new file name by appending information from the configurations, such as the simulation mode.

    Parameters:
        file (str): The base file name.
        cfg (dict): A configuration dictionary containing various parameters.
        sim_cfg (dict): A simulation configuration dictionary containing simulation-specific parameters.

    Returns:
        str: The updated file name with appended information from the configurations.
    """
    MODE = str(cfg['mode'])
    file_name = file +  "_cmode_" + MODE + ".csv"
    return file_name

def _global_update(ROBOT, kwargs, sim_cfg):
    """
    Update global configuration settings with robot and simulation data.

    Args:
        ROBOT: An object representing the robot.
        kwargs (dict): A dictionary containing various data related to the robot and simulation.
    """
    global_cfg.ROBOT_CFG.robot = ROBOT
    global_cfg.ROBOT_CFG.global_COM_xyz = list(kwargs['COM'])
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

def record_simulation(args):
    """
    Main simulation interface that runs the Bullet engine and records trajectory data.

    This function serves as the main interface for running a simulation using the Bullet engine. It controls the simulation
    loop, updates the robot's state, and records trajectory data to a CSV file.

    Parameters:
        args (dict): A dictionary containing various simulation parameters and objects.

    Notes:
        This function orchestrates the simulation and recording process with the following steps:
        1. Initializes simulation parameters and objects.
        2. Performs an initial stance phase to position the robot.
        3. Enters the main simulation loop, updating the robot's state and controlling its motion.
        4. Records trajectory data at specified intervals during the simulation.
        5. Saves the recorded trajectory to a CSV file.
        6. Cleans up and disconnects from the simulation.

    Args:
        args (dict): A dictionary containing simulation parameters and objects.
    """
    global key_press_init_phase
    ROBOT = args['robot']
    sim_cfg = args["sim_cfg"]
    init_phase = sim_cfg['stance_phase']
    last_loop_time = time.time()
    sim_step = 1
    RECORD_TRAJ = False
    csv_entries = []
    stance_step = 0
    update_data = {"ROBOT": ROBOT, "sim_step": sim_step}
    """============Record-CONFIGURATION============"""
    csv_file = open(TOWR, 'r', newline='')
    reader = csv.reader(csv_file, delimiter=',')
    TRAJ_SIZE = sum(1 for row in reader)
    traj = np.genfromtxt(TOWR, delimiter=',')
    first_traj_point = traj[0]
    time_step, EE_POSE = first_traj_point[0], first_traj_point[1:]
    ref_start_cmd = towr_transform(ROBOT, vec_to_cmd_pose(EE_POSE))
    q_init, _, _ = ROBOT.control_multi(ref_start_cmd, ROBOT.EE_index['all'], mode=ROBOT.mode)
    FILE = update_file_name(TOWR_TRAJ, cfg, sim_cfg)
    record_file = open(FILE, 'w', newline='')
    print(f"RECORDING FILE AT -> {record_file}")
    writer = csv.writer(record_file) 
    record_timestep = 0
    RECORD_TRAJ = True
    """=========================================="""
    """==Init stance to force robot in good starting position=="""
    while (init_phase):
        loop_time = time.time() - last_loop_time
        if loop_time > sim_cfg['TIMESTEPS']:
            if stance_step >= sim_cfg.get('stance_period'):
                init_phase = False
                ROBOT._motor.set_motor_gains(ROBOT._kp, ROBOT._kd)
                break
            if init_phase:
                csv_entry = ROBOT.csv_entry
                _, _, joint_toq = ROBOT.default_stance_control(q_init)
                p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=joint_toq)
                p.stepSimulation()
                last_loop_time = time.time()
                stance_step += 1
                csv_entries.append(csv_entry)
                last_loop_time = time.time()
    """============Main-running-loop============"""
    while (sim_step < sim_cfg["SIM_STEPS"]):
        if sim_step < sim_cfg["TRAJ_SIZE"]:
            loop_time = time.time() - last_loop_time
            time_loop = time.time()
            if loop_time > sim_cfg['TIMESTEPS']:
                try:
                    if global_cfg.RUN._done:
                        print("ROBOT HAS REACHED THE GOAL")
                        break
                    if global_cfg.RUN._wait:
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
                    last_loop_time = time.time()
                    if sim_cfg['skip_forward_idx'] > 1:
                        for _ in range(sim_cfg['skip_forward_idx']):
                            next(reader)
                except StopIteration:
                    global_cfg.RUN._stance = True
                #==============Controlling Robot=================##
                if global_cfg.RUN._stance:
                    _, _, joint_toq = ROBOT.default_stance_control()
                    p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=joint_toq)
                else:
                    joint_ang, joint_vel, joint_toq = ROBOT.control_multi(towr_traj, ROBOT.EE_index['all'], mode=ROBOT.mode)
                    ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
                #==============Controlling Robot=================##
                #==============Update Simulation State=================##
                if sim_step >= sim_cfg['TRAJ_SIZE']:
                    global_cfg.RUN._run_update_thread = False
                    break
                sim_step = update(update_data, sim_cfg)
                p.stepSimulation()
                last_loop_time = time.time()
                #==============Update Simulation State=================##
                #==============Record State=================##
                with mutex:
                    csv_entry = ROBOT.csv_entry
                    for copy_itr in range(sim_cfg['copy_trajectory_pts']): #IMPORTANT to regulate the record frequence bc Pybullet can't run at 1000 hz but at 240 hz                       
                        global_cfg.ROBOT_CFG.robot_states.append(csv_entry)
                        csv_entries.append(csv_entry)
                #==============Record State=================##
        else:
            loop_time = time.time() - last_loop_time
            time_loop = time.time()
            if loop_time > sim_cfg['TIMESTEPS']:
                towr_traj = towr_transform(ROBOT, vec_to_cmd_pose(EE_POSE))
                joint_ang, joint_vel, joint_toq = ROBOT.control_multi(towr_traj, ROBOT.EE_index['all'], mode=ROBOT.mode)
                ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
                p.stepSimulation()
                last_loop_time = time.time()
                sim_step += 1
                
        if make_even(sim_step) % 500 == 0:
            print(f"step [{sim_step} / {sim_cfg['TRAJ_SIZE']}]")

    save_traj(writer, csv_entries)
    print("Trajectory Recording is done!")
    print(f"Record Trajectory is at {record_file}")
    global_cfg.RUN._done = True
    p.disconnect()

def update(data_dic, sim_cfg):
    """
    Update function for simulation data and control.

    This function updates various components and data within the simulation based on the provided data dictionary and simulation configuration.

    Parameters:
        data_dic (dict): A dictionary containing simulation data and components.
        sim_cfg (dict): A dictionary containing simulation configuration parameters.

    Args:
        data_dic (dict): A dictionary containing simulation data and components.
        sim_cfg (dict): A dictionary containing simulation configuration parameters.
    """
    if sim_cfg['skip_forward_idx'] > 1:
        for _ in range(sim_cfg['skip_forward_idx'] + 1):
            data_dic['ROBOT'].time_step += 1
            data_dic['sim_step'] += 1
        data_dic['sim_step'] -= 1
    else:
        data_dic['ROBOT'].time_step += 1
    _global_update(data_dic['ROBOT'], data_dic['ROBOT'].state, sim_cfg)
    data_dic['ROBOT'].update()
    data_dic['sim_step'] += 1
    return data_dic['sim_step']
