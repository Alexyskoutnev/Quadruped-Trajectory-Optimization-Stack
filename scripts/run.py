#! /usr/bin/python3

import time
import os
import sys
import math
import csv
import argparse
from threading import Thread, Lock
import subprocess
import shlex

#third party
import pybullet as p
import pybullet_data as pd
import yaml
import numpy as np

#project
from QTOS.robot.robot import SOLO12
from QTOS.utils import *
from QTOS.gaitPlanner import Gait
from QTOS.pybulletInterface import PybulletInterface, RecordInterface
from QTOS.simulation import Simulation
from QTOS.logger import Logger
from QTOS.tracking import Tracking
from QTOS.visual import Visual_Planner
from QTOS.builder import builder
import QTOS.config.global_cfg as global_cfg

config_sim = "./data/config/simulation.yml"
sim_cfg = yaml.safe_load(open(config_sim, 'r'))
PLAN = "./data/traj/towr.csv"

mutex = Lock()

def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--towr', action="store_true", help="Default TOWR local planner")
    parser.add_argument('-g', '--g', nargs=3, type=float, default=[3.0,0,0.24])
    parser.add_argument('-s', '--s', nargs=3, type=float)
    parser.add_argument('-s_ang', '--s_ang', nargs=3, type=float)
    parser.add_argument('-s_vel', '--s_vel', nargs=3, type=float)
    parser.add_argument('-n', '--n', nargs=1, type=str, default="t")
    parser.add_argument('-e1', '--e1', nargs=3, type=float)
    parser.add_argument('-e2', '--e2', nargs=3, type=float)
    parser.add_argument('-e3', '--e3', nargs=3, type=float)
    parser.add_argument('-e4', '--e4', nargs=3, type=float)
    parser.add_argument('-step', '--step', type=float, default=0.5)
    parser.add_argument('-forced_steps', '--f_steps', type=int, default=2500)
    parser.add_argument('-l', '--look', type=float, default=3750)
    parser.add_argument('-r', '--record', type=bool, default=False)
    parser.add_argument('-exp', '--experiment', type=str, default="default")
    parser.add_argument('-p', '--mpc_p', type=bool, default=False)
    args = vars(parser.parse_args())
    return args

def update_file_name(file, cfg, sim_cfg):
    MODE = str(cfg['mode'])
    file_name = file +  "_cmode_" + MODE + ".csv"
    return file_name

def _global_update(ROBOT, kwargs):
    """
    Update global configuration settings with robot and simulation data.

    Args:
        ROBOT: An object representing the robot.
        kwargs (dict): A dictionary containing various data related to the robot and simulation.
    """
    global_cfg.ROBOT_CFG.global_COM_xyz = list(kwargs['COM'])
    global_cfg.ROBOT_CFG.global_COM_ang = list(p.getEulerFromQuaternion(kwargs['linkWorldOrientation']))
    global_cfg.ROBOT_CFG.EE['FL_FOOT'] = list(kwargs['FL_FOOT'])
    global_cfg.ROBOT_CFG.EE['FR_FOOT'] = list(kwargs['FR_FOOT'])
    global_cfg.ROBOT_CFG.EE['HL_FOOT'] = list(kwargs['HL_FOOT'])
    global_cfg.ROBOT_CFG.EE['HR_FOOT'] = list(kwargs['HR_FOOT'])
    global_cfg.ROBOT_CFG.runtime = ROBOT.time
    global_cfg.ROBOT_CFG.joint_state = ROBOT.jointstate
    if sim_cfg['skip_forward_idx'] > 1:
        global_cfg.RUN.step += sim_cfg['skip_forward_idx'] + 1
    else:
        global_cfg.RUN.step += 1
    
def simulation(args):
    """
    Main simulation interface that runs the Bullet physics engine.

    This function initializes and runs a simulation using the Bullet physics engine based on the provided arguments.

    Parameters:
        args (dict): A dictionary containing simulation configuration and robot information.

    Notes:
        This function performs the following steps:
        1. Initializes various simulation-related variables and settings.
        2. Configures the simulation based on the provided simulation mode ('QTOS' or other).
        3. Initializes the PyBullet interface or a custom camera view if specified.
        4. Starts the simulation with the provided robot and initial state.
        5. Conducts an initial stance phase to position the robot properly.
        6. Enters the main simulation loop, which controls the robot according to the specified mode.
        7. Updates the simulation and robot state in each loop iteration.
        8. Checks for termination conditions, such as reaching the goal or completing trajectory steps.
        9. Handles custom camera view updates if configured.
        10. Optionally plots tracking data if tracking is enabled.
        11. Disconnects from the simulation.

    Args:
        args (dict): A dictionary containing simulation configuration and robot information.

    Returns:
        None
    """

    """============Sim-Init-Variables============"""
    goal = global_cfg.ROBOT_CFG.robot_goal
    if args.get('sim_cfg'):
        sim_cfg = args['sim_cfg']
    log = Logger("./logs", "simulation_log")
    ROBOT = args['robot']
    init_phase = sim_cfg['stance_phase']
    last_loop_time = time.time()
    sim_step = 1
    start_STATE = None
    stance_step = 0
    update_data = {"ROBOT": ROBOT, "sim_step": sim_step,
                  "TRACK_RECORD": None, "v_planner" : None,  "pybullet_interface": None,
                  "ref_cmd" : None}
    """============SIM-CONFIGURATION============"""
    csv_file = open(PLAN, 'r', newline='')
    reader = csv.reader(csv_file, delimiter=',')
    TRAJ_SIZE = sum(1 for row in reader)
    traj = np.genfromtxt(PLAN, delimiter=',')
    first_traj_point = traj[5]
    update_data['v_planner'] = Visual_Planner(PLAN, sim_cfg)
    time_step, EE_POSE = first_traj_point[0], first_traj_point[1:]
    ref_start_cmd = vec_to_cmd_pose(EE_POSE)
    q_init, _, _ = ROBOT.control_multi(ref_start_cmd, ROBOT.EE_index['all'], mode=ROBOT.mode)   
    start_STATE = EE_POSE 
    if sim_cfg.get('track'):
        TRACK_RECORD = Tracking(ROBOT, TRAJ_SIZE, sim_cfg)
        update_data['TRACK_RECORD'] = TRACK_RECORD
    if sim_cfg.get('py_interface'):
        pybullet_interface = PybulletInterface()
        update_data['pybullet_interface'] = pybullet_interface
    elif sim_cfg.get('custom_camera_view'):
        pybullet_interface = RecordInterface(args, ROBOT.robot)
        update_data['pybullet_interface'] = pybullet_interface
    """=========================================="""
    args['sim'].start(ROBOT, start_STATE)
    """==Init stance to force robot in good starting position=="""
    while (init_phase):
        loop_time = time.time() - last_loop_time
        if loop_time > sim_cfg['TIMESTEPS']:
            if stance_step >= sim_cfg.get('stance_period'):
                init_phase = False
                ROBOT._motor.set_motor_gains(ROBOT._kp, ROBOT._kd)
                break
            if init_phase:
                _, _, joint_toq = ROBOT.default_stance_control(q_init)
                p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=joint_toq)
                p.stepSimulation()
                last_loop_time = time.time()
                stance_step += 1
    """============Main-running-loop============"""
    while (sim_step < sim_cfg["SIM_STEPS"]):
        if sim_step < sim_cfg["TRAJ_SIZE"]:
            loop_time = time.time() - last_loop_time
            if loop_time > sim_cfg['TIMESTEPS']:
                try:
                    if global_cfg.RUN._done:
                        print("ROBOT REACHED THE GOAL!")
                        break
                    if global_cfg.RUN._wait: #Waits for local planner thread to copy over the trajectory
                        time.sleep(0.001)
                        continue
                    elif global_cfg.RUN._update:
                        print("============UPDATE STATE============")
                        mutex.acquire()
                        reader = csv.reader(open(PLAN, 'r', newline=''))
                        global_cfg.RUN._update = False 
                        global_cfg.RUN.step = 0
                        mutex.release()
                    traj = np.array([float(x) for x in next(reader)])
                    time_step, EE_POSE = traj[0], traj[1:]
                    global_cfg.ROBOT_CFG.last_POSE = EE_POSE[0:3]
                    towr_traj = towr_transform(ROBOT, vec_to_cmd_pose(EE_POSE))
                    update_data['ref_cmd'] = vec_to_cmd_pose(EE_POSE)
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
                sim_step = update(update_data, sim_cfg)
                if global_cfg.ROBOT_CFG.global_COM_xyz[0] >= goal[0]:
                    global_cfg.RUN._done = True
                    global_cfg.RUN._stance = True
                    break
                last_loop_time = time.time()
                p.stepSimulation()
                #==============Update Simulation State=================##
                write(log, EE_POSE)
        else:
            loop_time = time.time() - last_loop_time
            if loop_time > sim_cfg['TIMESTEPS']:
                towr_traj = towr_transform(ROBOT, vec_to_cmd_pose(EE_POSE))
                joint_ang, joint_vel, joint_toq = ROBOT.control_multi(towr_traj, ROBOT.EE_index['all'], mode=ROBOT.mode)
                ROBOT.set_joint_control_multi(ROBOT.jointidx['idx'], ROBOT.mode, joint_ang, joint_vel, joint_toq)
                p.stepSimulation()
                last_loop_time = time.time()
                sim_step += 1
    """============Main-running-loop============"""
    if sim_cfg.get('track'):
        TRACK_RECORD.plot()
    p.disconnect()

def write(log, EE_POSE):
    """
    Write data to a log file.

    Parameters:
        log (Logger): An instance of a logger for writing log data.
        EE_POSE (list): A list containing End-Effector (EE) position information.

    Args:
        log (Logger): An instance of a logger for writing log data.
        EE_POSE (list): A list containing End-Effector (EE) position information.
    """
    log.write(f"TIME STEP ==> {global_cfg.RUN.step}\n")
    log.write(f"Towr CoM POS -> {EE_POSE[0:3]}\n")
    log.write(f"Global POS -> {global_cfg.ROBOT_CFG.global_COM_xyz}\n")
    log.write(f"=========Global Vars=========\n")
    log.write(f"{global_cfg.print_vars(log.log)}\n")

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
    if sim_cfg.get('track'):
        data_dic['TRACK_RECORD'].update(data_dic['ref_cmd'], data_dic['ROBOT'].time_step)
    _global_update(data_dic['ROBOT'], data_dic['ROBOT'].state)
    data_dic['v_planner'].step(data_dic['sim_step'], data_dic['ROBOT'].time)
    data_dic['ROBOT'].update()
    data_dic['sim_step'] += 1
    if sim_cfg['custom_camera_view']:
        data_dic['pybullet_interface'].update()
    return data_dic['sim_step']

def init(args):
    """Default configuration for default TOWR based planning

    Args:
        args (dict): user + config inputs for simulation and solver
    """
    def start_config(args):
            args['-s'] = [0, 0, 0.24]
            args['-e1'] = [0.21, 0.19, 0.0]
            args['-e2'] = [0.21, -0.19, 0.0]
            args['-e3'] = [-0.21, 0.19, 0.0]
            args['-e4'] = [-0.21, -0.19, 0.0]
            args['-s_ang'] = [0, 0, 0]
    start_config(args)
    args['-r'] = 30 * args['sim'].num_tiles
    args['-g'] = args['args']['goal']
    args['-duration'] = 2.5 * args['sim'].num_tiles
    subprocess.run(shlex.split(args['scripts']['delete']))
    subprocess.run(shlex.split(args['scripts']['touch_file']))
    DEFAULT_SCRIPT = shlex.split(args['scripts']['run'] + " " + cmd_args(args))
    subprocess.run(DEFAULT_SCRIPT, stderr=subprocess.STDOUT)
    subprocess.run(shlex.split(scripts['copy']))
    return args

if __name__ == "__main__":
    args = parser()    
    if args.get('towr'):
        print("Default Test")
        docker_id = DockerInfo()
        args['sim_cfg'] = experimentInfo(args['experiment'])
        args['scripts'] = parse_scripts(scripts, docker_id)
        sim_cfg = args['sim_cfg']
        PLAN = "./test/data/traj/towr.csv"
        args.update(builder(sim_cfg=args['sim_cfg']))
        init(args)
        simulation(args)
    else:
        args = builder()
        simulation(args)

