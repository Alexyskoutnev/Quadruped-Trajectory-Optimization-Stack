#! /usr/bin/python3

import time
import subprocess
import shlex
import argparse
import copy
import sys
import csv
import os
from threading import Thread, Lock

import run
import trajectory_record

import numpy as np
import yaml

import QTOS.config.global_cfg as global_cfg
from QTOS.utils import *
from QTOS.global_planner import MPC, MPC_THREAD
from QTOS.logger import Logger
from QTOS.simulation import Simulation
from QTOS.builder import builder

CURRENT_TRAJ_CSV_FILE = "./data/traj/towr.csv"
NEW_TRAJ_CSV_FILE = "/tmp/towr.csv"

mutex = Lock()

def mpc_update_thread(mpc):
    """MPC thread that runs the global planner

    Args:
        mpc (MPC): MPC object that interfaces with the global planner
    """
    while True:
        mpc.update()

def _update(args, log, mpc):
    """Update function for the MPC controller

    Args:
        args (dict): Q-TOS user arguments + hyperparameters
        log (log): logger for simulator and local planner
        mpc (MPC): MPC object that interfaces with the global planner
    """
    _wait = False
    while (global_cfg.RUN._run_update_thread):
            mutex.acquire()
            mpc.update()
            mutex.release()
            time.sleep(0.001)
            if mpc.goal_diff < 0.1 or global_cfg.RUN._done:
                print("Robot reach the goal!")
                global_cfg.RUN._stance = True
                global_cfg.RUN._wait = False
                global_cfg.RUN._update = False
                global_cfg.RUN._done = True
                break
            elif not _wait:
                args = mpc.plan(args)
                MPC_SCRIPT = shlex.split(args['scripts']['run'] + " " + cmd_args(args))
                p_status = subprocess.run(MPC_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
                _wait = True
            elif mpc.cutoff_idx >= args['f_steps']:
                global_cfg.RUN._wait = True
                p = subprocess.run(shlex.split(scripts['data'])) 
                mpc.update()
                mpc.combine()
                p = subprocess.run(shlex.split(scripts['copy_tmp']))
                global_cfg.RUN._update = True
                global_cfg.RUN._wait = False
                _wait = False
                while (not global_cfg.RUN._update):
                            print("towr thread waiting")

def _init(args):
    """Configure trajectory solver in proper restart state

    Args:
        args (dict): user + config inputs for simulation and solver

    Returns:
        log: logger to log the status/state of the trajectory solver + simulation
    """
    log = Logger("./logs", "towr_log")
    global_cfg.ROBOT_CFG.robot_goal = args['-g']
    subprocess.run(shlex.split(args['scripts']['delete']))
    subprocess.run(shlex.split(args['scripts']['touch_file']))
    subprocess.run(shlex.split(args['scripts']['heightfield_rm']))
    subprocess.run(shlex.split(args['scripts']['heightfield_copy']))
    args['-resolution'] = 0.01 if args['sim_cfg'].get('resolution') is None else args['sim_cfg']['resolution'] 
    return log

def _run(args):
    """launch function to start the simulation and the solver concurrently

    Args:
        args (dict): user + config inputs for simulation and solver
    """
    log = _init(args)
    mpc = MPC(args, CURRENT_TRAJ_CSV_FILE, NEW_TRAJ_CSV_FILE, lookahead=args['look_ahead'])
    mpc.plan_init(args)
    MPC_SCRIPT = shlex.split(args['scripts']['run'] + " " + cmd_args(args))
    p = subprocess.run(MPC_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
    p_copy = subprocess.run(shlex.split(scripts['copy']))
    if p_copy.returncode == 0:
        print("Launching Simulation")
        mpc_thread = Thread(target=_update, args=(args, log, mpc))
        mpc_thread.start()
        if args.get('record'):
            trajectory_record.record_simulation(args)
        else:
            run.simulation(args)
    else: 
        print("Error in copying MPC Trajectory")
        sys.exit(1)

def default_init(args):
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
    args['-r'] = 120 * args['sim'].num_tiles
    if args['sim_cfg'].get('goal'):
        args['-g'] = args['args']['goal']
    args['-duration'] = 4.0 * args['sim'].num_tiles
    args['-resolution'] = 0.01 if args['sim_cfg'].get('resolution') is None else args['sim_cfg']['resolution'] 
    global_cfg.ROBOT_CFG.robot_goal = args['-g']
    subprocess.run(shlex.split(args['scripts']['delete']))
    subprocess.run(shlex.split(args['scripts']['touch_file']))
    DEFAULT_SCRIPT = shlex.split(args['scripts']['run'] + " " + cmd_args(args))
    subprocess.run(DEFAULT_SCRIPT, stderr=subprocess.STDOUT)
    subprocess.run(shlex.split(scripts['copy']))
    subprocess.run(shlex.split(args['scripts']['heightfield_copy']))
    return p

def run_default(args):
    """Launch function for default local planner dervived from TOWR

    Args:
        args (dict): user + config inputs for simulation and solver
    """
    default_init(args)
    run.simulation(args)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-g', '--g', nargs=3, type=float, default=[3.0,0,0.24], help="The initial global position of the robot")
    parser.add_argument('-s', '--s', nargs=3, type=float, default=[0.0, 0.0, 0.24], help="The initial position of the robot")
    parser.add_argument('-s_ang', '--s_ang', nargs=3, type=float, help="The initial angular velocity of the robot")
    parser.add_argument('-s_vel', '--s_vel', nargs=3, type=float, help="The initial velocity of the robot")
    parser.add_argument('-e1', '--e1', nargs=3, type=float, help="The starting state for front left leg")
    parser.add_argument('-e2', '--e2', nargs=3, type=float, help="The starting state for front right leg")
    parser.add_argument('-e3', '--e3', nargs=3, type=float, help="The starting state for back right leg")
    parser.add_argument('-e4', '--e4', nargs=3, type=float, help="The starting state for back left leg")
    parser.add_argument('-step', '--step', type=float, default=2.0, help="The step size to traverse along the global trajectory spline")
    parser.add_argument('-forced_steps', '--f_steps', type=int, default=2500, help="The amount of timesteps to force the robot to run and then start stitching the next planned trajectory")
    parser.add_argument('-l', '--look', type=float, default=3750, help='Number of timesteps to lookahead in the planned trajectory (the next starting state for the optimizer)')
    parser.add_argument('-r', '--record', type=bool, default=False, help="Record the joint-angle, joint-velocity, and torque while running the simulator")
    parser.add_argument('-exp', '--experiment', type=str, default="default", help="Selects the experiment to run in the simulator")
    parser.add_argument('-p', '--mpc_p', type=bool, default=False, help="Switch for global plan correction")
    parser.add_argument('-t', '--towr', action="store_true", help="Default TOWR local planner", default=False)
    p_args = parser.parse_args()
    docker_id = DockerInfo()
    args = {"-s": p_args.s, "-g": p_args.g, "-s_ang": p_args.s_ang, "s_ang": p_args.s_vel, "-e1": p_args.e1, 
            "-e2": p_args.e2, "-e3": p_args.e3, "-e4": p_args.e4, docker_id : docker_id,
            "scripts": parse_scripts(scripts, docker_id), "step_size": p_args.step, "look_ahead": p_args.look,
            "f_steps": p_args.f_steps, "record": p_args.record, "mpc_p": p_args.mpc_p, "towr" : p_args.towr}
    args['sim_cfg'] = experimentInfo(p_args.experiment)
    args.update(builder(sim_cfg=args['sim_cfg']))
    if args.get('towr'):
        print("Default Test")
        args['-r'] = 30 * args['sim'].num_tiles
        if args['sim_cfg'].get('goal'):
            args['-g'] = args['args']['goal']
        args['-duration'] = 1.0 * args['sim'].num_tiles
        run_default(args)
    elif args['sim_cfg'].get('goal'):
        args['-g'] = args['sim_cfg']['goal']
    else:
        args['-g'][0] = (args['sim'].num_tiles - 1) * 2.0 + 0.5
    _run(args)

if __name__ == "__main__":
    main()