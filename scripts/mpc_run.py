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

import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg
from SOLO12_SIM_CONTROL.utils import norm, tf_2_world_frame, percentage_look_ahead, zero_filter, transformation_mtx, transformation_inv, transformation_multi
from SOLO12_SIM_CONTROL.mpc import MPC, MPC_THREAD
from SOLO12_SIM_CONTROL.logger import Logger
from SOLO12_SIM_CONTROL.simulation import Simulation
from SOLO12_SIM_CONTROL.builder import builder

scripts =  {'copy_tmp': 'cp /tmp/towr.csv ./data/traj/towr.csv',
            'copy': 'docker cp <id>:root/catkin_ws/src/towr_solo12/towr/build/traj.csv ./data/traj/towr.csv',
            'run': 'docker exec <id> ./main',
            'info': 'docker ps -f ancestor=towr',
            'data': 'docker cp <id>:root/catkin_ws/src/towr_solo12/towr/build/traj.csv /tmp/towr.csv',
            'delete': 'rm ./data/traj/towr.csv',
            'heightfield_rm' : 'docker exec -t <id> rm /root/catkin_ws/src/towr_solo12/towr/data/heightfields/from_pybullet/towr_heightfield.txt',
            'heightfield_copy': 'docker cp ./data/heightfields/from_pybullet/towr_heightfield.txt <id>:root/catkin_ws/src/towr_solo12/towr/data/heightfields/from_pybullet/towr_heightfield.txt',
            'touch_file' : 'touch ./data/traj/towr.csv',
}

_flags = ['-g', '-s', '-s_ang', '-s_vel', '-n', '-e1', '-e2', '-e3', '-e4', '-t', '-r', '-resolution', 's_vel', 's_ang_vel']

CURRENT_TRAJ_CSV_FILE = "./data/traj/towr.csv"
NEW_TRAJ_CSV_FILE = "/tmp/towr.csv"

mutex = Lock()

def DockerInfo():
    """Helper function that finds the ID of the Docker runnning on the host system
    """
    p = subprocess.run([scripts['info']], shell=True, capture_output=True, text=True)
    output = p.stdout.replace('\n', ' ')
    dockerid, idx = output.split(), output.split().index('towr') - 1
    return dockerid[idx]

def experimentInfo(experiement_name):
    """Helper function to setup different terrain experiments to test the Q-TOS Stack

    Args:
        experiement_name (string): name of experiment the user is trying to test

    Returns:
        sim_cfg: The corresponding configuration file to setup + run the the experiment
    """
    experiment_names = {"default": "simulation.yml", "exp_1": "experiment_1_straight_line.yml",
                        "exp_2": "experiment_2_climbing.yml", "exp_3": "experiment_3_collision_avoidance.yml",
                        "exp_4" : "experiment_4_rough_terrain.yml", "exp_5": "experiment_5_extreme_climbing.yml",
                        "exp_6" : "experiment_6_stairs.yml", "FSS_Plot" : "create_FSS_plots.yml",
                        "exp_7" : "experiment_7_climb_obstacle.yml"}
    file_path = os.path.join("./data/config", experiment_names[experiement_name])
    sim_cfg = yaml.safe_load(open(file_path, 'r'))
    return sim_cfg

def parse_scripts(scripts_dic, docker_id):
    """Helper function that parsers through the user command arguments

    Args:
        scripts_dic (dict): The available Bash script commands for the user
        docker_id (dict): The ID of the Docker container running the local planner

    Returns:
        scripts_dic: Formated Bash scripts commands for Python process execution
    """
    for script_name, script in scripts_dic.items():
        scripts_dic[script_name] = script.replace("<id>", docker_id)
    return scripts_dic

def start_config(args):
    args['-s'] = [0, 0, 0.24]
    args['-e1'] = [0.20590930477664196, 0.14927536747689948, 0.0]
    args['-e2'] = [0.2059042161427424, -0.14926921805769638, 0.0]
    args['-e3'] = [-0.20589422629511542, 0.14933201572367907, 0.0]
    args['-e4'] = [-0.2348440184502048, -0.17033609357109808, 0.0]
    args['-s_ang'] = [0, 0, 0]

def _step(args):
    start_config(args)
    step_size = args['step_size'] #try implementing in config-file
    global_pos = np.array(global_cfg.ROBOT_CFG.linkWorldPosition)
    goal = global_cfg.ROBOT_CFG.robot_goal
    diff_vec = np.clip(goal - global_pos, -step_size, step_size)
    diff_vec[2] = 0.0
    args['-g'] = list(global_pos + diff_vec)
    args['-g'][2] = 0.24
    return args

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
                MPC_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
                print(f"Updated with a new plan - \n {MPC_SCRIPT}")
                p_status = subprocess.run(MPC_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
                print(f"P_status {p_status}")
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
            
def _cmd_args(args):
    """Commandline parser to run python subprocesses correctly

    Args:
        args (dict): user + config inputs for simulation and solver

    Return:
        _cmd : parsed command line string that can be ran as a excutable
    """

    def _bracket_rm(s):
        return s.replace("[", "").replace("]", "")

    def _remove_comma(s):
        return s.replace(",", "")
    
    def _filter(s):
        return _bracket_rm(_remove_comma(str(s)))

    _cmd = ""

    for key, value in args.items():
        if key in _flags and value:
            _cmd += key + " " + _filter(value)
            _cmd += " "

    return _cmd

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
    args = _step(args)
    args['-resolution'] = 0.01 if args['sim_cfg'].get('resolution') is None else args['sim_cfg']['resolution'] 
    return log

def _run(args):
    """:aunch function to start the simulation and the solver concurrently

    Args:
        args (dict): user + config inputs for simulation and solver
    """
    log = _init(args)
    mpc = MPC(args, CURRENT_TRAJ_CSV_FILE, NEW_TRAJ_CSV_FILE, lookahead=args['look_ahead'])
    mpc.plan_init(args)
    MPC_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
    p = subprocess.run(MPC_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
    p_copy = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-g', '--g', nargs=3, type=float, default=[2.0,0,0.24])
    parser.add_argument('-s', '--s', nargs=3, type=float)
    parser.add_argument('-s_ang', '--s_ang', nargs=3, type=float)
    parser.add_argument('-s_vel', '--s_vel', nargs=3, type=float)
    parser.add_argument('-n', '--n', nargs=1, type=str, default="t")
    parser.add_argument('-e1', '--e1', nargs=3, type=float)
    parser.add_argument('-e2', '--e2', nargs=3, type=float)
    parser.add_argument('-e3', '--e3', nargs=3, type=float)
    parser.add_argument('-e4', '--e4', nargs=3, type=float)
    parser.add_argument('-step', '--step', type=float, default=1.0)
    parser.add_argument('-forced_steps', '--f_steps', type=int, default=2500)
    parser.add_argument('-l', '--look', type=float, default=3750)
    parser.add_argument('-r', '--record', type=bool, default=False)
    parser.add_argument('-exp', '--experiment', type=str, default="default")
    parser.add_argument('-p', '--mpc_p', type=bool, default=False)
    p_args = parser.parse_args()
    docker_id = DockerInfo()
    args = {"-s": p_args.s, "-g": p_args.g, "-s_ang": p_args.s_ang, "s_ang": p_args.s_vel, "-n": p_args.n,
            "-e1": p_args.e1, "-e2": p_args.e2, "-e3": p_args.e3, "-e4": p_args.e4, docker_id : docker_id,
            "scripts": parse_scripts(scripts, docker_id), "step_size": p_args.step, "look_ahead": p_args.look,
            "f_steps": p_args.f_steps, "record": p_args.record, "mpc_p": p_args.mpc_p}
    args['sim_cfg'] = experimentInfo(p_args.experiment)
    args.update(builder(sim_cfg=args['sim_cfg']))
    args['-g'][0] = (args['sim'].num_tiles - 1) * 2.0 + 0.5
    _run(args)

if __name__ == "__main__":
    main()