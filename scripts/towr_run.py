#! /usr/bin/python3

import time
import subprocess
import shlex
import argparse
import copy
import sys
import csv
from threading import Thread, Lock


import run
import collect_towr_data
import numpy as np
import yaml

import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg
from SOLO12_SIM_CONTROL.utils import norm, tf_2_world_frame, percentage_look_ahead, zero_filter, transformation_mtx, transformation_inv, transformation_multi
from SOLO12_SIM_CONTROL.mpc import MPC, MPC_THREAD
from SOLO12_SIM_CONTROL.logger import Logger
from SOLO12_SIM_CONTROL.simulation import Simulation
from SOLO12_SIM_CONTROL.builder import builder

scripts =  {'copy_tmp': 'cp /tmp/towr.csv ./data/traj/towr.csv',
            'copy': 'docker cp <id>:root/catkin_ws/src/towr/towr/build/traj.csv ./data/traj/towr.csv',
            'run': 'docker exec <id> ./main',
            'info': 'docker ps -f ancestor=towr',
            'data': 'docker cp <id>:root/catkin_ws/src/towr/towr/build/traj.csv /tmp/towr.csv',
            'delete': 'rm ./data/traj/towr.csv',
            'heightfield_rm' : 'docker exec -t <id> rm /root/catkin_ws/src/towr/towr/data/heightfields/from_pybullet/towr_heightfield.txt',
            'heightfield_copy': 'docker cp ./data/heightfields/from_pybullet/towr_heightfield.txt <id>:root/catkin_ws/src/towr/towr/data/heightfields/from_pybullet/towr_heightfield.txt'}

_flags = ['-g', '-s', '-s_ang', '-s_vel', '-n', '-e1', '-e2', '-e3', '-e4', '-t']

CURRENT_TRAJ_CSV_FILE = "./data/traj/towr.csv"
NEW_TRAJ_CSV_FILE = "/tmp/towr.csv"
config_sim = "./data/config/simulation.yml"
sim_cfg = yaml.safe_load(open(config_sim, 'r'))

mutex = Lock()

def strip(x):
    st = " "
    for s in x:
        if s == "[" or s == "]":
            st += ''
        else:
            st += s
    return st

def DockerInfo():
    p = subprocess.run([scripts['info']], shell=True, capture_output=True, text=True)
    output = p.stdout.replace('\n', ' ')
    dockerid, idx = output.split(), output.split().index('towr') - 1
    return dockerid[idx]

def parse_scripts(scripts_dic, docker_id):
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

def _state(p = 0.6):
    state = {"CoM": None, "orientation": None, "FL_FOOT": None, 
             "FR_FOOT": None, "HL_FOOT": None, "HR_FOOT": None}
    with open(CURRENT_TRAJ_CSV_FILE, "r", newline='') as f:
        reader = percentage_look_ahead(f, p)
        row = next(reader)
        state["CoM"] = [float(_) for _ in row[0:3]]
        state["orientation"] = [float(_) for _ in row[3:6]]
        state["FL_FOOT"] = [float(_) for _ in row[6:9]]
        state["FR_FOOT"] = [float(_) for _ in row[9:12]]
        state["HL_FOOT"] = [float(_) for _ in row[12:15]]
        state["HR_FOOT"] = [float(_) for _ in row[15:18]]
    state = {key: zero_filter(value) for key, value in state.items()}
    return state

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

def _plan(args):
    """
    Trajcetory plan towards the final goal
    """
    args = _step(args)
    _state_dic = _state()
    args['-s'] = _state_dic["CoM"]
    args['-e1'] = _state_dic["FL_FOOT"]
    args['-e2'] = _state_dic["FR_FOOT"]
    args['-e3'] = _state_dic["HL_FOOT"]
    args['-e4'] = _state_dic["HR_FOOT"]
    return args

def mpc_update_thread(mpc):
    while True:
        mpc.update()

def _update(args, log, mpc):
    """
    Threaded towr trajectory update function
    """
    _wait = False
    # mpc = MPC(args, CURRENT_TRAJ_CSV_FILE, NEW_TRAJ_CSV_FILE, lookahead=args['look_ahead'])
    while (global_cfg.RUN._run_update_thread):
            mutex.acquire()
            mpc.update()
            mutex.release()
            time.sleep(0.01)
            if mpc.goal_diff < 0.05:
                print("Robot reach the goal!")
                global_cfg.RUN._stance = True
                global_cfg.RUN._wait = False
                global_cfg.RUN._update = False
                global_cfg.RUN._done = True
                break
            elif not _wait:
                # global_cfg.RUN._wait = True
                # breakpoint()
                args = mpc.plan(args)
                TOWR_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
                print(f"Updated with a new plan - \n {TOWR_SCRIPT}")
                p_status = subprocess.run(TOWR_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
                print(f"P_status {p_status}")
                _wait = True
                # breakpoint()
                # global_cfg.RUN._wait = False
            elif p_status.returncode == 0 and mpc.cutoff_idx >= args['f_steps']:
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
    """Configure trajectory solver in proper start state

    Args:
        args (dict): user + config inputs for simulation and solver

    Returns:
        log: logger to log the status/state of the trajectory solver + simulation
    """
    log = Logger("./logs", "towr_log")
    global_cfg.ROBOT_CFG.robot_goal = args['-g']
    subprocess.run(shlex.split(args['scripts']['heightfield_rm']))
    subprocess.run(shlex.split(args['scripts']['heightfield_copy']))
    args = _step(args)
    args['map'] = args['sim'].height_map
    return log

def _run(args):
    """launch function to start the simulation and the solver concurrently

    Args:
        args (dict): user + config inputs for simulation and solver
    """
    log = _init(args)
    mpc = MPC(args, CURRENT_TRAJ_CSV_FILE, NEW_TRAJ_CSV_FILE, lookahead=args['look_ahead'])
    mpc.plan_init(args)
    TOWR_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
    p = subprocess.run(TOWR_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
    if p.returncode == 0:
        p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
        if p.returncode == 0:
            print("Launching Simulation")
            towr_thread = Thread(target=_update, args=(args, log, mpc))
            towr_thread.start()
            run.simulation(args)
        else: 
            print("Error in copying Towr Trajectory")
            sys.exit(1)
    else:
        print("Error Generating Towr Trajectory")
        sys.exit(1)

def test_mpc_single_loop(args):
    """TO BE REMOVE"""
    log = _init(args)
    mpc = MPC(args, CURRENT_TRAJ_CSV_FILE, NEW_TRAJ_CSV_FILE, lookahead=args['look_ahead'])
    mpc.plan_init(args)
    TOWR_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
    p = subprocess.run(TOWR_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
    if p.returncode == 0:
        p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
        if p.returncode == 0:
            _update(args, log, mpc)
    else:
        print("Error Generating Towr Trajectory")
        sys.exit(1)

if __name__ == "__main__":
    test = False
    parser = argparse.ArgumentParser()
    parser.add_argument('-g', '--g', nargs=3, type=float, default=[5.0,0,0.24])
    parser.add_argument('-s', '--s', nargs=3, type=float)
    parser.add_argument('-s_ang', '--s_ang', nargs=3, type=float)
    parser.add_argument('-s_vel', '--s_vel', nargs=3, type=float)
    parser.add_argument('-n', '--n', nargs=1, type=str, default="t")
    parser.add_argument('-e1', '--e1', nargs=3, type=float)
    parser.add_argument('-e2', '--e2', nargs=3, type=float)
    parser.add_argument('-e3', '--e3', nargs=3, type=float)
    parser.add_argument('-e4', '--e4', nargs=3, type=float)
    parser.add_argument('-step', '--step', type=float, default=0.50)
    parser.add_argument('-forced_steps', '--f_steps', type=int, default=2500)
    parser.add_argument('-l', '--look', type=float, default=3750)
    parser.add_argument('-r', '--record', type=bool, default=False)
    parser.add_argument('-p', '--mpc_p', type=bool, default=False)
    p_args = parser.parse_args()
    docker_id = DockerInfo()
    args = {"-s": p_args.s, "-g": p_args.g, "-s_ang": p_args.s_ang, "s_ang": p_args.s_vel, "-n": p_args.n,
            "-e1": p_args.e1, "-e2": p_args.e2, "-e3": p_args.e3, "-e4": p_args.e4, docker_id : docker_id,
            "scripts": parse_scripts(scripts, docker_id), "step_size": p_args.step, "look_ahead": p_args.look,
            "f_steps": p_args.f_steps, "record": p_args.record, "mpc_p": p_args.mpc_p}
    if test:
        args.update(builder())
        args['-g'][0] = (args['sim'].num_tiles - 1) * 1.0 + 0.5
        test_mpc_single_loop(args)
    else:
        args.update(builder())
        args['-g'][0] = (args['sim'].num_tiles - 1) * 1.0 + 1.0
        _run(args)