#! /usr/bin/python3

import time
import subprocess
import shlex
import argparse
import copy
from threading import Thread, Lock
import csv

import run
import collect_towr_data
import numpy as np

import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg
from SOLO12_SIM_CONTROL.utils import norm, tf_2_world_frame, percentage_look_ahead, zero_filter, transformation_mtx, transformation_inv, transformation_multi
from SOLO12_SIM_CONTROL.mpc import MPC, MPC_THREAD
from SOLO12_SIM_CONTROL.logger import Logger

scripts =  {'copy_tmp': 'cp /tmp/towr.csv ./data/traj/towr.csv',
            'copy': 'docker cp <id>:/root/catkin_ws/src/towr/towr/build/traj.csv ./data/traj/towr.csv',
            'run': 'docker exec <id> ./towr-example',
            'info': 'docker ps -f ancestor=towr',
            'data': 'docker cp <id>:/root/catkin_ws/src/towr/towr/build/traj.csv /tmp/towr.csv',
            'delete': 'rm ./data/traj/towr.csv'}

_flags = ['-g', '-s', '-s_ang', '-s_vel', '-n', '-e1', '-e2', '-e3', '-e4', '-t']

CURRENT_TRAJ_CSV_FILE = "./data/traj/towr.csv"
NEW_TRAJ_CSV_FILE = "/tmp/towr.csv"

lock = Lock()

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
    step_size = args['step_size'] #try implementing in config-file
    global_pos = np.array(global_cfg.ROBOT_CFG.linkWorldPosition)
    goal = global_cfg.ROBOT_CFG.robot_goal
    diff_vec = np.clip(goal - global_pos, -step_size, step_size)
    diff_vec[2] = 0.0
    args['-s'] = [0, 0, 0.24]
    # args['-s'][2] = 0.24
    args['-s_ang'] = global_cfg.ROBOT_CFG.linkWorldOrientation
    args['-e1'] = list(global_cfg.ROBOT_CFG.EE['FL_FOOT'])
    args['-e2'] = list(global_cfg.ROBOT_CFG.EE['FR_FOOT'])
    args['-e3'] = list(global_cfg.ROBOT_CFG.EE['HL_FOOT'])
    args['-e4'] = list(global_cfg.ROBOT_CFG.EE['HR_FOOT'])
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
        print("update")
        mpc.update()


def _update(args, log):
    """
    Threaded towr trajectory update function
    """
    _wait = False
    mpc = MPC(args, CURRENT_TRAJ_CSV_FILE, NEW_TRAJ_CSV_FILE, lookahead=args['look_ahead'])
    while (global_cfg.RUN._run_update_thread):
            mpc.update()
            time.sleep(0.01)
            if mpc.goal_diff < 0.05:
                print("Robot reach the goal!")
                global_cfg.RUN._stance = True
                global_cfg.RUN._wait = False
                global_cfg.RUN._update = False
            elif not _wait:
                args = mpc.plan(args)
                towr_runtime_0 = time.process_time()
                TOWR_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
                p_status = subprocess.run(TOWR_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
                towr_runtime_1 = time.process_time()
                print(f'TOWR Execution time: {towr_runtime_1 - towr_runtime_0:0.3f} seconds')
                _wait = True
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

def _run(args):
    log = Logger("./logs", "towr_log")
    global_cfg.ROBOT_CFG.robot_goal = args['-g']
    args = _step(args)
    towr_runtime_0 = time.process_time()
    TOWR_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
    p = subprocess.run(TOWR_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
    towr_runtime_1 = time.process_time()
    print(f'TOWR Execution time: {towr_runtime_1 - towr_runtime_0:0.3f} seconds')
    if p.returncode == 0:
        print("TOWR found a trajectory")
        p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
        if p.returncode == 0:
            towr_thread = Thread(target=_update, args=(args, log))
            towr_thread.start()
            if args.get('record'):
                run.simulation(args)
            else:
                run.simulation()
        else: 
            print("Error in copying Towr Trajectory")
    else:
        print("Error input trajectory cmds")
        print("running default commamd")
        TOWR_SCRIPT = shlex.split(args['scripts']['run'] + "-g 0.5 0.0 0.21 -s 0.0 0.0 0.21")
        p = subprocess.run(TOWR_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
        if p.returncode == 0:
            print("TOWR found a trajectory with default cmd")
            p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
            if p.returncode == 0:
                run.simulation()
            else: 
                print("Error in copying Towr Trajectory")
            return

def test_mpc(args):
    log = open("./logs/towr_log.out", "w")
    global_cfg.ROBOT_CFG.robot_goal = args['-g']
    args = _step(args)
    towr_runtime_0 = time.time()
    TOWR_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
    p = subprocess.run(TOWR_SCRIPT, stdout=log, stderr=subprocess.STDOUT)
    towr_runtime_1 = time.time()
    print(f'TOWR Execution time: {towr_runtime_1 - towr_runtime_0} seconds')
    if p.returncode == 0:
        print("TOWR found a trajectory")
        p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
        if p.returncode == 0:
            towr_thread = Thread(target=_update, args=(args, log))
            towr_thread.start()
            run.simulation()
        else: 
            print("Error in copying Towr Trajectory")

if __name__ == "__main__":
    test = False
    parser = argparse.ArgumentParser()
    parser.add_argument('-g', '--g', nargs=3, type=float, default=[10.0,0,0.24])
    parser.add_argument('-s', '--s', nargs=3, type=float)
    parser.add_argument('-s_ang', '--s_ang', nargs=3, type=float)
    parser.add_argument('-s_vel', '--s_vel', nargs=3, type=float)
    parser.add_argument('-n', '--n', nargs=1, type=str, default="t")
    parser.add_argument('-e1', '--e1', nargs=3, type=float)
    parser.add_argument('-e2', '--e2', nargs=3, type=float)
    parser.add_argument('-e3', '--e3', nargs=3, type=float)
    parser.add_argument('-e4', '--e4', nargs=3, type=float)
    parser.add_argument('-step', '--step', type=float, default=0.5)
    parser.add_argument('-forced_steps', '--f_steps', type=int, default=550)
    parser.add_argument('-l', '--look', type=float, default=750)
    parser.add_argument('-r', '--record', type=bool, default=False)
    p_args = parser.parse_args()
    docker_id = DockerInfo()
    args = {"-s": p_args.s, "-g": p_args.g, "-s_ang": p_args.s_ang, "s_ang": p_args.s_vel, "-n": p_args.n,
            "-e1": p_args.e1, "-e2": p_args.e2, "-e3": p_args.e3, "-e4": p_args.e4, docker_id : docker_id,
            "scripts": parse_scripts(scripts, docker_id), "step_size": p_args.step, "look_ahead": p_args.look,
            "f_steps": p_args.f_steps, "record": p_args.record}
    if test:
        test_mpc(args)
    else:
        _run(args)