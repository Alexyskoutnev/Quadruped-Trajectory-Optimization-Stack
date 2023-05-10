#! /usr/bin/python3

import time
import subprocess
import shlex
import argparse
import copy
from threading import Thread

import run
import numpy as np

import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg
from SOLO12_SIM_CONTROL.utils import norm, tf_2_world_frame

scripts =  {'copy_tmp': 'cp /tmp/towr.csv ./data/traj/towr.csv',
            'copy': 'docker cp <id>:/root/catkin_ws/src/towr/towr/build/traj.csv ./data/traj/towr.csv',
            'run': 'docker exec <id> ./towr-example',
            'info': 'docker ps -f ancestor=towr',
            'data': 'docker cp <id>:/root/catkin_ws/src/towr/towr/build/traj.csv /tmp/towr.csv'}

_flags = ['-g', '-s', '-s_ang', '-s_vel', '-n', '-e1', '-e2', '-e3', '-e4']

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
        
def _plan(args):
    """
    Trajcetory plan towards the final goal
    """
    step_size = 0.25 #try implementing in config-file
    global_pos = np.array(global_cfg.ROBOT_CFG.linkWorldPosition)
    goal = global_cfg.ROBOT_CFG.robot_goal
    diff_vec = np.clip(goal - global_pos, -step_size, step_size)
    CoM = {"linkWorldPosition": global_cfg.ROBOT_CFG.linkWorldPosition, "linkWorldOrientation": global_cfg.ROBOT_CFG.linkWorldOrientation}
    print("EE -> ", global_cfg.ROBOT_CFG.EE)
    diff_vec[2] = 0.0
    print("diff vec ", diff_vec)
    print("COM ", CoM)
    args['-g'] = list(global_pos + diff_vec)
    args['-g'][2] = 0.24
    args['-s'] = list(global_cfg.ROBOT_CFG.linkWorldPosition)
    # args['-s'][2] = 0.24
    # args['-e1'] = global_cfg.ROBOT_CFG.EE['FL_FOOT']
    # args['-e1'][0] = diff_vec[0]
    # args['-e1'][1] = diff_vec[1]
    # args['-e1'] = tf_2_world_frame(args['-e1'], CoM)
    # args['-e2'] = global_cfg.ROBOT_CFG.EE['FR_FOOT']
    # args['-e2'][0] += diff_vec[0]
    # args['-e2'][1] += diff_vec[1]
    # args['-e2'] = tf_2_world_frame(args['-e2'], CoM)
    # args['-e3'] = global_cfg.ROBOT_CFG.EE['HL_FOOT']
    # args['-e3'][0] += diff_vec[0]
    # args['-e3'][1] += diff_vec[1]
    # args['-e3'] = tf_2_world_frame(args['-e3'], CoM)
    # args['-e4'] = global_cfg.ROBOT_CFG.EE['HR_FOOT']
    # args['-e4'][0] += diff_vec[0]
    # args['-e4'][1] += diff_vec[1]
    # args['-e4'] = tf_2_world_frame(args['-e4'], CoM)
    print("ARRGS", args)
    return args
    

def _update(args, log):
    """
    Threaded towr trajectory update function
    """
    i = 0
    while (True):
            args = _plan(args)
            TOWR_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
            p = subprocess.run(TOWR_SCRIPT, stdout=log, stderr=subprocess.STDOUT)
            if global_cfg.RUN._wait:
                global_cfg.ROBOT_CFG.last_POSE = global_cfg.ROBOT_CFG.linkWorldPosition
                p = subprocess.run(shlex.split(scripts['copy_tmp']))
                global_cfg.RUN._wait = False
            elif p.returncode == 0:
                p = subprocess.run(shlex.split(scripts['data'])) #temp hold csv
                # breakpoint()
                print("norm ", norm(global_cfg.ROBOT_CFG.last_POSE, global_cfg.ROBOT_CFG.linkWorldPosition))
                if norm(global_cfg.ROBOT_CFG.last_POSE, global_cfg.ROBOT_CFG.linkWorldPosition) > 0.25:
                    global_cfg.ROBOT_CFG.last_POSE = global_cfg.ROBOT_CFG.linkWorldPosition
                    p = subprocess.run(shlex.split(scripts['copy_tmp'])) #copy trajectory to simulator data
                    global_cfg.RUN.update = True
                    while (not global_cfg.RUN.update):
                        print("towr thread waiting")
            else:
                print("Error in copying Towr Trajectory")

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
    log = open("./logs/towr_log.out", "w")
    towr_runtime_0 = time.time()
    global_cfg.ROBOT_CFG.robot_goal = args['-g']
    # args = _plan/(args)
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
    else:
        print("Error input trajectory cmds")
        print("running default commamd")
        TOWR_SCRIPT = shlex.split(args['scripts']['run'] + "-g 0.5 0.0 0.21 -s 0.0 0.0 0.21")
        p = subprocess.run(TOWR_SCRIPT, stdout=log, stderr=subprocess.STDOUT)
        if p.returncode == 0:
            print("TOWR found a trajectory with default cmd")
            p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
            if p.returncode == 0:
                run.simulation()
            else: 
                print("Error in copying Towr Trajectory")
            return

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-g', '--g', nargs=3, type=float, default=[1.0,0,0.24])
    parser.add_argument('-s', '--s', nargs=3, type=float)
    parser.add_argument('-s_ang', '--s_ang', nargs=3, type=float)
    parser.add_argument('-s_vel', '--s_vel', nargs=3, type=float)
    parser.add_argument('-n', '--n', nargs=1, type=str, default="t")
    parser.add_argument('-e1', '--e1', nargs=3, type=float)
    parser.add_argument('-e2', '--e2', nargs=3, type=float)
    parser.add_argument('-e3', '--e3', nargs=3, type=float)
    parser.add_argument('-e4', '--e4', nargs=3, type=float)
    p_args = parser.parse_args()
    docker_id = DockerInfo()
    args = {"-s": p_args.s, "-g": p_args.g, "-s_ang": p_args.s_ang, "s_ang": p_args.s_vel, "-n": p_args.n,
            "-e1": p_args.e1, "-e2": p_args.e2, "-e3": p_args.e3, "-e4": p_args.e4, docker_id : docker_id,
            "scripts": parse_scripts(scripts, docker_id)}
    _run(args)