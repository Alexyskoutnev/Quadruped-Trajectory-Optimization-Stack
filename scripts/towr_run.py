#! /usr/bin/python3

import time
import subprocess
import shlex
import argparse
import copy
from threading import Thread

import run

import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg

scripts =  {'copy': 'docker cp <id>:/root/catkin_ws/src/towr/towr/build/traj.csv ./data/traj/towr.csv',
            'run': 'docker exec <id> ./towr-example',
            'info': 'docker ps -f ancestor=towr'}

_flags = ['-g', '-s', '-s_ang', '-s_vel']

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


        
def _update(args, log):
    """
    Threaded towr trajectory update function
    """
    def _update_step(args):
        global_pos = copy.copy(global_cfg.ROBOT_CFG.linkWorldPosition)
        # global_pos[0] += 0.25
        # global_pos = [0]
        # args['-s'] = global_cfg.ROBOT_CFG.linkWorldPosition
        args['-s'] = [0, 0, 0.21]
        args['-s__ang'] = global_cfg.ROBOT_CFG.linkWorldOrientation
        args['-g'] = [0.5, 0, 0.21]
        return args
    i = 0
    while (True):
        args = _update_step(args)
        print(f"i {i}", args)
        TOWR_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
        p = subprocess.run(TOWR_SCRIPT, stdout=log, stderr=subprocess.STDOUT)
        if p.returncode == 0:
            print("TOWR found a trajectory")
            p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
            print("update the csv")
            global_cfg.RUN = True
            while (not global_cfg.RUN):
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
    TOWR_SCRIPT = shlex.split(args['scripts']['run'] + " " + _cmd_args(args))
    p = subprocess.run(TOWR_SCRIPT, stdout=log, stderr=subprocess.STDOUT)
    towr_runtime_1 = time.time()
    print(f'TOWR Execution time: {towr_runtime_1 - towr_runtime_0} seconds')

    if p.returncode == 0:
        print("TOWR found a trajectory")
        p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
        if p.returncode == 0:
            #Start threading process
            # simulation_thread = Thread(target=run.simulation)
            towr_thread = Thread(target=_update, args=(args, log))
            towr_thread.start()
            print("RUNNING sim")
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
    parser.add_argument('-g', '--g', nargs=3, type=float, default=[0.5,0,0.21])
    parser.add_argument('-s', '--s', nargs=3, type=float)
    parser.add_argument('-s_ang', '--s_ang', nargs=3, type=float)
    parser.add_argument('-s_vel', '--s_vel', nargs=3, type=float)
    p_args = parser.parse_args()
    docker_id = DockerInfo()
    args = {"-s": p_args.s, "-g": p_args.g, "-s_ang": p_args.s_ang, "s_ang": p_args.s_vel, 
            "docker_id": docker_id, "scripts": parse_scripts(scripts, docker_id)}
    _run(args)