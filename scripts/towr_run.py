#! /usr/bin/python3

import time
import subprocess
import shlex
import argparse

import run

scripts =  {'copy': 'docker cp <id>:/root/catkin_ws/src/towr/towr/build/traj.csv ./data/traj/towr.csv',
            'run': 'docker exec <id> ./towr-example',
            'info': 'docker ps -f ancestor=towr'}

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
        
def _update(args):
    """
    Threaded towr trajectory update function
    """
    TOWR_SCRIPT = shlex.split(args['scripts']['run'] + strip(str(args['start_pos'])))
    p = subprocess.run(TOWR_SCRIPT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    if p.returncode == 0:
        print("TOWR found a trajectory")
        p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
        if p.returncode == 0:
            run.simulation()
        else: 
            print("Error in copying Towr Trajectory")
    else:
        print("Error in copying Towr Trajectory")

def _run(args):
    towr_runtime_0 = time.time()
    TOWR_SCRIPT = shlex.split(args['scripts']['run'] + strip(str(args['start_pos'])))
    p = subprocess.run(TOWR_SCRIPT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    towr_runtime_1 = time.time()
    print(f'TOWR Execution time: {towr_runtime_1 - towr_runtime_0} seconds')
    if p.returncode == 0:
        print("TOWR found a trajectory")
        p = subprocess.run(shlex.split(scripts['copy'])) #copy trajectory to simulator data
        if p.returncode == 0:
            run.simulation()
        else: 
            print("Error in copying Towr Trajectory")
    else:
        print("Error in TOWR")
        return

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--d_pos', nargs=3, type=float, default=[1,0,0.3])
    p_args = parser.parse_args()
    docker_id = DockerInfo()
    args = {"start_pos": p_args.d_pos, "docker_id": docker_id, "scripts": parse_scripts(scripts, docker_id)}
    _run(args)