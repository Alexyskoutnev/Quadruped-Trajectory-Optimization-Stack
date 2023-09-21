#! /usr/bin/python3

import time
import subprocess
import shlex
import argparse
import sys
from threading import Thread, Lock

import run, trajectory_record #importing running modules from run.py and trajectory_record.py

import numpy as np

import QTOS.config.global_cfg as global_cfg
from QTOS.utils import *
from QTOS.combiner import Combiner
from QTOS.logger import Logger
from QTOS.builder import builder

CURRENT_TRAJ_CSV_FILE = "./data/traj/towr.csv"
NEW_TRAJ_CSV_FILE = "/tmp/towr.csv"

mutex = Lock()

def _update(args, log, combiner):
    """Update function for the motion combiner

    Args:
        args (dict): Q-TOS user arguments + hyperparameters
        log (log): logger for simulator and local planner
        combiner: object that stitches the motion plans from global planner
    """
    _wait = False
    while (global_cfg.RUN._run_update_thread):
            mutex.acquire()
            combiner.update()
            mutex.release()
            time.sleep(0.001)
            if combiner.goal_diff < 0.1 or global_cfg.RUN._done:
                print("Robot reach the goal!")
                global_cfg.RUN._stance = True
                global_cfg.RUN._wait = False
                global_cfg.RUN._update = False
                global_cfg.RUN._done = True
                break
            elif not _wait:
                args = combiner.plan(args)
                combiner_SCRIPT = shlex.split(args['scripts']['run'] + " " + cmd_args(args))
                p_status = subprocess.run(combiner_SCRIPT, stdout=log.log, stderr=subprocess.STDOUT)
                _wait = True
            elif combiner.cutoff_idx >= args['f_steps']:
                global_cfg.RUN._wait = True
                p = subprocess.run(shlex.split(scripts['data'])) 
                combiner.update()
                combiner.combine()
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
    return log

def _run(args):
    """launch function to start the simulation and the solver concurrently

    Args:
        args (dict): user + config inputs for simulation and solver
    """
    log = _init(args)
    combiner = Combiner(args, CURRENT_TRAJ_CSV_FILE, NEW_TRAJ_CSV_FILE, lookahead=args['look_ahead'])
    combiner.plan_init(args)
    combiner_runscript = shlex.split(args['scripts']['run'] + " " + cmd_args(args))
    p = subprocess.run(combiner_runscript, stdout=log.log, stderr=subprocess.STDOUT)
    p_copy = subprocess.run(shlex.split(scripts['copy']))
    if p_copy.returncode == 0 and p.returncode == 0:
        print("Launching Simulation")
        combiner_thread = Thread(target=_update, args=(args, log, combiner))
        combiner_thread.start()
        if args.get('record'):
            trajectory_record.record_simulation(args)
        else:
            run.simulation(args)
    else: 
        print("Error in finding or copying global trajectory")
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

def build_args():
    """
    Parse command-line arguments for interacting with the QTOS stack.

    Returns:
        dict: A dictionary containing the parsed arguments.

    This function parses command-line arguments using argparse and returns a dictionary
    containing the parsed arguments. The parsed arguments also builds the enviroment and 
    configures QTOS with the running TOWR solver and gets necessary configuration files.

    Command-line arguments:
        -g, --g: The initial global position of the robot as a list of three floats.
        -s, --s: The initial position of the robot as a list of three floats. Default: [0.0, 0.0, 0.24].
        -s_ang, --s_ang: The initial angular velocity of the robot as a list of three floats.
        -s_vel, --s_vel: The initial velocity of the robot as a list of three floats.
        -e1, --e1: The starting state for the front left leg as a list of three floats.
        -e2, --e2: The starting state for the front right leg as a list of three floats.
        -e3, --e3: The starting state for the back right leg as a list of three floats.
        -e4, --e4: The starting state for the back left leg as a list of three floats.
        -step, --step: The step size to traverse along the global trajectory spline as a float. Default: 1.0.
        -forced_steps, --f_steps: The number of timesteps to force the robot to run and then start stitching the next planned trajectory as an integer. Default: 2500.
        -l, --look: The number of timesteps to lookahead in the planned trajectory as a float. Default: 3750.
        -r, --record: A boolean flag to indicate whether to record joint-angle, joint-velocity, and torque while running the simulator. Default: False.
        -exp, --experiment: The name of the experiment to run in the simulator as a string. Default: "default".
        -p, --mpc_p: A boolean flag indicating a switch for global plan correction. Default: False.
        -t, --towr: A boolean flag indicating the use of the default TOWR local planner. Default: False.
    """
    parser = argparse.ArgumentParser(description="I/O arguments to interact with the QTOS stack")
    parser.add_argument('-g', '--g', nargs=3, type=float, default=None, dest='-g', help="The initial global position of the robot")
    parser.add_argument('-s', '--s', nargs=3, type=float, default=[0.0, 0.0, 0.24], dest='-s', help="The initial position of the robot")
    parser.add_argument('-s_ang', '--s_ang', nargs=3, type=float, dest='-s_ang', help="The initial angular velocity of the robot")
    parser.add_argument('-s_vel', '--s_vel', nargs=3, type=float, dest='-s_vel', help="The initial velocity of the robot")
    parser.add_argument('-e1', '--e1', nargs=3, type=float, dest='-e1', help="The starting state for front left leg")
    parser.add_argument('-e2', '--e2', nargs=3, type=float, dest='-e2' ,help="The starting state for front right leg")
    parser.add_argument('-e3', '--e3', nargs=3, type=float, dest='-e3', help="The starting state for back right leg")
    parser.add_argument('-e4', '--e4', nargs=3, type=float, dest='-e4', help="The starting state for back left leg")
    parser.add_argument('-step', '--step', type=float, default=1.0, dest='step_size', help="The step size to traverse along the global trajectory spline")
    parser.add_argument('-forced_steps', '--f_steps', type=int, default=2500, dest='f_steps', help="The amount of timesteps to force the robot to run and then start stitching the next planned trajectory")
    parser.add_argument('-l', '--look', type=float, default=3750, dest='look_ahead', help='Number of timesteps to lookahead in the planned trajectory (the next starting state for the optimizer)')
    parser.add_argument('-r', '--record', type=bool, default=False, dest='record', help="Record the joint-angle, joint-velocity, and torque while running the simulator")
    parser.add_argument('-exp', '--experiment', type=str, default="default", dest='experiment', help="Selects the experiment to run in the simulator")
    parser.add_argument('-p', '--mpc_p', type=bool, default=False, dest='mpc_p', help="Switch for global plan correction")
    parser.add_argument('-t', '--towr', action="store_true", dest='towr', help="Default TOWR local planner", default=False)
    args = vars(parser.parse_args())
    docker_id = DockerInfo()
    args.update({"scripts": parse_scripts(scripts, docker_id)})
    args['sim_cfg'] = experimentInfo(args['experiment'], args['record'])
    args['-resolution'] = 0.01 if args['sim_cfg'].get('resolution') is None else args['sim_cfg']['resolution'] 
    args.update(builder(sim_cfg=args['sim_cfg']))
    return args

def main():
    args = build_args()
    if args.get('towr'):
        print("Default Test")
        args['-r'] = 30 * args['sim'].num_tiles
        if args['sim_cfg'].get('goal'):
            args['-g'] = args['args']['goal']
        args['-duration'] = 1.0 * args['sim'].num_tiles
        run_default(args)
    elif args.get('-g') is None:
        args['-g'] = [(args['sim'].num_tiles - 1) * 2.0 + 0.5, 0, 0.24]
    _run(args)

if __name__ == "__main__":
    main()