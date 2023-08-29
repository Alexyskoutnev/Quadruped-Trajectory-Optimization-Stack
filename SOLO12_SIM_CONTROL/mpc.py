import csv
import math
import threading
import time

import numpy as np
import pandas as pd
import pybullet as p

import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg
from SOLO12_SIM_CONTROL.utils import *
from SOLO12_SIM_CONTROL.planner import Global_Planner



class MPC_THREAD(threading.Thread):
    def __init__(self, obj) -> None:
        threading.Thread.__init__(self)
        self.obj = obj

    def run(self):
        while True:
            self.obj.update()
            time.sleep(0.01)

class MPC(object):

    def __init__(self, args, current_traj, new_traj, lookahead=None, hz=1000) -> None:
        self.args = args
        self.current_traj = current_traj
        self.current_traj_temp = "/tmp/towr_old_temp.csv"
        self.new_traj = new_traj
        self.cutoff_idx = 0
        self.last_timestep = 0.0
        self.lookahead = lookahead if lookahead else 7500
        self.next_traj_step = 0
        self.hz = hz
        self.decimal_precision = math.log10(hz)
        self.global_planner = Global_Planner(args, self.lookahead, hz)
        self.global_solver = self.global_planner.path_solver
        self.spine_x = self.global_solver.spine_x_track
        self.spine_y = self.global_solver.spine_y_track
        self.traj_plan = txt_2_np_reader(self.current_traj)
        self.robot = args['robot']
        self.set_correct_flag = False
        self.set_spine_flag = True

    @property
    def global_plan_state(self):
        x_ref = self.spine_x(self.last_timestep)
        y_ref = self.spine_y(self.last_timestep)
        return [x_ref.item(), y_ref.item()]

    @property
    def plan_state(self):
        state = np.squeeze(self.traj_plan[np.where(self.traj_plan[:, 0] == round(self.last_timestep, int(self.decimal_precision)))[0]])
        return state

    @property
    def robot_state(self):
        state = global_cfg.ROBOT_CFG.state
        return state

    def combine(self):
        """Combines the old and new csv trajectories together
        """
        time_combine = time.process_time()
        _old_traj = open(self.current_traj, "r")
        _new_traj = open(self.new_traj, "r")
        df_old = self._truncate_csv(_old_traj).to_numpy()
        df_new = pd.read_csv(self.new_traj).to_numpy()
        combined_df = np.concatenate((df_old, df_new), axis=0)
        self.traj_plan = combined_df
        combined_df = pd.DataFrame(combined_df)
        combined_df.to_csv(self.new_traj, index=False, header=None)    

    def plan_init(self, args):

        def start_config(args):
            args['-s'] = [0, 0, 0.24]
            args['-e1'] = [0.20590930477664196, 0.14927536747689948, 0.0]
            args['-e2'] = [0.2059042161427424, -0.14926921805769638, 0.0]
            args['-e3'] = [-0.20589422629511542, 0.14933201572367907, 0.0]
            args['-e4'] = [-0.2348440184502048, -0.17033609357109808, 0.0]
            args['-s_ang'] = [0, 0, 0]
        start_config(args)
        step_size = args['step_size'] #try implementing in config-file
        if self.set_spine_flag:
            print("spine")
            global_pos = np.array(global_cfg.ROBOT_CFG.linkWorldPosition)
            goal = self.spine_step(args['-s'], self.last_timestep)
            diff_vec = np.clip(goal - global_pos, -step_size, step_size)
            diff_vec[2] = 0.0
            args['-g'] = list(global_pos + diff_vec)
            args['-g'][2] = 0.24
        else:
            global_pos = np.array(global_cfg.ROBOT_CFG.linkWorldPosition)
            goal = global_cfg.ROBOT_CFG.robot_goal
            diff_vec = np.clip(goal - global_pos, -step_size, step_size)
            diff_vec[2] = 0.0
            args['-g'] = list(global_pos + diff_vec)
            args['-g'][2] = 0.24
        return args


    def plan(self, args):
        """
        Trajectory plan towards the final goal

        Args: 
            args (dic) : updates the input arguments to Towr script
        Returns:
            self.args (dic) : input argument to Towr script
        """

        if self.set_spine_flag or self.global_planner.P_correction and not self.global_planner.empty():
            _state_dic = self._state()
            start_pos, end_pos = self.global_planner.pop()
            self.args['-s'] = _state_dic["CoM"]
            self.args['-s_ang'] = _state_dic['orientation']
            self.args['-e1'] = _state_dic["FL_FOOT"]
            self.args['-e2'] = _state_dic["FR_FOOT"]
            self.args['-e3'] = _state_dic["HL_FOOT"]
            self.args['-e4'] = _state_dic["HR_FOOT"]
            self.args['-t'] = global_cfg.ROBOT_CFG.runtime + (self.lookahead / self.hz)
            self.args['-g'] = end_pos
        else:
            _state_dic = self._state()
            self.args['-s'] = _state_dic["CoM"]
            self.args['-s_ang'] = _state_dic['orientation']
            self.args['-e1'] = _state_dic["FL_FOOT"]
            self.args['-e2'] = _state_dic["FR_FOOT"]
            self.args['-e3'] = _state_dic["HL_FOOT"]
            self.args['-e4'] = _state_dic["HR_FOOT"]
            self.args['-t'] = global_cfg.ROBOT_CFG.runtime + (self.lookahead / self.hz)
            self._step(_state_dic["CoM"])
        print("ARRGS", self.args)
        # breakpoint()
        return self.args

    def spine_step(self, CoM, timestep, total_traj_time=5.0):
        step_size = self.args['step_size']
        timestep_future = timestep + total_traj_time
        goal = np.array([self.spine_x(timestep_future), self.spine_y(timestep_future), 0.24])
        diff_vec = np.clip(goal - CoM, -step_size, step_size)
        return CoM + diff_vec

    def update(self):
        """
        Updates the state of the MPC loop
        """
        self.cutoff_idx = int(global_cfg.RUN.step)
        self.last_timestep = round(global_cfg.ROBOT_CFG.runtime, int(self.decimal_precision))
        self.goal_diff = np.linalg.norm(np.array(self.args['-s'])[0:2] - np.array(self.args['-g'])[0:2])
        goal_step_vec = np.array(self.args['-g']) - np.array(self.args['-s'])
        #Updating the global planner
        self.global_planner.update(self.last_timestep, self.global_plan_state, self.robot_state[1:3], goal_step_vec)
        if self.global_planner.max_t < self.last_timestep + 5.0:
            print("STOPING MPC CONTROLLER")
            global_cfg.RUN._done = True


    def update_timestep(self):
        """
        Update the timestep by one time unit
        """
        self.last_timestep += 1/self.hz

    def _step(self, CoM):
        """
        Updates step vector toward robot's goal depending on the stepping size
        """
        step_size = self.args['step_size']
        global_pos = np.array(CoM)
        goal = global_cfg.ROBOT_CFG.robot_goal
        diff_vec = np.clip(goal - global_pos, -step_size, step_size)
        diff_vec[2] = 0.0
        self.args['-g'] = list(global_pos + diff_vec)
        self.args['-g'][2] = 0.24

    def _state(self):
        """Returns the updated robot state back to the optimizer

        Args:
            lookahead_steps (int): number of steps to look ahead for the 
                                   next trajectory start
        Returns:
            state (dic): robot state dic
        """
        state = {"CoM": None, "orientation": None, "FL_FOOT": None, 
             "FR_FOOT": None, "HL_FOOT": None, "HR_FOOT": None}
        with open(self.current_traj, "r", newline='') as f:
            print(f"=============NEXT SEARCH QUERY [{self.last_timestep}]=============")
            print(f"=============OLD TRAJ START INDEX QUERY [{self.cutoff_idx}]=============")
            print(f"=============OLD TRAJ END INDEX QUERY [{self.next_traj_step}]=============")
            reader, step = look_ahead(f, self.last_timestep, self.lookahead)
            row = next(reader)[1:]
            state["CoM"] = [float(_) for _ in row[0:3]]
            state["orientation"] = [float(_) for _ in row[3:6]]
            state["FL_FOOT"] = [float(_) for _ in row[6:9]]
            state["FR_FOOT"] = [float(_) for _ in row[9:12]]
            state["HL_FOOT"] = [float(_) for _ in row[12:15]]
            state["HR_FOOT"] = [float(_) for _ in row[15:18]]
        state = {key: zero_filter(value) for key, value in state.items()}
        self.next_traj_step = step + self.lookahead - 1
        return state

    def _truncate_csv(self, file : object):
        """Predicts where robot is in simulation and truncates old trajectory points

        Args:
            file (object): csv trajectory file
            lookahead (float): upper bound cutoff for old trajectory
        """
        df = pd.read_csv(file)
        if self.cutoff_idx <= 0:
            start_idx = self.cutoff_idx = 0
        else:
            start_idx = self.cutoff_idx - 1
        end_idx = self.next_traj_step
        _new_csv = df.iloc[start_idx:end_idx]
        return _new_csv

