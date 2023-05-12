import csv
import math

import numpy as np
import pandas as pd

import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg
from SOLO12_SIM_CONTROL.utils import zero_filter, percentage_look_ahead

class MPC(object):

    def __init__(self, args, current_traj, new_traj) -> None:
        self.args = args
        self.current_traj = current_traj
        self.current_traj_temp = "/tmp/towr_old_temp.csv"
        self.new_traj = new_traj
        self.cutoff_idx = 0
    
    def combine(self):
        """Combines the old and new csv trajectories together
        """
        _old_traj = open(self.current_traj, "r")
        _new_traj = open(self.new_traj, "r")
        self._truncate_csv(_old_traj, self.args['look_ahead'])
        df_old = pd.read_csv(self.current_traj_temp).to_numpy()
        df_new = pd.read_csv(self.new_traj).to_numpy()
        combined_df = np.concatenate((df_old, df_new), axis=0)
        combined_df = pd.DataFrame(combined_df)
        combined_df.to_csv(self.new_traj, index=False, header=None)

    def plan(self, args):
        """
        Trajcetory plan towards the final goal
        """
        self._step()
        _state_dic = self._state(self.args['look_ahead'])
        self.args['-s'] = _state_dic["CoM"]
        self.args['-s_ang'] = _state_dic['orientation']
        self.args['-e1'] = _state_dic["FL_FOOT"]
        self.args['-e2'] = _state_dic["FR_FOOT"]
        self.args['-e3'] = _state_dic["HL_FOOT"]
        self.args['-e4'] = _state_dic["HR_FOOT"]
        print("ARRGS", self.args)
        return self.args

    def update(self):
        """Updates the state of the MPC loop
        """
        self.cutoff_idx = global_cfg.RUN.step

    def _step(self):
        """Updates step vector toward robot's goal depending on the stepping size
        """
        step_size = self.args['step_size'] #try implementing in config-file
        global_pos = np.array(global_cfg.ROBOT_CFG.linkWorldPosition)
        goal = global_cfg.ROBOT_CFG.robot_goal
        diff_vec = np.clip(goal - global_pos, -step_size, step_size)
        diff_vec[2] = 0.0
        self.args['-g'] = list(global_pos + diff_vec)
        self.args['-g'][2] = 0.24

    def _state(self, p=0.60):
        """Returns the updated robot state back to the optimizer

        Args:
            p (float, optional): Percentage to look-ahead in trajectory. Defaults to 0.60.

        Returns:
            _type_: robot state
        """
        state = {"CoM": None, "orientation": None, "FL_FOOT": None, 
             "FR_FOOT": None, "HL_FOOT": None, "HR_FOOT": None}
        with open(self.current_traj, "r", newline='') as f:
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

    def _truncate_csv(self, file : object, lookahead=0.60):
        """Predicts where robot is in simulation and truncates old trajectory points

        Args:
            file (_type_): csv trajectory file
            lookahead (float): upper bound cutoff for old trajectory
        """
        df = pd.read_csv(file)
        num_row = len(df)
        start_idx = self.cutoff_idx
        end_idx = math.ceil(num_row * lookahead)
        _new_csv = df.iloc[start_idx:end_idx]
        _new_csv.to_csv(self.current_traj_temp, index=False)

