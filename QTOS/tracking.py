import os
import datetime
import numpy as np
import pybullet
import matplotlib.pyplot as plt
from collections import namedtuple

from QTOS.utils import vec_to_cmd, vec_to_cmd_pose
from QTOS.logger import Logger
from QTOS.config.global_cfg import ROBOT_CFG

SAVE_FILE = "./data/tracking/ref_sim_track_"
SAVE_FILE_COM = "./data/tracking/CoM_track_"
SAVE_FILE_ERROR_TIME = "./data/tracking/tracking_error_vs_time"
SAVE_FILE_ERROR = "./data/tracking/ref_sim_error_"
SAVE_DIR = "./data/tracking/"
NUM_TRAJS_TO_SAVE = 2

def date_salt(f_type=".png"):
    current_datetime = datetime.datetime.now()
    timestamp = current_datetime.strftime("%Y%m%d_%H%M%S")
    return timestamp + f_type

class COMMAND:
    """
    Represents a robot command at a specific timestep.
    """

    def __init__(self, timestep, cmd) -> None:
        """
        Initialize a COMMAND object.

        Args:
            timestep (float): The timestep when the command was issued.
            cmd (dict): The robot command data.
        """
        self.timestep = timestep
        self.cmd = cmd
        self.CoM = {"CoM_pos": tuple(self.cmd['COM'][0:3]), "CoM_ang": tuple(self.cmd['COM'][3:6])}
        self.position = {"FL_FOOT": tuple(self.cmd['FL_FOOT']['P']), "FR_FOOT": tuple(self.cmd['FR_FOOT']['P']),
                         "HL_FOOT": tuple(self.cmd['HL_FOOT']['P']), "HR_FOOT": tuple(self.cmd['HR_FOOT']['P'])}
        self.velocity = {"FL_FOOT": tuple(self.cmd['FL_FOOT']['D']), "FR_FOOT": tuple(self.cmd['FR_FOOT']['D']),
                         "HL_FOOT": tuple(self.cmd['HL_FOOT']['D']), "HR_FOOT": tuple(self.cmd['HR_FOOT']['D'])}

class Tracking:
    """
    A class for tracking and visualizing robot trajectory data.
    """

    def __init__(self, robot, num_traj, cfg) -> None:
        """
        Initialize a Tracking object.

        Args:
            robot: The robot object.
            num_traj (int): The number of trajectories to track.
            cfg (dict): Configuration data.
        """
        self.logger = Logger("./logs", "experiment_data")
        self.robot = robot
        self.traj = {"reference" : [], "sim" : []}
        self.idx = 0
        self.num_traj = num_traj
        self.metrics = {}
        self.FL_FOOT = {"reference": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.FR_FOOT = {"reference": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.HL_FOOT = {"reference": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.HR_FOOT = {"reference": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.CoM_Pos = {"reference": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.CoM_Ang = {"reference": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.end_time = 5.0
        self.size = num_traj
        self.timeseries = np.linspace(0, self.end_time, num_traj)
        self.total_error_feet_error = 0.0
        self.total_error_com_error = 0.0
        self.track_rate = cfg['track_rate']
        self.track_flag = cfg['track']
        self.date_time_salt = date_salt()
        self.delay_tracking_itr = 500
        self.delay_flag = False
        self.delay_num = 0
        self.error_v_time = list()
        self.distance_v_time = list()
        if os.path.exists(SAVE_DIR) and os.path.isdir(SAVE_DIR):
            if len(os.listdir(SAVE_DIR)) > (NUM_TRAJS_TO_SAVE * 3):
                for file in os.listdir(SAVE_DIR):
                    file_path = os.path.join(SAVE_DIR, file)
                    os.remove(file_path)
    
    def update(self, reference_cmd, timestep):
        """
        Update tracking data with a new robot command.

        Args:
            reference_cmd (dict): The reference robot command.
            timestep (float): The timestep when the command was issued.
        """
        
        self.delay_num += 1
        if self.delay_num < self.delay_tracking_itr and not self.delay_flag:
            pass
        elif self.delay_num> self.delay_tracking_itr and not self.delay_flag:
            self.delay_flag = True
        else:
            sim_cmd = self.get_sim_cmd()
            self.traj['reference'].append(COMMAND(timestep, reference_cmd))
            self.traj['sim'].append(COMMAND(timestep, sim_cmd))
            self._update()
            self.idx += 1
            self.size = self.idx + 1
            self.end_time = timestep
            if self.idx % self.track_rate == 0:
                if self.track_flag:
                    self.plot()

    @property
    def distance(self):
        """
        Property method to retrieve the robot's distance information.

        Returns:
            np.ndarray: An array representing the robot's distance information.
        """
        return ROBOT_CFG.global_COM_xyz
        
    def _update(self):
        """
        Update method to compute errors and update data dictionaries with simulation results.
        Calculates and stores errors between reference and simulated values for various robot parts.
        Also, logs important information.

        Returns:
            None
        """
        self.timeseries = np.linspace(0, self.end_time, self.size)
        for NAME in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT', 'CoM_pos'):
            if NAME in ('CoM_pos'):
                r_com_x, r_com_y, r_com_z = self.traj['reference'][self.idx].CoM[NAME]
                s_com_x, s_com_y, s_com_z = self.traj['sim'][self.idx].CoM[NAME]
                error = np.linalg.norm(np.array(self.traj['reference'][self.idx].CoM[NAME]) - np.array(self.traj['sim'][self.idx].CoM[NAME]))
                self.total_error_com_error += error
                self.CoM_Pos['reference'].append([r_com_x, r_com_y, r_com_z])
                self.CoM_Pos['sim'].append([s_com_x, s_com_y, s_com_z])
                self.CoM_Pos['r_x'].append(r_com_x)
                self.CoM_Pos['r_y'].append(r_com_y)
                self.CoM_Pos['r_z'].append(r_com_z)
                self.CoM_Pos['s_x'].append(s_com_x)
                self.CoM_Pos['s_y'].append(s_com_y)
                self.CoM_Pos['s_z'].append(s_com_z)
                self.CoM_Pos['error'].append(error)
            elif NAME in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'):
                r_x, r_y, r_z = self.traj['reference'][self.idx].position[NAME]
                s_x, s_y, s_z = self.traj['sim'][self.idx].position[NAME]
                error = np.linalg.norm(np.array(self.traj['reference'][self.idx].position[NAME]) - np.array(self.traj['sim'][self.idx].position[NAME]))
                self.total_error_feet_error += error
                if NAME == 'FL_FOOT':
                    self.FL_FOOT['reference'].append([r_x, r_y, r_z])
                    self.FL_FOOT['sim'].append([s_x, s_y, s_z])
                    self.FL_FOOT['r_x'].append(r_x)
                    self.FL_FOOT['r_y'].append(r_y)
                    self.FL_FOOT['r_z'].append(r_z)
                    self.FL_FOOT['s_x'].append(s_x)
                    self.FL_FOOT['s_y'].append(s_y)
                    self.FL_FOOT['s_z'].append(s_z)
                    self.FL_FOOT['error'].append(error)
                elif NAME == 'FR_FOOT':
                    self.FR_FOOT['reference'].append([r_x, r_y, r_z])
                    self.FR_FOOT['sim'].append([s_x, s_y, s_z])
                    self.FR_FOOT['r_x'].append(r_x)
                    self.FR_FOOT['r_y'].append(r_y)
                    self.FR_FOOT['r_z'].append(r_z)
                    self.FR_FOOT['s_x'].append(s_x)
                    self.FR_FOOT['s_y'].append(s_y)
                    self.FR_FOOT['s_z'].append(s_z)
                    self.FR_FOOT['error'].append(error)
                elif NAME == 'HL_FOOT':
                    self.HL_FOOT['reference'].append([r_x, r_y, r_z])
                    self.HL_FOOT['sim'].append([s_x, s_y, s_z])
                    self.HL_FOOT['r_x'].append(r_x)
                    self.HL_FOOT['r_y'].append(r_y)
                    self.HL_FOOT['r_z'].append(r_z)
                    self.HL_FOOT['s_x'].append(s_x)
                    self.HL_FOOT['s_y'].append(s_y)
                    self.HL_FOOT['s_z'].append(s_z)
                    self.HL_FOOT['error'].append(error)
                elif NAME == 'HR_FOOT':
                    self.HR_FOOT['reference'].append([r_x, r_y, r_z])
                    self.HR_FOOT['sim'].append([s_x, s_y, s_z])
                    self.HR_FOOT['r_x'].append(r_x)
                    self.HR_FOOT['r_y'].append(r_y)
                    self.HR_FOOT['r_z'].append(r_z)
                    self.HR_FOOT['s_x'].append(s_x)
                    self.HR_FOOT['s_y'].append(s_y)
                    self.HR_FOOT['s_z'].append(s_z)
                    self.HR_FOOT['error'].append(error)
        
        self.logger.write(f"[{self.idx}] Total COM ERROR : {self.total_error_com_error}\n")
        self.logger.write(f"[{self.idx}] Average COM ERROR PER Second : {(self.total_error_com_error / (self.idx + 1)) * 1000}\n")
        self.logger.write(f"[{self.idx}] Distance : {self.distance}\n")
        self.logger.write(f"[{self.idx}] x-distance : {self.distance[0]}\n")

    def plot_reference_vs_sim(self, plot_graph=False):
        """
        Generate a multi-panel plot comparing reference and simulated positions of robot parts.
        
        Args:
            plot_graph (bool, optional): Whether to display the generated plot. Defaults to False.

        Returns:
            None
        """
        plt.figure(figsize=(10, 6))
        plt.tight_layout()
        plt.subplot(4, 3, 1)
        plt.plot(self.timeseries[0:len(self.FL_FOOT['r_x'])], self.FL_FOOT['r_x'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FL_FOOT['s_x'])], self.FL_FOOT['s_x'], label='sim', color='blue')
        plt.grid(True)
        plt.title('FL X Position')
        plt.subplot(4, 3, 2) 
        plt.plot(self.timeseries[0:len(self.FL_FOOT['r_y'])], self.FL_FOOT['r_y'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FL_FOOT['s_y'])], self.FL_FOOT['s_y'], label='sim', color='blue')
        plt.grid(True)
        plt.title('FL Y Position')
        plt.subplot(4, 3, 3)
        plt.plot(self.timeseries[0:len(self.FL_FOOT['r_z'])], self.FL_FOOT['r_z'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FL_FOOT['s_z'])], self.FL_FOOT['s_z'], label='sim', color='blue')
        plt.ylim(-0.1,0.3)
        plt.grid(True)
        plt.title('FL Z Position')

        plt.subplot(4, 3, 4)
        plt.plot(self.timeseries[0:len(self.FR_FOOT['r_x'])], self.FR_FOOT['r_x'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FR_FOOT['s_x'])], self.FR_FOOT['s_x'], label='sim', color='blue')
        plt.grid(True)
        plt.title('FR X Position')
        plt.subplot(4, 3, 5)
        plt.plot(self.timeseries[0:len(self.FR_FOOT['r_y'])], self.FR_FOOT['r_y'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FR_FOOT['s_y'])], self.FR_FOOT['s_y'], label='sim', color='blue')
        plt.grid(True)
        plt.title('FR Y Position')
        plt.subplot(4, 3, 6)
        plt.plot(self.timeseries[0:len(self.FR_FOOT['r_z'])], self.FR_FOOT['r_z'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FR_FOOT['s_z'])], self.FR_FOOT['s_z'], label='sim', color='blue')
        plt.ylim(-0.1,0.3)
        plt.grid(True)
        plt.title('FR Z Position')

        plt.subplot(4, 3, 7)
        plt.plot(self.timeseries[0:len(self.HL_FOOT['r_x'])], self.HL_FOOT['r_x'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HL_FOOT['s_x'])], self.HL_FOOT['s_x'], label='sim', color='blue')
        plt.grid(True)
        plt.title('HL X Position')
        plt.subplot(4, 3, 8)
        plt.plot(self.timeseries[0:len(self.HL_FOOT['r_y'])], self.HL_FOOT['r_y'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HL_FOOT['s_y'])], self.HL_FOOT['s_y'], label='sim', color='blue')
        plt.grid(True)
        plt.title('HL Y Position')
        plt.subplot(4, 3, 9)
        plt.plot(self.timeseries[0:len(self.HL_FOOT['r_z'])], self.HL_FOOT['r_z'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HL_FOOT['s_z'])], self.HL_FOOT['s_z'], label='sim', color='blue')
        plt.ylim(-0.1,0.3)
        plt.grid(True)
        plt.title('HL Z Position')

        plt.subplot(4, 3, 10)
        plt.plot(self.timeseries[0:len(self.HR_FOOT['r_x'])], self.HR_FOOT['r_x'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HR_FOOT['s_x'])], self.HR_FOOT['s_x'], label='sim', color='blue')
        plt.grid(True)
        plt.title('HR X Position')
        plt.subplot(4, 3, 11)
        plt.plot(self.timeseries[0:len(self.HR_FOOT['r_y'])], self.HR_FOOT['r_y'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HR_FOOT['s_y'])], self.HR_FOOT['s_y'], label='sim', color='blue')
        plt.grid(True)
        plt.title('HR Y Position')
        plt.subplot(4, 3, 12)
        plt.plot(self.timeseries[0:len(self.HR_FOOT['r_z'])], self.HR_FOOT['r_z'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HR_FOOT['s_z'])], self.HR_FOOT['s_z'], label='sim', color='blue')
        plt.ylim(-0.1,0.3)
        plt.grid(True)
        plt.title('HR Z Position')
        plt.legend(loc='lower right')
        plt.savefig(SAVE_FILE + self.date_time_salt)
        plt.close()
        
        if plot_graph:
            plt.show()

    def plot_error(self, plot_graph=False):
        """
        Generate a multi-panel plot displaying errors related to different robot parts.
        
        Args:
            plot_graph (bool, optional): Whether to display the generated plot. Defaults to False.

        Returns:
            None
        """
        plt.figure(figsize=(10, 6))
        plt.tight_layout()
        plt.subplot(4, 1, 1)
        plt.plot(self.timeseries[0:len(self.FL_FOOT['error'])], self.FL_FOOT['error'], label='error', color='red')
        plt.grid(True)
        plt.legend()

        plt.subplot(4, 1, 2)
        plt.plot(self.timeseries[0:len(self.FR_FOOT['error'])], self.FR_FOOT['error'], label='error', color='red')
        plt.grid(True)
        plt.legend()
        plt.title('FR X Error')

        plt.subplot(4, 1, 3)
        plt.plot(self.timeseries[0:len(self.HL_FOOT['error'])], self.HL_FOOT['error'], label='error', color='red')
        plt.grid(True)
        plt.legend()
        plt.title('HL X Error')

        plt.subplot(4, 1, 4)
        plt.plot(self.timeseries[0:len(self.HR_FOOT['error'])], self.HR_FOOT['error'], label='error', color='red')
        plt.grid(True)
        plt.legend()
        plt.title('HR X Error')
        plt.savefig(SAVE_FILE_ERROR + self.date_time_salt)
        plt.close()

        if plot_graph:
            plt.show()

    def plot_CoM(self, plot_graph=False):
        """
        Generate a multi-panel plot for displaying the center of mass (CoM) position in X, Y, and Z directions.
        
        Args:
            plot_graph (bool, optional): Whether to display the generated plot. Defaults to False.

        Returns:
            None
        """
        plt.figure(figsize=(10, 6))
        plt.subplot(1, 3, 1)
        plt.plot(self.timeseries[0:len(self.CoM_Pos['r_x'])], self.CoM_Pos['r_x'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.CoM_Pos['s_x'])], self.CoM_Pos['s_x'], label='realized', color='blue')
        plt.grid(True)
        plt.xlabel('Timestep')
        plt.ylabel('Position [m]')
        plt.title('Center of Mass [X]')
        plt.subplot(1, 3, 2)
        plt.plot(self.timeseries[0:len(self.CoM_Pos['r_y'])], self.CoM_Pos['r_y'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.CoM_Pos['s_y'])], self.CoM_Pos['s_y'], label='realized', color='blue')
        plt.grid(True)
        plt.ylim(-0.5,0.5)
        plt.xlabel('Timestep')
        plt.title('Center of Mass [Y]')
        plt.subplot(1, 3, 3)
        plt.plot(self.timeseries[0:len(self.CoM_Pos['r_z'])], self.CoM_Pos['r_z'], label='reference', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.CoM_Pos['s_z'])], self.CoM_Pos['s_z'], label='realized', color='blue')
        plt.ylim(-0.00,0.5)
        plt.xlabel('Timestep')
        plt.grid(True)
       
        plt.title('Center of Mass [Z]')
        plt.legend(loc='lower right')
        plt.savefig(SAVE_FILE_COM + self.date_time_salt)
        plt.close()
        if plot_graph:
            plt.show()

    def plot_error_over_time(self, plot_graph=False):
        plt.figure(figsize=(10, 6))
        plt.tight_layout()
        plt.plot(self.distance_v_time, self.error_v_time)
        plt.xlabel('Distance [m]')
        plt.savefig(SAVE_FILE_ERROR_TIME + self.date_time_salt)
        plt.close()


    def plot(self, plot_graph=False):
        """
        Generate and save various plots to visualize simulation results.
        This method calls plot_CoM, plot_reference_vs_sim, and plot_error methods.
        It also prints out total feet and CoM errors.
        
        Args:
            plot_graph (bool, optional): Whether to display the generated plots. Defaults to False.

        Returns:
            None
        """
        if self.delay_flag:
            self.plot_CoM(plot_graph)
            self.plot_reference_vs_sim(plot_graph)
            self.plot_error(plot_graph)
            self.plot_error_over_time(plot_graph)
            self.distance_v_time.append(self.distance[0])
            self.error_v_time.append(self.total_error_com_error / self.idx * 1000) 

    def get_sim_cmd(self):
        """
        Retrieve a simulation command based on robot properties.

        Returns:
            cmd_pose: A command for the simulation based on robot properties.
        """
        vec = self.robot.traj_vec
        return vec_to_cmd_pose(vec)