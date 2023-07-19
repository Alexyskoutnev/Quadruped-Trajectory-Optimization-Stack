import numpy as np
import pybullet
import matplotlib.pyplot as plt
from collections import namedtuple


from SOLO12_SIM_CONTROL.utils import vec_to_cmd

class COMMAND:
    def __init__(self, timestep, cmd) -> None:
        self.timestep = timestep
        self.cmd = cmd
        self.position = {"FL_FOOT": tuple(self.cmd['FL_FOOT']['P']), "FR_FOOT": tuple(self.cmd['FR_FOOT']['P']), "HL_FOOT": tuple(self.cmd['HL_FOOT']['P']), "HR_FOOT": tuple(self.cmd['HR_FOOT']['P'])}
        self.velocity = {"FL_FOOT": tuple(self.cmd['FL_FOOT']['D']), "FR_FOOT": tuple(self.cmd['FR_FOOT']['D']), "HL_FOOT": tuple(self.cmd['HL_FOOT']['D']), "HR_FOOT": tuple(self.cmd['HR_FOOT']['D'])}

class Tracking:

    def __init__(self, robot, num_traj) -> None:
        self.robot = robot
        self.traj = {"realized" : [], "sim" : []}
        self.idx = 0
        self.num_traj = num_traj
        self.metrics = {}
        self.FL_FOOT = {"realized": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.FR_FOOT = {"realized": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.HL_FOOT = {"realized": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.HR_FOOT = {"realized": [], "sim": [], "error": [], "r_x" : [], "r_y" : [], "r_z" : [], "s_x": [], "s_y" : [], "s_z" : []}
        self.timeseries = np.linspace(0, 10, num_traj)
    
    def update(self, realized_cmd, timestep):
        sim_cmd = self.get_sim_cmd()
        self.traj['realized'].append(COMMAND(timestep, realized_cmd))
        self.traj['sim'].append(COMMAND(timestep, sim_cmd))
        self._update()
        self.idx += 1
        
    def _update(self):
        for EE_NAME in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'):
            r_x, r_y, r_z = self.traj['realized'][self.idx].position[EE_NAME]
            s_x, s_y, s_z = self.traj['sim'][self.idx].position[EE_NAME]
            error = np.linalg.norm(np.array(self.traj['realized'][self.idx].position[EE_NAME]) - np.array(self.traj['sim'][self.idx].position[EE_NAME]))
            if EE_NAME == 'FL_FOOT':
                self.FL_FOOT['realized'].append([r_x, r_y, r_z])
                self.FL_FOOT['sim'].append([s_x, s_y, s_z])
                self.FL_FOOT['r_x'].append(r_x)
                self.FL_FOOT['r_y'].append(r_y)
                self.FL_FOOT['r_z'].append(r_z)
                self.FL_FOOT['s_x'].append(s_x)
                self.FL_FOOT['s_y'].append(s_y)
                self.FL_FOOT['s_z'].append(s_z)
                self.FL_FOOT['error'].append(error)
            elif EE_NAME == 'FR_FOOT':
                self.FR_FOOT['realized'].append([r_x, r_y, r_z])
                self.FR_FOOT['sim'].append([s_x, s_y, s_z])
                self.FR_FOOT['r_x'].append(r_x)
                self.FR_FOOT['r_y'].append(r_y)
                self.FR_FOOT['r_z'].append(r_z)
                self.FR_FOOT['s_x'].append(s_x)
                self.FR_FOOT['s_y'].append(s_y)
                self.FR_FOOT['s_z'].append(s_z)
                self.FR_FOOT['error'].append(error)
            elif EE_NAME == 'HL_FOOT':
                self.HL_FOOT['realized'].append([r_x, r_y, r_z])
                self.HL_FOOT['sim'].append([s_x, s_y, s_z])
                self.HL_FOOT['r_x'].append(r_x)
                self.HL_FOOT['r_y'].append(r_y)
                self.HL_FOOT['r_z'].append(r_z)
                self.HL_FOOT['s_x'].append(s_x)
                self.HL_FOOT['s_y'].append(s_y)
                self.HL_FOOT['s_z'].append(s_z)
                self.HL_FOOT['error'].append(error)
            elif EE_NAME == 'HR_FOOT':
                self.HR_FOOT['realized'].append([r_x, r_y, r_z])
                self.HR_FOOT['sim'].append([s_x, s_y, s_z])
                self.HR_FOOT['r_x'].append(r_x)
                self.HR_FOOT['r_y'].append(r_y)
                self.HR_FOOT['r_z'].append(r_z)
                self.HR_FOOT['s_x'].append(s_x)
                self.HR_FOOT['s_y'].append(s_y)
                self.HR_FOOT['s_z'].append(s_z)
                self.HR_FOOT['error'].append(error)

    def plot_realized_vs_sim(self):
        plt.figure(figsize=(10, 6))
        plt.tight_layout()
        plt.subplot(4, 3, 1)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.FL_FOOT['r_x'])], self.FL_FOOT['r_x'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FL_FOOT['s_x'])], self.FL_FOOT['s_x'], label='sim', color='blue')
        # plt.plot(self.timeseries[0:len(self.FL_FOOT['error'])], self.FL_FOOT['error'], label='error', color='red')
        plt.grid(True)
        plt.legend()
        plt.title('FL X Position')
        plt.subplot(4, 3, 2)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.FL_FOOT['r_y'])], self.FL_FOOT['r_y'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FL_FOOT['s_y'])], self.FL_FOOT['s_y'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('FL Y Position')
        plt.subplot(4, 3, 3)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.FL_FOOT['r_z'])], self.FL_FOOT['r_z'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FL_FOOT['s_z'])], self.FL_FOOT['s_z'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('FL Z Position')

        plt.subplot(4, 3, 4)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.FR_FOOT['r_x'])], self.FR_FOOT['r_x'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FR_FOOT['s_x'])], self.FR_FOOT['s_x'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('FR X Position')
        plt.subplot(4, 3, 5)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.FR_FOOT['r_y'])], self.FR_FOOT['r_y'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FR_FOOT['s_y'])], self.FR_FOOT['s_y'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('FR Y Position')
        plt.subplot(4, 3, 6)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.FR_FOOT['r_z'])], self.FR_FOOT['r_z'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.FR_FOOT['s_z'])], self.FR_FOOT['s_z'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('FR Z Position')

        plt.subplot(4, 3, 7)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.HL_FOOT['r_x'])], self.HL_FOOT['r_x'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HL_FOOT['s_x'])], self.HL_FOOT['s_x'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('HL X Position')
        plt.subplot(4, 3, 8)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.HL_FOOT['r_y'])], self.HL_FOOT['r_y'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HL_FOOT['s_y'])], self.HL_FOOT['s_y'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('HL Y Position')
        plt.subplot(4, 3, 9)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.HL_FOOT['r_z'])], self.HL_FOOT['r_z'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HL_FOOT['s_z'])], self.HL_FOOT['s_z'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('HL Z Position')


        plt.subplot(4, 3, 10)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.HR_FOOT['r_x'])], self.HR_FOOT['r_x'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HR_FOOT['s_x'])], self.HR_FOOT['s_x'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('HR X Position')
        plt.subplot(4, 3, 11)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.HR_FOOT['r_y'])], self.HR_FOOT['r_y'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HR_FOOT['s_y'])], self.HR_FOOT['s_y'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('HR Y Position')
        plt.subplot(4, 3, 12)  # 2 rows, 2 columns, plot number 1
        plt.plot(self.timeseries[0:len(self.HR_FOOT['r_z'])], self.HR_FOOT['r_z'], label='realized', color='green', linestyle="dashed")
        plt.plot(self.timeseries[0:len(self.HR_FOOT['s_z'])], self.HR_FOOT['s_z'], label='sim', color='blue')
        plt.grid(True)
        plt.legend()
        plt.title('HR Z Position')
        plt.show()


    def get_sim_cmd(self):
        vec = self.robot.traj_vec
        return vec_to_cmd(vec)
        


if __name__ == "__main__":
    pass