import numpy as np
import pybullet
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


    def tracking_error(self):
        for opt, sim in zip(self.traj['realized'], self.traj['sim']):
            # breakpoint()
            pass
            # metrics[]
            # self.FL_FOOT['realized'] = 

    
    def plot(self):
        pass

    def get_sim_cmd(self):
        vec = self.robot.traj_vec
        return vec_to_cmd(vec)
        


if __name__ == "__main__":
    pass