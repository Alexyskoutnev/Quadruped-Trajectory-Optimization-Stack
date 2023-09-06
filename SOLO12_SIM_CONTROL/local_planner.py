import csv

import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import ListedColormap

from SOLO12_SIM_CONTROL.utils import is_numeric

READ_FILE = "./data/traj/towr.csv"
HEIGHT_MAP = "./data/heightfields/EXAMPLE_HEIGHT_MAP.txt"

def heighmap_2_np_reader(file, delimiter=','):
    data = []
    with open(file, 'r') as f:
        reader = f.readlines()
        for row in reader:
            _row = row.strip().split(delimiter)
            _row = [float(x) for x in _row if is_numeric(x)]
            data.append(_row)
    return np.transpose(np.array(data))

def plot(trajs, map):
    t = np.linspace(0, 10, trajs[0].shape[0])  # Time points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    colors = ("red", "green", "orange", "purple")
    for traj, color in zip(trajs, colors):
        x = traj[:,0]
        y = traj[:,1]
        z = traj[:,2]
        ax.plot(x, y, z, label='Trajectory', color=color, linewidth=2.0)
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('3D Trajectory Plot')
    ax.legend()

    #heightmap
    _x_grid = np.linspace(0, map.shape[1] / 100, map.shape[1])
    _y_grid = np.linspace(0, map.shape[0] / 100, map.shape[0])
    x_grid, y_grid = np.meshgrid(_x_grid - 1, _y_grid - 1)
    z_grid = map
    custom_cmap = ListedColormap(['grey'])
    surf = ax.plot_surface(x_grid, y_grid, z_grid, cmap=custom_cmap, alpha=0.9)
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10)
    ax.grid(False)
    plt.show()



class Local_Planner(object):

    def __init__(self, robot, traj, map) -> None:
        self.traj = traj
        self.robot = robot
        self.map = map
        self.height_set = self.get_height_set(self.map)
        self.EE_traj = {"FL_FOOT": {"traj": None, "contact_idx": None}, "FR_FOOT": None, "HL_FOOT": None, "HR_FOOT": None}

    def plan(self, traj):
        self._filter(traj)
        pass

    def _process(self, traj):
        pass

    def _filter(self, traj):
        self.EE_traj["FL_FOOT"] = traj[:,7:10]
        self.EE_traj["FR_FOOT"] = traj[:,10:13]
        self.EE_traj["HL_FOOT"] = traj[:,13:16]
        self.EE_traj["HR_FOOT"] = traj[:,16:19]
        self.orignal_traj = (self.EE_traj["FL_FOOT"], self.EE_traj["FR_FOOT"], self.EE_traj["HL_FOOT"], self.EE_traj["HR_FOOT"])

    def cost_function(traj):
        pass

    def constraints(traj):
        pass

    def get_height_set(self, map):
        height_values = np.unique(map)
        return set(height_values)
        
    def check_legs_contact(self, state, tol=9):
        for leg in state:
            if leg in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'):
                if round(state[leg][2], tol) not in self.height_set:
                    return False
        return True

    def _plot(self):
        plot(self.plot_orignal_traj, self.map)

    
if __name__ == "__main__":
    traj = np.loadtxt(READ_FILE, delimiter=',', dtype=float)
    map = heighmap_2_np_reader(HEIGHT_MAP)
    planner = Local_Planner(None, traj, map)
    planner.plan(traj)
    breakpoint()

