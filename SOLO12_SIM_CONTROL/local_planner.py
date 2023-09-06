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
        self.EE_traj = {"FL_FOOT": {}, "FR_FOOT": {}, 
                        "HL_FOOT": {}, "HR_FOOT": {}}

    def plan(self, traj):
        self._filter(traj)
        pass

    def _process(self, traj):
        pass

    def contact_swing_phase(self, contacts):
        contact_idx = []
        swing_idx = []
        last_contact = 0
        for idx in contacts:
            if idx[0] == 0:
                contact_idx.append((idx[0], idx[-1]))
                last_contact = idx[-1]
            else:
                contact_idx.append((idx[0], idx[-1]))
                swing_idx.append((last_contact + 1, idx[0] - 1))
                last_contact = idx[-1]
        return contact_idx, swing_idx

    def get_contact_idx(self, foot, traj):
        list_idx = []
        first_check, second_check = False, True
        first_idx, last_idx = 0, 0
        idx = 0
        contact_pairs = []
        _last = traj[0]
        contact_timing = list()
        for idx, ee in enumerate(traj):
            if self.check_leg_contact(ee):
                if all(_last != ee):
                    contact_pairs.append(contact_timing)
                    contact_timing = list()
                    _last = ee
                if all(_last == ee):
                    contact_timing.append(idx)
        contact_pairs.append(contact_timing)
        contact_idx, swing_idx = self.contact_swing_phase(contact_pairs)

        self.EE_traj[foot]["contact_idx"] = contact_idx
        self.EE_traj[foot]["swing_idx"] = swing_idx

    def get_gait_contact_idx(self, traj):
        list_idx = []
        for idx, (ee1, ee2, ee3, ee4) in enumerate(zip(self.EE_traj["FL_FOOT"], self.EE_traj["FR_FOOT"], self.EE_traj["HL_FOOT"], self.EE_traj["HR_FOOT"])):
            EE = (ee1, ee2, ee3, ee4)
            if self.check_legs_contact(EE):
                print(EE)
                list_idx.append(idx)
        first_idx, last_idx = list_idx[0], list_idx[-1]
        return (first_idx, last_idx)

    def _filter(self, traj):
        self.EE_traj["FL_FOOT"]["traj"] = traj[:,7:10]
        self.EE_traj["FR_FOOT"]["traj"] = traj[:,10:13]
        self.EE_traj["HL_FOOT"]["traj"] = traj[:,13:16]
        self.EE_traj["HR_FOOT"]["traj"] = traj[:,16:19]
        self.orignal_traj = (self.EE_traj["FL_FOOT"], self.EE_traj["FR_FOOT"], self.EE_traj["HL_FOOT"], self.EE_traj["HR_FOOT"])
        # breakpoint()
        for foot_name in ("FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"):
            self.get_contact_idx(foot_name, self.EE_traj[foot_name]["traj"])
        breakpoint()

    def cost_function(traj):
        pass

    def constraints(traj):
        pass

    def get_height_set(self, map):
        height_values = np.unique(map)
        return set(height_values)

    def check_leg_contact(self, EE, tol=9):
        if round(EE[2], tol) not in self.height_set:
                    return False
        return True
        
    def check_legs_contact(self, EE, tol=9):
        for ee in EE:
                if self.check_legs_contact(ee, tol):
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

