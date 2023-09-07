import csv

import numpy as np
from scipy.optimize import minimize
import osqp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import ListedColormap
import scipy.sparse as sp
from scipy.sparse import identity, csr_matrix

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


def plot_traj(traj, sphere_center, sphere_radius, original_traj=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], marker='o', markersize=1)
    ax.plot(original_traj[:, 0], original_traj[:, 1], original_traj[:, 2], marker='o', markersize=1)

    # Plot the sphere constraint
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = sphere_center[0] + sphere_radius * np.outer(np.cos(u), np.sin(v))
    y = sphere_center[1] + sphere_radius * np.outer(np.sin(u), np.sin(v))
    z = sphere_center[2] + sphere_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='red', alpha=0.3)

    # Set plot limits and labels
    ax.set_xlim(0, 0.5)
    ax.set_ylim(0, 0.5)
    ax.set_zlim(0, 0.5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()



class Local_Planner(object):

    def __init__(self, robot, traj, map) -> None:
        self.traj = traj
        self.robot = robot
        self.map = map
        self.height_set = self.get_height_set(self.map)
        self.EE_traj = {"FL_FOOT": {}, "FR_FOOT": {}, 
                        "HL_FOOT": {}, "HR_FOOT": {}}
        self.EE_traj_optumized = {"FL_FOOT": {}, "FR_FOOT": {}, 
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
        for foot_name in ("FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"):
            self.get_contact_idx(foot_name, self.EE_traj[foot_name]["traj"])
        
        self.optimize(self.EE_traj["FL_FOOT"])

    # def cost_function(self, inputs, expert_traj):
    #     control_nodes = inputs.reshape((expert_traj.shape[0], 3))
    #     error = np.sum((control_nodes - expert_traj) ** 2)
    #     return error

    def constraints(self, traj):
        pass

    # Ground clearance constraint function
    def ground_clearance_constraint(self, x, ground_height):
        return x[:, 2] - ground_height

    def cost_function_track(self, x, expert_traj):
        error = np.linalg.norm(x - expert_traj, ord=2)
        return error

    def smoothness_penalty(self, x):
        jerk = np.diff(np.diff(x, axis=0), axis=0)
        jerk_norm = np.linalg.norm(jerk, axis=1)
        penalty = np.sum(jerk_norm**2)
        return penalty

    def spacing_penalty(self, x):
        squared_distances = np.sum(np.diff(x, axis=0) ** 2, axis=1)
        regularization_term = np.sum(squared_distances)
        return regularization_term

    def length_penalty(self, x):
        return np.sum(np.linalg.norm(np.diff(x, axis=0), axis=1))

    def calculate_potential_field(self, x, sphere_radius, sphere_center):
        _x = x.reshape((-1, 3))
        distances = np.linalg.norm(_x - sphere_center, axis=1)
        return  (1 / distances + 0.005)
        # potential_field_values  = np.where(distances < sphere_radius, (1.0 / distances + 1), 0.0)
        # return potential_field_values

    def cost_function(self, x, expert_traj, sphere_radius, sphere_center):
        # potential_field_values = self.calculate_potential_field(x, sphere_radius, sphere_center)
        # obstacle_cost = np.sum(potential_field_values)
        _x = x.reshape((-1, 3))
        # jerk_cost = self.smoothness_penalty(_x)
        track_cost = self.cost_function_track(_x, expert_traj)
        # traj_length_cost = self.length_penalty(_x)
        # spacing_cost = self.spacing_penalty(_x)
        # total_cost = track_cost + obstacle_cost * 3.0
        # total_cost = track_cost * 2 + traj_length_cost 
        total_cost = track_cost
        return total_cost

    def sphere_constraint(self, x, sphere_center, sphere_radius):
        _x = x.reshape((-1, 3))  # Reshape the flattened x to a 2D array
        distances = np.linalg.norm(_x - sphere_center, axis=1)
        return distances - sphere_radius   # Should be >= 0 for valid constraint

    def get_swing_traj(self,traj):
        swing_traj = list()
        for swing_start_end in traj['swing_idx']:
            swing_traj.append(traj['traj'][swing_start_end[0]:swing_start_end[1]])
        return swing_traj
    
    def optimize(self, traj):
        expert_traj = traj['traj']
        swing_traj = self.get_swing_traj(traj)

        sphere_radius = 0.05
        sphere_center = np.array([0.3, 0.15, 0.05])
        ground_height = 0.0  # Minimum height above ground (z = 0)
        constraints = [{'type': 'ineq', 'fun': lambda x: self.sphere_constraint(x, sphere_center, sphere_radius)}]
        for swing_t in swing_traj:
            min_swing = swing_t
            num_nodes = len(min_swing)
            # initial_guess = np.zeros((num_nodes, 3))
            # initial_guess = min_swing.flatten()
            initial_guess = np.linspace(min_swing[0], min_swing[-1], num_nodes).flatten()
            result = minimize(self.cost_function, initial_guess, args=(min_swing, sphere_radius, sphere_center), method="SLSQP", options={'maxiter': 100}, constraints=constraints)
            print(result)
            optimized_traj = result.x.reshape((min_swing.shape[0], 3))
            breakpoint()
            plot_traj(optimized_traj, sphere_center, sphere_radius, min_swing)
        # constraints = [{}]
        breakpoint()
        result = minimize(self.cost_function, initial_guess, args=(expert_traj,))

        breakpoint()


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

