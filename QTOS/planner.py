import numpy as np
import matplotlib.pyplot as plt
import math
import os
import heapq
from scipy.interpolate import CubicSpline

from QTOS.containers import Limited_Stack, LimitedFIFOQueue
import QTOS.config.global_cfg as global_cfg

TOWR_HEIGHT_MAP = "../data/heightmaps/test_heightfield_towr.txt"
GLOBAL_TRAJ_DIR = "./data/plots/"
GLOBAL_TRAJ_MAP = "./data/plots/global_plan.png"

class Global_Planner(object):
    
    def __init__(self, args, lookahead=7500, hz=1000, start_goal_pts_history_sz = 500, scale=1):
        """
        Initializes an instance of the Global_Planner class with the provided parameters.

        Args:
            args (dict): A dictionary containing configuration and settings.
            lookahead (int, optional): The lookahead distance for planning (default: 7500).
            hz (int, optional): The frequency in Hz for planning (default: 1000).
            start_goal_pts_history_sz (int, optional): The size of the history buffer for start and goal points (default: 500).
            scale (int, optional): A scaling factor (default: 1).

        Attributes:
            error_bound (float): The error bound value (default: 0.1).
            approx_z (float): The approximate z-coordinate value (default: 0.24).
            start_goal_pts (Limited_Stack): A limited-size stack for storing start and goal points.
            P_correction (float): A correction value based on 'args'.
            plan_state (np.ndarray): An array representing the planner's state (initially zeros).
            robot_state (np.ndarray): An array representing the robot's state (initially zeros).
            plan_desired_goal_pt (np.ndarray): An array representing the desired goal point (initially zeros).
            plan_desired_start_pt (np.ndarray): An array representing the desired start point (initially zeros).
            args (dict): The configuration and settings provided as input.
            origin_x_shift (float): A shift value for the x-origin (default: 1.0).
            origin_y_shift (float): A shift value for the y-origin (default: 1.0).
            grid_res (float): The resolution of the grid based on 'args'.
            map (object): The height map object obtained from 'args'.
            path_solver (PATH_Solver): An instance of the PATH_Solver class for path solving.
            error_tol (float): The error tolerance value (default: 0.05).
            CoM_avg (float): The average Center of Mass (CoM) value (default: 0.0).
            CoM_avg_container (LimitedFIFOQueue): A limited-size FIFO queue for storing CoM values.
            max_t (float): The maximum time value obtained from 'path_solver'.

        Returns:
            None
        """
        self.error_bound = 0.1
        self.approx_z = 0.24
        self.lookahead = lookahead
        self.hz = hz
        self.start_goal_pts = Limited_Stack(start_goal_pts_history_sz)
        self.P_correction = args['mpc_p']
        self.plan_state = np.zeros(3)
        self.robot_state = np.zeros(3)
        self.plan_desired_goal_pt = np.zeros(3)
        self.plan_desired_start_pt = np.zeros(3)
        self.args = args
        self.origin_x_shift = 1.0
        self.origin_y_shift = 1.0
        self.grid_res = args['args']['resolution']
        self.map = args['sim'].height_map
        if not os.path.exists(GLOBAL_TRAJ_DIR):
            os.makedirs(GLOBAL_TRAJ_DIR)
        if args['sim'].bool_map is not None:
            self.path_solver = PATH_Solver(self.map, args['-s'], global_cfg.ROBOT_CFG.robot_goal, self.args, bool_map=args['sim'].bool_map, grid_res=args['args']['resolution'])
        else:
            self.path_solver = PATH_Solver(self.map, args['-s'], global_cfg.ROBOT_CFG.robot_goal, self.args, grid_res=args['args']['resolution'])
        self.error_tol = 0.05
        self.CoM_avg = 0.0
        self.CoM_avg_container = LimitedFIFOQueue(500)
        self.max_t = self.path_solver.predicted_t

    def push_CoM(self, xy):
        """
        Pushes the Center of Mass (CoM) coordinates into a container for tracking.

        Args:
            xy (tuple or list): A tuple or list containing the x and y coordinates of the CoM.

        Returns:
            None
        """
        self.CoM_avg_container.enqueue(xy)

    def get_avg_CoM(self):
        """
        Calculates and returns the average Center of Mass (CoM) coordinates from the container.

        Returns:
            tuple: A tuple containing the average x and y coordinates of the CoM.
        """
        return self.CoM_avg_container.average()

    def error_pose_test(self, robot_pose, plan_pose, eps=0.0):
        """
        Checks if the error between robot and planned poses exceeds a specified threshold.

        Args:
            robot_pose (np.ndarray): The current robot pose as an array [x, y, theta].
            plan_pose (np.ndarray): The planned pose as an array [x, y, theta].
            eps (float, optional): Additional error tolerance (default: 0.0).

        Returns:
            bool: True if the error exceeds the threshold, False otherwise.
        """
        return np.linalg.norm(robot_pose - plan_pose) > self.error_bound + eps

    def proj(self, A, B):
        """
        Calculate the projection of vector A onto vector B.

        Parameters:
        A (numpy.ndarray): The vector to be projected.
        B (numpy.ndarray): The target vector onto which A is projected.

        Returns:
        numpy.ndarray: The projection of vector A onto vector B.
        """
        dot_p = np.dot(A, B)
        sq_magnitude = np.dot(B, B)
        proj_vec = (dot_p / sq_magnitude) * B
        return proj_vec

    def plan_robot_error(self, plan_state, robot_state):
        """Error Between the plan and the current estimate position of the robot

        Args:
            plan_state (np.array(2 or 3)): The 2d position in the trajectory node
            robot_state (np.array(2 or 3)): The estimated 2d position of the robot 

        Returns:
            np.array(): The error vector between the robot position and trajectory position
        """
        return plan_state - robot_state

    def spine_step(self, CoM, timestep, total_traj_time=5.0, tol=0.00001):
        """
        Perform a step for spine control based on future trajectory information.

        This function calculates a step for spine control based on the future position of the spine along a trajectory.
        
        Parameters:
        CoM (numpy.ndarray): The current center of mass position.
        timestep (float): The current time step.
        total_traj_time (float, optional): Total trajectory time used to calculate the future position (default 5.0).
        tol (float, optional): Tolerance value for checking if the future spine position is close to zero (default 0.00001).

        Returns:
        numpy.ndarray: The updated center of mass position after taking the step.
        """
        step_size = self.args['step_size']
        timestep_future = timestep + total_traj_time
        x_future = self.path_solver.spine_x_track(timestep_future).item() if (np.abs(self.path_solver.spine_x_track(timestep_future)) > tol) else 0.0
        y_future = self.path_solver.spine_y_track(timestep_future).item() if (np.abs(self.path_solver.spine_y_track(timestep_future)) > tol) else 0.0
        z_goal = self.get_map_height((x_future, y_future))
        goal = np.array([x_future, y_future, z_goal + self.approx_z])
        diff_vec = np.clip(goal - CoM, -step_size, step_size)
        return CoM + diff_vec

    def goal_step(self, CoM):
        """
        Perform a step towards a predefined goal position for the robot's center of mass.

        This function calculates a step for the robot's center of mass (CoM) to approach a predefined goal position.

        Parameters:
        CoM (numpy.ndarray): The current center of mass position.

        Returns:
        numpy.ndarray: The updated center of mass position after taking the step towards the goal.
        """
        step_size = self.args['step_size']
        goal = global_cfg.ROBOT_CFG.robot_goal
        diff_vec = np.clip(goal - CoM, -step_size, step_size)
        return CoM + diff_vec

    def lookahead_timestamp(self, time):
        """
        Calculate the timestamp for a future time point with a specified lookahead duration.

        This function calculates the timestamp for a future time point by adding a specified lookahead duration
        to the given time.

        Parameters:
        time (float): The current timestamp.

        Returns:
        float: The timestamp for the future time point with the lookahead duration added, rounded to 3 decimal places.
        """
        return time + round(self.lookahead / self.hz, 3)

    def update(self, timestep, plan_state_xy, robot_state_xy, goal_step_vec):
        """
        Update the state of the planner based on the provided inputs.

        Args:
            timestep (float): The current timestep value.
            plan_state_xy (list): List of plan state coordinates (x, y).
            robot_state_xy (list): List of robot state coordinates (x, y).
            goal_step_vec (int): The step number in the goal trajectory.

        Returns:
            None
        """
        # Update the robot and trajectory state
        self.plan_state = np.array(plan_state_xy)
        self.robot_state = np.array(robot_state_xy)
        
        # Calculate lookahead time and desired start point for the trajectory
        lookahead_time = self.lookahead_timestamp(timestep)
        plan_desired_start_pt = np.array([
            self.path_solver.spine_x_track(lookahead_time),
            self.path_solver.spine_y_track(lookahead_time),
            0
        ])
        
        # Determine the desired height (z) based on the map
        z = self.get_map_height(plan_desired_start_pt[0:2]) + self.approx_z
        plan_desired_start_pt[2] = z
        
        # Calculate the goal point for the trajectory
        goal_pt = self.spine_step(plan_desired_start_pt, lookahead_time)
        self.plan_desired_goal_pt = goal_pt
        
        # Create a tuple of the desired start and goal points and push it to the container
        plan_start_goal_tuple = (plan_desired_start_pt, self.plan_desired_goal_pt)
        self.start_goal_pts.push(plan_start_goal_tuple)

    def pop(self):
        """
        Pop the top element from the start_goal_pts container.

        Returns:
            tuple: A tuple containing the desired start and goal points.
        """
        return self.start_goal_pts.pop()
    
    def empty(self):
        """
        Check if the start_goal_pts container is empty.

        Returns:
            bool: True if the container is empty, False otherwise.
        """
        return self.start_goal_pts.is_empty()

    def convert_2_idx(self, x, y):
        """
        Convert Cartesian coordinates to grid indices.

        Args:
            x (float): The x-coordinate in Cartesian space.
            y (float): The y-coordinate in Cartesian space.

        Returns:
            tuple: A tuple containing the row and column indices in the grid.
        """
        row = math.floor((y + self.origin_y_shift) / self.grid_res)
        column = math.floor((x + self.origin_x_shift) / self.grid_res)
        return row, column
    
    def get_map_height(self, pos):
        """
        Get the height of the map at a given position.

        Args:
            pos (list): List containing the x and y coordinates in Cartesian space.

        Returns:
            float: The height of the map at the specified position.
        """
        try:
            row, col = self.convert_2_idx(pos[0], pos[1])
            return self.map[row, col]
        except:
            last_row, last_col = self.map.shape
            return self.map[last_row - 1, last_col//2]

class PATH_Solver(object):
    
    def __init__(self, map, start, goal, args, grid_res = 0.1, origin_x_shift=1, origin_y_shift=1, visual=True, bool_map=None) -> None:
        """
        Initialize the PATH_Solver class.

        Args:
            map (numpy.ndarray): The map data for path planning.
            start (tuple): The start position (x, y).
            goal (tuple): The goal position (x, y).
            args (dict): A dictionary of arguments.
            grid_res (float, optional): The grid resolution (default 0.1).
            origin_x_shift (float, optional): X-coordinate shift (default 1).
            origin_y_shift (float, optional): Y-coordinate shift (default 1).
            visual (bool, optional): Whether to visualize the path (default True).
            bool_map (numpy.ndarray, optional): A boolean map (default None).

        Returns:
            None
        """
        self.visual_map = map
        if bool_map is None:
            self.bool_map = self.visual_map
        else:
            self.bool_map = bool_map
        self.args = args
        self.grid_res = grid_res
        self.solution_flag = False
        self.origin_x_shift = origin_x_shift
        self.origin_y_shift = origin_y_shift
        self.start_pos_x_y = start[0:2]
        self.goal_pos_x_y = goal[0:2]
        self.start_idx_x_y = self.convert_2_idx(self.start_pos_x_y[0], self.start_pos_x_y[1])
        self.goal_idx_x_y = self.convert_2_idx(self.goal_pos_x_y[0], self.goal_pos_x_y[1])
        self.path = self.astar(self.start_idx_x_y, self.goal_idx_x_y)
        self.predicted_t = np.linalg.norm(np.array(start[0:2]) - np.array(goal[0:2])) / (self.args['step_size']) * 10
        if self.solution_flag:
            self.solution_flag = True
            self._solve()
            if visual:
                self.visualize_path()
        else:
            print("Failed to find a solution")
        
    def convert_2_idx(self, x, y):
        """
        Convert Cartesian coordinates to grid indices.

        Args:
            x (float): The x-coordinate in Cartesian space.
            y (float): The y-coordinate in Cartesian space.

        Returns:
            tuple: A tuple containing the row and column indices in the grid.
        """
        row = math.floor((y + self.origin_y_shift) / self.grid_res)
        column = math.floor((x + self.origin_x_shift) / self.grid_res)
        return row, column

    def heuristic(self, a, b):
        """
        Calculate the heuristic distance between two points.

        Args:
            a (tuple): The coordinates of point A (x, y).
            b (tuple): The coordinates of point B (x, y).

        Returns:
            float: The heuristic distance between points A and B.
        """
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def astar(self, start, goal, height_bound=0.2):
        """
        Perform the A* path planning algorithm.

        Args:
            start (tuple): The start grid cell coordinates (row, column).
            goal (tuple): The goal grid cell coordinates (row, column).
            height_bound (float, optional): Height threshold for traversability (default 0.2).

        Returns:
            list or None: A list of grid cell coordinates representing the path from start to goal, or None if no path is found.
        """
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        close_set = set()
        came_from = {}
        gscore = {start: 0 }
        fscore = {start: self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        i = 0
        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                self.solution_flag = True
                return [start] + data[::-1]
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                _g_score = gscore[current] + self.heuristic(current,  neighbor) #Current min cost from start node [start node] to current [n]
                if 0 <= neighbor[0] < self.bool_map.shape[0] and 0 <= neighbor[1] < self.bool_map.shape[1]: #Check for a feasible solution
                    if self.bool_map[neighbor[0]][neighbor[1]] > height_bound:
                        continue
                else:
                    continue
                if neighbor in close_set and _g_score >= gscore.get(neighbor, 0): #skipping if neighbor has been visited again
                    continue
                if _g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]: #
                    came_from[neighbor] = current #store the path where n+1 node came from 
                    gscore[neighbor] = _g_score
                    fscore[neighbor] = _g_score + self.heuristic(neighbor, goal) # f(n) = g(n) + h(n) Compute the total cost of the node 
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        return None

    def _solve(self):
        """
        Solve the path planning problem and generate trajectory splines.

        Returns:
            None
        """
        t = np.linspace(0, self.predicted_t, len(self.path[::2]) + 1)
        path_x_track = [(pos[1] * self.grid_res) - self.origin_x_shift for pos in self.path[::2]]
        path_x_track.append(self.path[-1][1] * self.grid_res)
        path_y_track = [(pos[0] * self.grid_res) - self.origin_y_shift for pos in self.path[::2]]
        path_y_track.append(self.path[-1][0] * self.grid_res)
        path_x_plot = [pos[1] * self.grid_res for pos in self.path[::2]]
        path_x_plot.append(self.path[-1][1] * self.grid_res)
        path_y_plot = [pos[0] * self.grid_res for pos in self.path[::2]]
        path_y_plot.append(self.path[-1][0] * self.grid_res)
        self.spine_x_track = CubicSpline(t, path_x_track)
        self.spine_y_track = CubicSpline(t, path_y_track)
        self.spine_x_plot = CubicSpline(t, path_x_plot)
        self.spine_y_plot = CubicSpline(t, path_y_plot)

    def solve(self, start, goal, plot=False):
        """
        Solve the path planning problem with new start and goal positions.

        Args:
            start (tuple): The new start position (x, y).
            goal (tuple): The new goal position (x, y).
            plot (bool, optional): Whether to plot the trajectory (default False).

        Returns:
            None
        """
        self.start_pos_x_y = start[0:2]
        self.goal_pos_x_y = goal[0:2]
        self.start_idx_x_y = self.convert_2_idx(start[0], start[1])
        self.goal_idx_x_y = self.convert_2_idx(goal[0], goal[1])
        self.path = self.astar(self.start_idx_x_y, self.goal_idx_x_y)
        self.predicted_t = np.linalg.norm(np.array(start[0:2]) - np.array(goal[0:2])) / (self.args['step_size']) * 10
        if self.solution_flag:
            t = np.linspace(0, self.predicted_t, len(self.path[::2]) + 1)
            path_x_track = [(pos[1] * self.grid_res) - self.origin_x_shift for pos in self.path[::2]]
            path_x_track.append(self.path[-1][1] * self.grid_res)
            path_y_track = [(pos[0] * self.grid_res) - self.origin_y_shift for pos in self.path[::2]]
            path_y_track.append(self.path[-1][0] * self.grid_res)
            path_x_plot = [pos[1] * self.grid_res for pos in self.path[::2]]
            path_x_plot.append(self.path[-1][1] * self.grid_res)
            path_y_plot = [pos[0] * self.grid_res for pos in self.path[::2]]
            path_y_plot.append(self.path[-1][0] * self.grid_res)
            self.spine_x_track = CubicSpline(t, path_x_track)
            self.spine_y_track = CubicSpline(t, path_y_track)
            self.spine_x_plot = CubicSpline(t, path_x_plot)
            self.spine_y_plot = CubicSpline(t, path_y_plot)
        else:
            print("Failed to solve")
        if plot:
            self.visualize_path()


    def visualize_path(self, plot_spline=True, plot_nodes=False):
        """
        Visualize the path and trajectory on the map.

        Args:
            plot_spline (bool, optional): Whether to plot the trajectory spline (default True).
            plot_nodes (bool, optional): Whether to plot individual nodes in the path (default False).

        Returns:
            None
        """
        x_range = np.arange(0, self.visual_map.shape[1]) * self.grid_res
        y_range = np.arange(0, self.visual_map.shape[0]) * self.grid_res
        X, Y = np.meshgrid(x_range, y_range)
        dpi = 600
        width, height = 10, 6
        fig, ax = plt.subplots(figsize=(width, height))
        plt.pcolormesh(X, Y, self.visual_map, cmap='gray_r', shading='auto')
        plt.scatter(self.start_pos_x_y[0] + self.origin_x_shift, self.start_pos_x_y[1] + self.origin_y_shift, color='green', marker='o', label='Start')
        plt.scatter(self.goal_pos_x_y[0] + self.origin_x_shift, self.goal_pos_x_y[1] + self.origin_y_shift, color='red', marker='x', label='Goal')
        if self.path:
            path_x = [pos[1] * self.grid_res for pos in self.path]
            path_y = [pos[0] * self.grid_res for pos in self.path]
            if plot_nodes:
                plt.plot(path_x, path_y, color='blue', label='Path')
            if plot_spline:
                t_new = np.linspace(0, self.predicted_t, 100)
                path_x_new = self.spine_x_plot(t_new)
                path_y_new = self.spine_y_plot(t_new)
                plt.plot(path_x_new, path_y_new, color='blue', label='Trajectory Spline')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Global Trajectory Path')
        plt.axis('off')
        plt.savefig(GLOBAL_TRAJ_MAP)

if __name__ == "__main__":
   pass