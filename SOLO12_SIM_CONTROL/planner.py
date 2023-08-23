
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import math
import heapq
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline
from scipy.interpolate import BSpline


from SOLO12_SIM_CONTROL.config.global_cfg import PLANNER
from SOLO12_SIM_CONTROL.containers import FIFOQueue, Limited_Stack
import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg

TOWR_HEIGHT_MAP = "../data/heightmaps/test_heightfield_towr.txt"
GLOBAL_TRAJ_MAP = "./data/plots/global_plan.png"


class Global_Planner(object):
    
    def __init__(self, args, lookahead=7500, hz=1000, start_goal_pts_history_sz = 10):
        self.error_bound = 0.1
        self.lookahead = lookahead
        self.forced_num = None
        self.hz = hz
        self.set_correct_flag = False
        self.start_goal_pts = Limited_Stack(start_goal_pts_history_sz)
        self.P_correction = True # correction for proportional error btw planner and robot CoM
        self.plan_state = None
        self.robot_state = None
        self.plan_desired_goal_pt = np.zeros(3)
        self.plan_desired_start_pt = np.zeros(3)
        self.args = args
        self.path_solver = PATH_Solver(args['map'], args['-s'], global_cfg.ROBOT_CFG.robot_goal, self.args)
        self.error_tol = 0.15
        

    def error_pose_test(self, robot_pose, plan_pose, eps=0.0):
        return np.linalg.norm(robot_pose - plan_pose) > self.error_bound + eps

    def proj(self, A, B):
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

    def goal_step(self, CoM):
        step_size = self.args['step_size']
        goal = global_cfg.ROBOT_CFG.robot_goal
        diff_vec = np.clip(goal - CoM, -step_size, step_size)
        return CoM + diff_vec


    def lookahead_timestamp(self, time):
        return time + round(self.lookahead / self.hz, 3)


    def update(self, timestep, plan_state_xy, robot_state_xy, goal_step_vec):
        """Template to update the state of the planner

        Args:
            timestep (_type_): _description_
            plan (_type_): _description_
            goal_step (_type_): _description_
        """
        ###Update the robot and trajectory state
        self.plan_state = np.array(plan_state_xy)
        self.robot_state = np.array(robot_state_xy)
        ###Calculate the error btw robot and trajectory
        error = self.plan_robot_error(self.plan_state, self.robot_state)
        total_err = np.linalg.norm(error)
        lookahead_time = self.lookahead_timestamp(timestep)
        plan_desired_start_pt = np.array([self.path_solver.spine_x_track(lookahead_time), self.path_solver.spine_y_track(lookahead_time), 0.24])
        goal_pt = self.goal_step(plan_desired_start_pt)
        z = 0.24 #Might need to add info about the grid map to query height map + 0.24 (optimal height z-height for robot)
        if self.P_correction and total_err > self.error_tol:
            self.plan_desired_goal_pt[0:2] = self.P_goal_point(goal_pt[0:2], error)
            self.plan_desired_goal_pt[2] = z
            plan_start_goal_tuple = (plan_desired_start_pt, self.plan_desired_goal_pt)
            self.start_goal_pts.push(plan_start_goal_tuple)
        else:
            self.start_goal_pts.clear()
            self.plan_desired_goal_pt = goal_pt
            plan_start_goal_tuple = (plan_desired_start_pt, self.plan_desired_goal_pt)
            self.start_goal_pts.push(plan_start_goal_tuple)



    def P_goal_point(self, goal_pt, error = [0, 0], kp=1.0):
        return goal_pt + (kp * error)

    def D_goal_point(self, goal_pt, d_error = [0, 0], kd=1.0):
        return goal_pt + (kd * d_error)

    def pop(self):
        return self.start_goal_pts.pop()
    
    def empty(self):
        return self.start_goal_pts.is_empty()

class PATH_Solver(object):
    
    def __init__(self, map, start, goal, args, grid_res = 0.1, origin_x_shift=1, origin_y_shift=1, visual=True) -> None:
        self.in_map = map
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
        row = math.floor((y + self.origin_y_shift) / self.grid_res)
        column = math.floor((x + self.origin_x_shift) / self.grid_res)
        return row, column

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def astar(self, start, goal, height_bound=0):

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
                print("Found a path")
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
                if 0 <= neighbor[0] < self.in_map.shape[0] and 0 <= neighbor[1] < self.in_map.shape[1]: #Check for a feasible solution
                    if self.in_map[neighbor[0]][neighbor[1]] > height_bound:
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
        t = np.linspace(0, self.predicted_t, len(self.path))
        path_x_track = [(pos[1] * self.grid_res) - self.origin_x_shift for pos in self.path]
        path_y_track = [(pos[0] * self.grid_res) - self.origin_y_shift for pos in self.path]
        path_x_plot = [pos[1] * self.grid_res for pos in self.path]
        path_y_plot = [pos[0] * self.grid_res for pos in self.path]
        self.spine_x_track = CubicSpline(t, path_x_track)
        self.spine_y_track = CubicSpline(t, path_y_track)
        self.spine_x_plot = CubicSpline(t, path_x_plot)
        self.spine_y_plot = CubicSpline(t, path_y_plot)


    def solve(self, start, goal, plot=False):
        self.start_pos_x_y = start[0:2]
        self.goal_pos_x_y = goal[0:2]
        self.start_idx_x_y = self.convert_2_idx(start[0], start[1])
        self.goal_idx_x_y = self.convert_2_idx(goal[0], goal[1])
        self.path = self.astar(self.start_idx_x_y, self.goal_idx_x_y)
        self.predicted_t = np.linalg.norm(np.array(start[0:2]) - np.array(goal[0:2])) / (self.args['step_size']) * 10
        if self.solution_flag:
            t = np.linspace(0, self.predicted_t, len(self.path))
            path_x_track = [(pos[1] * self.grid_res) - self.origin_x_shift for pos in self.path]
            path_y_track = [(pos[0] * self.grid_res) - self.origin_y_shift for pos in self.path]
            path_x_plot = [pos[1] * self.grid_res for pos in self.path]
            path_y_plot = [pos[0] * self.grid_res for pos in self.path]
            self.spine_x_track = CubicSpline(t, path_x_track)
            self.spine_y_track = CubicSpline(t, path_y_track)
            self.spine_x_plot = CubicSpline(t, path_x_plot)
            self.spine_y_plot = CubicSpline(t, path_y_plot)
        else:
            print("Failed to solve")
        if plot:
            self.visualize_path()


    def visualize_path(self, k=5, plot_spline=True, plot_nodes=False):

        x_range = np.arange(0, self.in_map.shape[1]) * self.grid_res
        y_range = np.arange(0, self.in_map.shape[0]) * self.grid_res
        X, Y = np.meshgrid(x_range, y_range)

        plt.figure()
        plt.pcolormesh(X, Y, self.in_map, cmap='gray', shading='auto')

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
        plt.legend()
        plt.savefig(GLOBAL_TRAJ_MAP)

if __name__ == "__main__":
   pass