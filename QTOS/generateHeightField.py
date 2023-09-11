from QTOS.utils import is_numeric, save_bool_map, save_height_grid_map, DockerInfo, parse_scripts, cmd_args

import time
import matplotlib.pyplot as plt
from collections import namedtuple
import multiprocessing
import random
import subprocess
import shlex

import numpy as np
from scipy.spatial import ConvexHull

np.set_printoptions(threshold=np.inf)

HEIGHT_FIELD_OUT = "./data/heightfields/heightfield.txt"
TOWR_HEIGHTFIELD_OUT = "./data/heightfields/from_pybullet/towr_heightfield.txt"
NUM_PROCESSES = 32

scripts =  {'run': 'docker exec <id> ./main',
            'info': 'docker ps -f ancestor=towr',
            'heightfield_rm' : 'docker exec -t <id> rm /root/catkin_ws/src/towr_solo12/towr/data/heightfields/from_pybullet/towr_heightfield.txt',
            'heightfield_copy': 'docker cp ./data/heightfields/from_pybullet/towr_heightfield.txt <id>:root/catkin_ws/src/towr_solo12/towr/data/heightfields/from_pybullet/towr_heightfield.txt'}

_flags = ['-g', '-s', '-s_ang', '-s_vel', '-n', '-e1', '-e2', '-e3', '-e4', '-t', '-r']

def strip(x):
    """
    Removes square brackets from a string.

    Args:
        x (str): The input string containing square brackets.

    Returns:
        str: The input string with square brackets removed.
    """

    st = " "
    for s in x:
        if s == "[" or s == "]":
            st += ''
        else:
            st += s
    return st

def scale_map(map, scale_factor=1):
    """
    Scales a 2D map by replicating its elements to achieve a specified scaling factor.

    Args:
        map (numpy.ndarray): The input 2D map to be scaled.
        scale_factor (int, optional): The scaling factor. Defaults to 1.

    Returns:
        numpy.ndarray: The scaled 2D map.
    """
    scaled_map = []
    for row in map:
        scaled_row = []
        for x in row:
            scaled_row.extend([x] * scale_factor)
        scaled_map.extend([scaled_row]  * scale_factor)
    return np.array(scaled_map)


def is_index_in_bounds(idx_row, idx_col, array):
    """
    Checks if a given row and column index is within the bounds of a 2D array.

    Args:
        idx_row (int): The row index to check.
        idx_col (int): The column index to check.
        array (numpy.ndarray): The 2D array to check against.

    Returns:
        bool: True if the index is within bounds, False otherwise.
    """
    return 0 <= idx_row < array.shape[0] and 0 <= idx_col < array.shape[1]

class Map_2_Idx(object):
    """
    Represents a mapping between coordinates and indices in a map.

    Args:
        map_coords_start (tuple): The starting coordinates (x, y, z) in the map.
        map_coords_goal (tuple): The goal coordinates (x, y, z) in the map.
        map_idx_start (tuple): The starting indices (row, column) in the map.
        map_idx_goal (tuple): The goal indices (row, column) in the map.
    """

    def __init__(self, map_coords_start, map_coords_goal, map_idx_start, map_idx_goal):
        self.map_coords_start = map_coords_start
        self.map_idx_start = map_idx_start
        self.map_coords_goal = map_coords_goal
        self.map_idx_goal =  map_idx_goal

    def __repr__(self) -> str:
        return f"Map_2_Idx(map_coords_start={self.map_coords_start}, map_coords_goal={self.map_coords_goal}, map_idx_start={self.map_idx_start}, map_idx_goal={self.map_idx_goal})"

    def __getstate__(self):
        state = self.__dict__.copy()
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)

def heighmap_2_np_reader(file, delimiter=','):
    """
    Reads heightmap data from a text file and returns it as a numpy array.

    Args:
        file (str): The path to the text file containing heightmap data.
        delimiter (str, optional): The delimiter used in the file. Defaults to ','.

    Returns:
        numpy.ndarray: A numpy array containing the heightmap data.
    """
    data = []
    with open(file, 'r') as f:
        reader = f.readlines()
        for row in reader:
            _row = row.strip().split(delimiter)
            _row = [float(x) for x in _row if is_numeric(x)]
            data.append(_row)
    return np.transpose(np.array(data))

def scale_values(file, scale=1.0):
    """
    Scales numeric values in a text file by a specified factor.

    Args:
        file (str): The path to the text file containing numeric values.
        scale (float, optional): The scaling factor. Defaults to 1.0.

    Returns:
        list: A list of lists containing the scaled numeric values.
    """

    scaled_values = []
    with open(file, 'r') as f:
        lines = f.readlines()
        for line in lines:
            nums = [x.strip() for x in line.strip().split(',')]
            scaled_line = []
            for num in nums:
                try:
                    if is_numeric(num):
                        num = float(num)
                        scaled_num = num * scale
                        scaled_line.append(scaled_num)
                except ValueError:
                    pass
            scaled_values.append(scaled_line)
    return scaled_values

def max_height(file):
    """
    Finds the maximum numeric value in a text file.

    Args:
        file (str): The path to the text file containing numeric values.

    Returns:
        float: The maximum numeric value found in the file.
    """
    max_num = 0
    with open(file, 'r') as f:
        lines = f.readlines()
        for line in lines:
            nums = [x.strip() for x in line.strip().split(',')]
            for num in nums:
                try:
                    num = float(num)
                    max_num = max(num, max_num)
                except:
                    pass
    return max_num

class PATH_MAP(object):
    """A class for path mapping and obstacle detection.

    This class generates a path map and performs obstacle detection using
    multiprocessing. It probes the input height map for path planning.

    Attributes:
        map (numpy.ndarray): The input height map.
        origin_shift_x (float): X-axis origin shift value.
        origin_shift_y (float): Y-axis origin shift value.
        docker_id (DockerInfo): An instance of DockerInfo for Docker-related information.
        scripts (dict): A dictionary containing various script paths.
        bool_map (numpy.ndarray): A binary map representing obstacle presence.
        data_queue (multiprocessing.Queue): A queue for passing data to worker processes.
        lock (multiprocessing.Lock): A lock for synchronization.
        shared_arr (multiprocessing.Array): A shared memory array for communication with workers.
        num_cols (int): Number of columns in the height map.
        mesh_resolution (float): Mesh resolution for probing the map.
        neighbors_start (numpy.ndarray): Indices of neighbors around the start point.
        neighbors_mid (numpy.ndarray): Indices of neighbors around the middle point.
        neighbors_end (numpy.ndarray): Indices of neighbors around the end point.
    """
    
    def __init__(self, map, multi_map_shift=1, scale=1):
        """Initialize a PATH_MAP object.

        Args:
            map (numpy.ndarray): The input height map.
            multi_map_shift (int, optional): Multi-map shift value. Defaults to 1.
            scale (int, optional): Scaling factor. Defaults to 1.
        """
        self.map = map
        self.origin_shift_x = 1.0
        self.origin_shift_y = 1.0
        self.docker_id = DockerInfo()
        self.scripts = parse_scripts(scripts, self.docker_id)
        self.setup()
        self.bool_map = np.zeros((map.shape[0], map.shape[1]), dtype=int)
        self.data_queue = multiprocessing.Queue()
        self.lock = multiprocessing.Lock()
        self.shared_arr = multiprocessing.Array('i', map.shape[0] * map.shape[1])
        self.num_cols = map.shape[1]
        self.mesh_resolution = 0.1 * (1 / scale)
        self.probe_map(map, multi_map_shift, self.mesh_resolution )
        neighbors = ((-(scale*3), 0), (scale*3, 0), (0, -(scale*3)), (0, (scale*3)))
        neighbors_mid = ((-scale*3, 0), (scale*3, 0), (0, scale*3), (0, -scale*3), (-scale*3, 0), (scale*3, 0), (0, -scale*3), (0, -scale*3))
        self.neighbors_start = self.find_convex_hull(neighbors)
        self.neighbors_mid = self.find_convex_hull(neighbors_mid)
        self.neighbors_end = self.find_convex_hull(neighbors)
        if self.check_flat_ground(map):
            pass
        else:
            self.run()

    def find_convex_hull(self, points):
        """Find the convex hull of a set of points.

        Args:
            points (list): List of (x, y) coordinates.

        Returns:
            numpy.ndarray: Indices of the convex hull points.
        """
        points = np.array(points)
        hull = ConvexHull(points)
        hull_vertices = points[hull.vertices]
        min_x, max_x = np.min(hull_vertices[:, 0]), np.max(hull_vertices[:, 0])
        min_y, max_y = np.min(hull_vertices[:, 1]), np.max(hull_vertices[:, 1])
        grid = np.zeros((max_y - min_y + 1, max_x - min_x + 1), dtype=int)

        for y in range(min_y, max_y + 1):
            intersections = []
            for i in range(len(hull_vertices)):
                x1, y1 = hull_vertices[i]
                x2, y2 = hull_vertices[(i + 1) % len(hull_vertices)]

                if y1 == y2:
                    continue

                if y1 <= y <= y2 or y2 <= y <= y1:
                    x_intersection = int(x1 + (x2 - x1) * (y - y1) / (y2 - y1))
                    intersections.append(x_intersection)

            intersections.sort()
            if len(intersections) >= 2:
                start_x = intersections[0]
                end_x = intersections[-1]
                grid[y - min_y, start_x - min_x:end_x - min_x + 1] = 1

        center_map = grid.shape[0] // 2, grid.shape[1] // 2
        neighbor_indices = np.argwhere(grid == 1) - np.array([center_map[0], center_map[1]])
        return neighbor_indices

    def check_flat_ground(self, map):
        """Check if the ground is flat.

        Args:
            map (numpy.ndarray): The input height map.

        Returns:
            bool: True if the ground is flat, False otherwise.
        """
        return np.all(map == 0) or np.all(map == 0.0)

    def setup(self):
        """Run setup scripts for path mapping."""
        subprocess.run(shlex.split(self.scripts['heightfield_rm']))
        subprocess.run(shlex.split(self.scripts['heightfield_copy']))

    def neighbors_danger_test(self, map, idx_x, idx_y, sz=1):
        """Test for dangerous neighbors.

        Args:
            map (numpy.ndarray): The input height map.
            idx_x (int): X-coordinate of the point to test.
            idx_y (int): Y-coordinate of the point to test.
            sz (int, optional): Size parameter. Defaults to 1.

        Returns:
            bool: True if dangerous neighbors are present, False otherwise.
        """
        neighbor = ((sz, 0), (-sz, 0), (0, sz), (0, -sz), (sz, sz), (sz, -sz), (-sz, -sz), (-sz, sz))
        for idx in neighbor:
            if idx[0] + idx_x >= map.shape[0] or idx[0] + idx_x < 0:
                return False
            elif idx[1] + idx_y >= map.shape[1] or idx[1] + idx_y < 0:
                return False
            elif map[idx[0] + idx_x][idx[1] + idx_y] > 0:
                return True
        return False

    def probe_map(self, map, multi_map_shift = 1, mesh_resolution_meters = 0.1):
        """Probe the height map for path planning.

        Args:
            map (numpy.ndarray): The input height map.
            multi_map_shift (int, optional): Multi-map shift value. Defaults to 1.
            mesh_resolution_meters (float, optional): Mesh resolution in meters. Defaults to 0.1.
        """
        step_x = mesh_resolution_meters
        step_y = mesh_resolution_meters
        x_start = - mesh_resolution_meters * (map.shape[1] / 2) - mesh_resolution_meters / 2 + ((multi_map_shift - 1) * self.origin_shift_x)
        y_start = - mesh_resolution_meters * (map.shape[1] / 2) - mesh_resolution_meters / 2 + ((multi_map_shift - 1) * self.origin_shift_y)
        x_goal =  - mesh_resolution_meters * (map.shape[1] / 2) + mesh_resolution_meters / 2 + ((multi_map_shift - 1) * self.origin_shift_x)
        y_goal = - mesh_resolution_meters * (map.shape[1] / 2) - mesh_resolution_meters / 2 + ((multi_map_shift - 1) * self.origin_shift_y)
        _x_start, _y_start = x_start, y_start
        _x_goal, _y_goal = x_goal, y_goal
        _idx_x, _idx_y, _idx_y_offset = 0, 0, 2
        for x in range(map.shape[0]):
                _y_start += step_y
                _y_goal += step_y
                _x_start, _x_goal = x_start, x_goal
                for y in range(map.shape[1]//2  - 1):
                    if y == 0:
                        _x_start += step_x
                        _idx_y = 0
                        _idx_y_offset = 2
                    else:
                        _x_start = _x_goal
                    
                    _x_goal += (2 * step_x)
                    _x_start, _y_start = round(_x_start, 2), round(_y_start, 2)
                    _x_goal, _y_goal = round(_x_goal, 2), round(_y_goal, 2)
                    _z_start, _z_goal = map[_idx_x][_idx_y], map[_idx_x][_idx_y_offset]
                    if self.neighbors_danger_test(map, _idx_x, _idx_y) or self.neighbors_danger_test(map, _idx_x, _idx_y_offset):
                        data = Map_2_Idx(map_coords_start=(_x_start, _y_start, _z_start), map_coords_goal=(_x_goal, _y_goal, _z_goal),
                                        map_idx_start=(_idx_x, _idx_y), map_idx_goal=(_idx_x, _idx_y_offset))
                        self.data_queue.put(data)
                    _idx_y += 2
                    _idx_y_offset += 2
                _idx_x += 1

    def run(self):
        """Run path mapping using multiprocessing."""
        processes = []
        for i in range(NUM_PROCESSES):
            process = multiprocessing.Process(target=self.worker_f, args=(self.shared_arr, self.data_queue, self.num_cols))
            processes.append(process)
            process.start()
        for process in processes:
            process.join()

        self.bool_map = np.frombuffer(self.shared_arr.get_obj(), dtype=np.float32).reshape(self.map.shape[0], self.map.shape[1]).astype('int')

    def worker_f(self, map, queue, num_cols):
        """Worker function for path mapping.

        Args:
            map (multiprocessing.Array): Shared memory array for the height map.
            queue (multiprocessing.Queue): Queue for passing data to workers.
            num_cols (int): Number of columns in the height map.
        """

        def state_config(args, start_pt, goal_pt, shift=[0, 0]):
            args['-s'] = [start_pt[0], start_pt[1], start_pt[2] + 0.24]
            args['-e1'] = (np.array([0.21, 0.19, 0.0]) + np.array(shift)).tolist()
            args['-e2'] = (np.array([0.21, -0.19, 0.0]) + np.array(shift)).tolist()
            args['-e3'] = (np.array([-0.21, 0.19, 0.0]) + np.array(shift)).tolist()
            args['-e4'] = (np.array([-0.21, -0.19, 0.0]) + np.array(shift)).tolist()
            args['-s_ang'] = [0, 0, 0]
            args['-g'] = [goal_pt[0], goal_pt[1], goal_pt[2] + 0.24]
            args['-r'] = 5.0

        while not queue.empty():
            args = {}
            data = queue.get()
            start_pt = data.map_coords_start
            goal_pt = data.map_coords_goal
            shift_vec = [start_pt[0], start_pt[1], start_pt[2]]
            start_idx = data.map_idx_start
            goal_idx = data.map_idx_goal
            state_config(args, start_pt, goal_pt, shift=shift_vec)
            local_array = np.frombuffer(map.get_obj(), dtype=np.float32).reshape(-1, num_cols)
            TOWR_SCRIPT = shlex.split(self.scripts['run'] + " " + cmd_args(args))
            p_status = subprocess.run(TOWR_SCRIPT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if p_status.returncode == 0:
                with self.lock:
                    mid_idx_x, mid_idx_y = start_idx[0], start_idx[1] + 1
                    local_array[start_idx] = 0
                    local_array[mid_idx_x][mid_idx_y] = 0
                    local_array[goal_idx] = 0
            else:
                with self.lock:
                    mid_idx_x, mid_idx_y = start_idx[0], start_idx[1] + 1
                    for idx in self.neighbors_start:
                        if is_index_in_bounds(start_idx[0] + idx[0], start_idx[1] + idx[1], local_array):
                            local_array[start_idx[0] + idx[0]][start_idx[1] + idx[1]] = 1
                    for idx in self.neighbors_mid:
                        if is_index_in_bounds(mid_idx_x + idx[0], mid_idx_y + idx[1], local_array):
                            local_array[mid_idx_x + idx[0]][mid_idx_y + idx[1]]
                    for idx in self.neighbors_end:
                        if is_index_in_bounds(goal_idx[0] + idx[0], goal_idx[1] + idx[1], local_array):
                            local_array[goal_idx[0] + idx[0]][goal_idx[1] + idx[1]] = 1

class Maps(object):
    """A class for managing and generating height maps.

    This class provides access to various pre-defined height maps and allows
    for the generation of custom maps by combining multiple pre-defined maps.

    Attributes:
        calibration_file (str): Path to the calibration height map file.
        step_file (str): Path to the step height map file.
        step_1_file (str): Path to the step_1 height map file.
        step_2_file (str): Path to the step_2 height map file.
        step_3_file (str): Path to the step_3 height map file.
        wall_1_file (str): Path to the wall_1 height map file.
        wall_2_file (str): Path to the wall_2 height map file.
        wall_3_file (str): Path to the wall_3 height map file.
        wall_4_file (str): Path to the wall_4 height map file.
        test_file (str): Path to the test height map file.
        stairs_file (str): Path to the staircase height map file.
        plane_file (str): Path to the plane height map file.
        feasibility (str): Path to the feasibility_test height map file.
        feasibility_1 (str): Path to the feasibility_test_1 height map file.
        random_terrain_1 (str): Path to the random_terrain height map file.
        climb_1 (str): Path to the climb_1 height map file.
        climb_2 (str): Path to the climb_2 height map file.
        plane_file_200 (str): Path to the plane_200 height map file.
        stairs (str): Path to the stairs height map file.
        stairs_1 (str): Path to the stairs_1 height map file.
        collision_hills (str): Path to the collision_wall_hills height map file.
    """
    calibration_file = "./data/heightfields/calibration.txt"
    step_file = "./data/heightfields/step.txt"
    step_1_file = "./data/heightfields/step_1.txt"
    step_2_file = "./data/heightfields/step_2.txt"
    step_3_file = "./data/heightfields/step_3.txt"
    wall_1_file = "./data/heightfields/wall_1.txt"
    wall_2_file = "./data/heightfields/wall_2.txt"
    wall_3_file = "./data/heightfields/wall_3.txt"
    wall_4_file = "./data/heightfields/wall_4.txt"
    test_file = "./data/heightfields/heightfield_test.txt"
    stairs_file = "./data/heightfields/staircase.txt"
    plane_file =  "./data/heightfields/plane.txt"
    feasibility = "./data/heightfields/feasibility_test.txt"
    feasibility_1 = "./data/heightfields/feasibility_test_1.txt"
    random_terrain_1 = "./data/heightfields/random_terrain.txt"
    climb_1 = "./data/heightfields/climb_1.txt"
    climb_2 = "./data/heightfields/climb_2.txt"
    plane_file_200 = "./data/heightfields/plane_200.txt"
    stairs = "./data/heightfields/stairs.txt"
    stairs_1 = "./data/heightfields/stairs_1.txt"
    collision_hills = "./data/heightfields/collision_wall_hills.txt"

    def __init__(self, maps=['plane'], dim=20, scale_factor=1):
        """Initialize a Maps object.

        Args:
            maps (str or list, optional): Type of map or list of map names. Defaults to 'plane'.
            dim (int, optional): Dimension of the map. Defaults to 20.
            scale_factor (int, optional): Scaling factor for the map. Defaults to 1.
        """
        self._generate(scale_factor)
        self.dim = self.name_2_np_arr[maps[0]].shape[0]
        self.standard_map_dim = (self.dim, self.dim)
        self.maps = maps
        if len(maps) == 0:
            self.map = self.plane
        elif len(maps) == 1:
            self.map = self.name_2_np_arr[maps[0]]
        else:
            self.map = self.multi_map_generator(maps)

    def multi_map_generator(self, maps):
        """Generate a map by combining multiple pre-defined maps.

        Args:
            maps (list): List of map names to combine.

        Returns:
            numpy.ndarray: A combined map generated from the input maps.
        """
        map_arr = np.zeros((self.standard_map_dim[1], self.standard_map_dim[0] * len(maps)))
        for i, map_id in enumerate(maps):
            map_arr[:, (i * self.dim):(i + 1) * self.dim] = self.name_2_np_arr[map_id]
        return map_arr

    def _generate(self, scale_factor):
        """Generate pre-defined height maps and store them as attributes.

        Args:
            scale_factor (int): Scaling factor for the maps.
        """
        map_files = {
            'calibration': self.calibration_file,
            'step': self.step_file,
            'step_1': self.step_1_file,
            'step_2': self.step_2_file,
            'step_3': self.step_3_file,
            'wall_1': self.wall_1_file,
            'wall_2': self.wall_2_file,
            'wall_3': self.wall_3_file,
            'wall_4': self.wall_4_file,
            'stairs': self.stairs_file,
            'plane': self.plane_file,
            'climb_1': self.climb_1,
            'climb_2': self.climb_2,
            'stair': self.stairs,
            'stair_1': self.stairs_1,
            'test': self.test_file,
            'feasibility': self.feasibility,
            'feasibility_1': self.feasibility_1,
            'random_terrain_1': self.random_terrain_1,
            'collision_hill': self.collision_hills
        }
        self.name_2_np_arr = {}
        for map_name, file_path in map_files.items():
            map_data = scale_map(heighmap_2_np_reader(file_path), scale_factor)
            self.name_2_np_arr[map_name] = map_data


class Height_Map_Generator(Maps):

    def __init__(self, dim=20, maps='plane', bool_map_search=False, scale_factor=1, randomize_env=False, random_shift_num=10, random_height_num=10):
        """Initialize a Height_Map_Generator object.

        Args:
            dim (int, optional): Dimension of the map. Defaults to 20.
            maps (str or list, optional): Type of map or list of map names. Defaults to 'plane'.
            bool_map_search (bool, optional): Whether to perform boolean map search. Defaults to False.
            scale_factor (int, optional): Scaling factor for the map. Defaults to 1.
            randomize_env (bool, optional): Whether to randomize the environment. Defaults to False.
            random_shift_num (int, optional): Number of random shifts to apply. Defaults to 500.
            random_height_num (int, optional): Number of random height adjustments to apply. Defaults to 10.

        Attributes:
            climb_map (bool): True if 'climb_1' or 'climb_2' is found in the list of maps; False otherwise.
            towr_map (numpy.ndarray): Transposed version of the map.
            towr_map_adjusted (numpy.ndarray): Adjusted transposed map.
            height_shift (float): Maximum height shift in the heightmap.
            num_rows (int): Number of rows in the map.
            num_cols (int): Number of columns in the map.
            resolution (float): Map resolution.
            multi_map_shift (int): Number of maps in the list.
            bool_map_search (bool): True if boolean map search is enabled; False otherwise.
            bool_map (numpy.ndarray): Boolean map resulting from pathfinding.
        """
        super(Height_Map_Generator, self).__init__(maps, dim, scale_factor)
        self.climb_map = self.climb_map_check(maps)
        self.towr_map = np.transpose(self.map)
        if randomize_env:
            self.towr_map = self.random_map_shift(self.towr_map, random_shift_num * scale_factor)
            self.map = self.random_map_shift(self.map, random_shift_num * scale_factor)
            self.towr_map = self.random_height_shift(self.towr_map, random_height_num)
            self.map = self.random_height_shift(self.map, random_height_num)
        self.towr_map_adjusted = self.towr_map_adjustment(np.transpose(self.map.copy()), shift_z=0.0, shift_down_num=0)
        self.create_height_file(HEIGHT_FIELD_OUT, self.map)
        self.create_height_file(TOWR_HEIGHTFIELD_OUT, self.towr_map_adjusted)
        self.height_shift = max_height(HEIGHT_FIELD_OUT) / 2.0
        self.num_rows = self.map.shape[0]
        self.num_cols = self.map.shape[1]
        self.resolution = 1 / (self.map.shape[0] / 2)
        self.multi_map_shift = len(self.maps)
        self.bool_map_search = bool_map_search
        if bool_map_search:
            save_height_grid_map(self.map)
            self.bool_map = PATH_MAP(self.map, multi_map_shift=self.multi_map_shift, scale=scale_factor).bool_map
            save_bool_map(self.bool_map)
        else:
            self.bool_map = np.zeros((self.map.shape[0], self.map.shape[1]))

    def create_height_file(self, file, h_data):
        """Create a height data file from a 2D array.

        Args:
            file (str): The name of the file to create.
            h_data (numpy.ndarray): The 2D array containing height data.

        """
        rows = len(h_data)
        with open(file, 'w') as f:
            for row_n, line in enumerate(h_data):
                s = ', '.join(str(value) for value in line)
                s += ','
                f.write(s)
                if row_n < rows - 1:
                    f.write('\n')

    def towr_map_adjustment(self, map, shift_z=0.00, shift_down_num = 0):
        """Adjust a given map by shifting non-zero values vertically.

        Args:
            map (numpy.ndarray): The input map as a NumPy array.
            shift_z (float, optional): The amount to vertically shift non-zero values.
                Defaults to 0.00.
            shift_down_num (int, optional): The number of rows to shift non-zero values
                downwards. Defaults to 0.

        Returns:
            numpy.ndarray: A modified map with adjusted non-zero values.
        """   
        def shift_rows_down(map, amount):
            rows, cols = map.shape
            shift_arr = np.zeros_like(map)

            for i in range(rows - 1):
                shift_arr[i + 1] = map[i]
            return shift_arr

        not_zero_mask = map != 0
        shift_values = map[not_zero_mask] + shift_z
        map[not_zero_mask] = shift_values
        map = shift_rows_down(map, shift_down_num)
        return map
    
    def climb_map_check(self, maps):
        """Check if a list of maps contains 'climb_1' or 'climb_2'.

        Args:
            maps (list): A list of map names to check.

        Returns:
            bool: True if 'climb_1' or 'climb_2' is found in the list; False otherwise.
        """
        for map in maps:
            if map == "climb_1" or map == "climb_2":
                return True
        return False

    def random_map_shift(self, map, shift=0):
        """Randomly shift a given map multiple times.
        Args:
            map (numpy.ndarray): The input map as a NumPy array.
            shift (int, optional): The number of times to apply the `shift_map`
                function to the map. Defaults to 0, meaning no shift.

        Returns:
            numpy.ndarray: A modified map with cumulative random shifts.
        """
        _map = map
        for i in range(shift):
            _map = self.shift_map(_map)
        return _map

    def shift_map(self, map):
        """Shift a given map in a random direction.

        Args:
            map (numpy.ndarray): The input map as a NumPy array.

        Returns:
            numpy.ndarray: A modified map with a random shift in the chosen direction.
        """
        directions = None
        if self.climb_map:
            directions = ['up', 'down']
        else:
            directions = ['left', 'right', 'up', 'down']
        direction = random.choice(directions)
        _map = np.copy(map)
        if direction == 'left':
            _map = np.roll(_map, shift=-1, axis=1)
        elif direction == 'up':
            _map = np.roll(_map, shift=-1, axis=0)
        elif direction == 'right':
            _map = np.roll(_map, shift=1, axis=1)
        elif direction == 'down':
            _map = np.roll(_map, shift=1, axis=0)
        return _map

    def random_height_shift(self, map, shift=0):
        """Randomly shift the height values in a given map multiple times.

        Args:
            map (numpy.ndarray): The heightmap as a NumPy array.
            shift (int, optional): The number of times to apply the `random_height`
                function to the heightmap. Defaults to 0.

        Returns:
            numpy.ndarray: A modified heightmap with cumulative random height shifts.
        """
        _map = map
        for i in range(shift):
            _map = self.random_height(_map)
        return _map

    def random_height(self, map, height_delta=0.01):
        """Randomly perturb the height values in a given map.

        Args:
            map (numpy.ndarray): The heightmap as a NumPy array.
            height_delta (float, optional): The maximum absolute height change allowed
                for each point. Defaults to 0.01.

        Returns:
            numpy.ndarray: A modified heightmap with randomly perturbed heights.
        """
        _map = np.copy(map)
        unique_h = np.unique(_map[_map != 0])
        for h in unique_h:
            delta_h = random.uniform(-height_delta, height_delta)
            n = random.choice((0, 1, 2))
            if n == 0:
                _map[_map == h] += delta_h
            elif n == 1:
                _map[_map == h] -= delta_h
            elif n == 2:
                _map[_map == h] += 0.0
        return _map

