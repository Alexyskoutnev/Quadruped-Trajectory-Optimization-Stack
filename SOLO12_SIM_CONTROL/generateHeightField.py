from SOLO12_SIM_CONTROL.utils import is_numeric

import time
from collections import namedtuple
import multiprocessing
import subprocess
import shlex

import numpy as np

np.set_printoptions(threshold=np.inf)

HEIGHT_FIELD_OUT = "./data/heightfields/heightfield.txt"
TOWR_HEIGHTFIELD_OUT = "./data/heightfields/from_pybullet/towr_heightfield.txt"
NUM_PROCESSES = 4

scripts =  {'run': 'docker exec <id> ./main',
            'info': 'docker ps -f ancestor=towr'}

_flags = ['-g', '-s', '-s_ang', '-s_vel', '-n', '-e1', '-e2', '-e3', '-e4', '-t']

def strip(x):
    st = " "
    for s in x:
        if s == "[" or s == "]":
            st += ''
        else:
            st += s
    return st

def DockerInfo():
    p = subprocess.run([scripts['info']], shell=True, capture_output=True, text=True)
    output = p.stdout.replace('\n', ' ')
    dockerid, idx = output.split(), output.split().index('towr') - 1
    return dockerid[idx]

def parse_scripts(scripts_dic, docker_id):
    for script_name, script in scripts_dic.items():
        scripts_dic[script_name] = script.replace("<id>", docker_id)
    return scripts_dic

def cmd_args(args):
    """Commandline parser to run python subprocesses correctly

    Args:
        args (dict): user + config inputs for simulation and solver

    Return:
        _cmd : parsed command line string that can be ran as a excutable
    """

    def _bracket_rm(s):
        return s.replace("[", "").replace("]", "")

    def _remove_comma(s):
        return s.replace(",", "")
    
    def _filter(s):
        return _bracket_rm(_remove_comma(str(s)))

    _cmd = ""

    for key, value in args.items():
        if key in _flags and value:
            _cmd += key + " " + _filter(value)
            _cmd += " "

    return _cmd


class Map_2_Idx(object):

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
    data = []
    with open(file, 'r') as f:
        reader = f.readlines()
        for row in reader:
            _row = row.strip().split(delimiter)
            _row = [float(x) for x in _row if is_numeric(x)]
            data.append(_row)
    return np.transpose(np.array(data))

def scale_values(file, scale=1.0):
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
    
    def __init__(self, map):
        self.map = map
        self.docker_id = DockerInfo()
        self.scripts = parse_scripts(scripts, self.docker_id)
        self.bool_map = np.ones((map.shape[0], map.shape[1]), dtype=int)
        self.data_queue = multiprocessing.Queue()
        self.probe_map(map)
        self.run()
        
        
    def probe_map(self, map, mesh_resolution_meters = 0.1):
        step_x = mesh_resolution_meters
        step_y = mesh_resolution_meters
        x_start = - mesh_resolution_meters * (map.shape[1] / 2) - mesh_resolution_meters / 2
        y_start = - mesh_resolution_meters * (map.shape[1] / 2) - mesh_resolution_meters / 2
        x_goal =  - mesh_resolution_meters * (map.shape[1] / 2) + mesh_resolution_meters / 2
        y_goal = - mesh_resolution_meters * (map.shape[1] / 2) - mesh_resolution_meters / 2
        _x_start, _y_start = x_start, y_start
        _x_goal, _y_goal = x_goal, y_goal
        for x in range(map.shape[0] - 1):
                _x_start += step_x
                _x_goal += step_x
                _y_start, _y_goal = y_start, y_goal
                for y in range(map.shape[1]):
                    _y_start += step_y
                    _y_goal += step_y
                    _x_start, _y_start = round(_x_start, 2), round(_y_start, 2)
                    _x_goal, _y_goal = round(_x_goal, 2), round(_y_goal, 2)
                    data = Map_2_Idx(map_coords_start=(_x_start, _y_start), map_coords_goal=(_x_goal, _y_goal),
                                     map_idx_start=(x, y), map_idx_goal=(x+1, y))
                    self.data_queue.put(data)

    def run(self):
        processes = []
        for i in range(NUM_PROCESSES):
            process = multiprocessing.Process(target=self.worker_f, args=(self.map, self.data_queue))
            processes.append(process)
            process.start()
        for process in processes:
            process.join()

    def worker_f(self, map, queue):

        def state_config(args, start_pt, goal_pt):
            args['-s'] = [start_pt[0], start_pt[1], 0.24]
            args['-e1'] = [0.20590930477664196, 0.14927536747689948, 0.0]
            args['-e2'] = [0.2059042161427424, -0.14926921805769638, 0.0]
            args['-e3'] = [-0.20589422629511542, 0.14933201572367907, 0.0]
            args['-e4'] = [-0.2348440184502048, -0.17033609357109808, 0.0]
            args['-s_ang'] = [0, 0, 0]
            args['-g'] = [goal_pt[0], goal_pt[1], 0.24]
            args['-n'] = 't'

        while not queue.empty():
            args = {}
            data = queue.get()
            start_pt = data.map_coords_start
            goal_pt = data.map_coords_goal
            start_idx = data.map_idx_start
            goal_idx = data.map_idx_goal
            state_config(args, start_pt, goal_pt)
            TOWR_SCRIPT = shlex.split(self.scripts['run'] + " " + cmd_args(args))
            p_status = subprocess.run(TOWR_SCRIPT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if p_status.returncode == 0:
                print(f"FOUND A SOLUTION [{start_idx}] -> [{goal_idx}]")
                map[start_idx] = 0
                map[goal_idx] = 0
            else:
                print(f"FAIL TO FIND A SOLUTION [{start_idx}] -> [{goal_idx}]")
                map[goal_idx] = 1
                map[start_idx] = 1
            # print(map)

class RandomMaps(object):
    pass

class Maps(object):

    calibration_file = "./data/heightfields/calibration.txt"
    step_file = "./data/heightfields/step.txt"
    step_1_file = "./data/heightfields/step_1.txt"
    step_2_file = "./data/heightfields/step_2.txt"
    step_3_file = "./data/heightfields/step_3.txt"
    wall_1_file = "./data/heightfields/wall_1.txt"
    wall_2_file = "./data/heightfields/wall_2.txt"
    wall_3_file = "./data/heightfields/wall_3.txt"
    test_file = "./data/heightfields/heightfield_test.txt"
    stairs_file = "./data/heightfields/staircase.txt"
    plane_file =  "./data/heightfields/plane.txt"
    calibration = heighmap_2_np_reader(calibration_file)
    step = heighmap_2_np_reader(step_file)
    step_1 = heighmap_2_np_reader(step_1_file)
    step_2 = heighmap_2_np_reader(step_2_file)
    step_3 = heighmap_2_np_reader(step_3_file)
    wall_1 = heighmap_2_np_reader(wall_1_file)
    wall_2 = heighmap_2_np_reader(wall_2_file)
    wall_3 = heighmap_2_np_reader(wall_3_file)
    stairs = heighmap_2_np_reader(stairs_file)
    plane = heighmap_2_np_reader(plane_file)
    test = heighmap_2_np_reader(test_file)
    name_2_np_arr = {'step_1': step_1, 'step_2': step_2, 'step_3': step_3, 'calibration' : calibration, 'step' : step, 'plane' : plane, 'wall_1' : wall_1, 'wall_2' : wall_2, 'wall_3' : wall_3,
                     'test': test, 'stairs': stairs}

    def __init__(self, maps=['plane'], dim=20):
        self.dim = 20
        self.standard_map_dim = (self.dim, self.dim)
        if len(maps) == 0:
            self.map = self.plane
        elif len(maps) == 1:
            self.map = self.name_2_np_arr[maps[0]]
        else:
            self.map = self.multi_map_generator(maps)

    def multi_map_generator(self, maps):
        # map_arr = np.zeros((self.standard_map_dim[0] * len(maps), self.standard_map_dim[1]))
        map_arr = np.zeros((self.standard_map_dim[1], self.standard_map_dim[0] * len(maps)))
        for i, map_id in enumerate(maps):
            # map_arr[(i * self.dim):(i + 1) * self.dim, :] = self.name_2_np_arr[map_id]
            map_arr[:, (i * self.dim):(i + 1) * self.dim] = self.name_2_np_arr[map_id]
        return map_arr

class Height_Map_Generator(Maps):

    def __init__(self, dim=20, maps='plane'):
        super(Height_Map_Generator, self).__init__(maps, dim)
        self.towr_map =  np.transpose(self.map)
        self.create_height_file(HEIGHT_FIELD_OUT, self.map)
        self.create_height_file(TOWR_HEIGHTFIELD_OUT, self.towr_map)
        self.height_shift = max_height(HEIGHT_FIELD_OUT) / 2.0
        self.num_rows = self.map.shape[0]
        self.num_cols = self.map.shape[1]
        self.bool_map = PATH_MAP(self.towr_map)

    def create_height_file(self, file, h_data):
            rows = len(h_data)
            with open(file, 'w') as f:
                for row_n, line in enumerate(h_data):
                    s = ', '.join(str(value) for value in line)
                    s += ','
                    f.write(s)
                    if row_n < rows - 1:
                        f.write('\n')