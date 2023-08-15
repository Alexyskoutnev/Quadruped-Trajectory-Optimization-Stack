import time

import numpy as np

HEIGHT_FIELD = "heightmaps/test_heightfield.txt"
HEIGHT_FIELD_TXT = "./data/heightmaps/test_heightfield.txt"
HEIGHT_FIELD_OUT = "./data/heightmaps/test_heightfield_towr.txt"

def is_numeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def txt_2_np_reader(file, delimiter=','):
    data = []
    with open(file, 'r') as f:
        reader = f.readlines()
        for row in reader:
            _row = row.strip().split(delimiter)
            _row = [float(x) for x in _row if is_numeric(x)]
            data.append(_row)
    return np.array(data)

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

class RandomMaps(object):
    pass

class Maps(object):

    wall_1_file = "./data/heightmaps/wall_1.txt"
    wall_2_file = "./data/heightmaps/wall_2.txt"
    wall_3_file = "./data/heightmaps/wall_3.txt"
    test_file = "./data/heightmaps/heightfield_test.txt"
    stairs_file = "./data/heightmaps/staircase.txt"
    plane_file =  "./data/heightmaps/plane.txt"
    wall_1 = txt_2_np_reader(wall_1_file)
    wall_2 = txt_2_np_reader(wall_2_file)
    wall_3 = txt_2_np_reader(wall_3_file)
    stairs = txt_2_np_reader(stairs_file)
    plane = txt_2_np_reader(plane_file)
    test = txt_2_np_reader(test_file)
    name_2_np_arr = {'plane' : plane, 'wall_1' : wall_1, 'wall_2' : wall_2, 'wall_3' : wall_3,
                     'test': test}

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
        self.create_height_file(HEIGHT_FIELD_TXT, self.map)
        self.create_height_file(HEIGHT_FIELD_OUT, self.map)
        self.height_shift = max_height(HEIGHT_FIELD_TXT) / 20.0
        self.f_name = HEIGHT_FIELD
        self.num_rows = self.map.shape[0]
        self.num_cols = self.map.shape[1]


    def create_height_file(self, file, h_data):
            rows = len(h_data)
            with open(file, 'w') as f:
                for row_n, line in enumerate(h_data):
                    s = ', '.join(str(value) for value in line)
                    s += ','
                    f.write(s)
                    if row_n < rows - 1:
                        f.write('\n')