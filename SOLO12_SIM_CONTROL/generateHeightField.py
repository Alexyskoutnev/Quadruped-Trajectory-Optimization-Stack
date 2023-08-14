import time

import numpy as np

URDF = "./data/urdf/"
HEIGHT_FIELD = "heightmaps/staircase.txt"
HEIGHT_FIELD_FILE = "./data/heightmaps/staircase.txt"
HEIGHT_FIELD_OUT = "./data/heightmaps/staircase.out"

class RandomMaps():
    pass

class Maps():

    wall_1_file = "../data/heightmaps/wall_1.txt"
    wall_2_file = "../data/heightmaps/wall_2.txt"
    wall_3_file = "../data/heightmaps/wall_3.txt"
    stairs_file = "../data/heightmaps/staircase.txt"
    plane_file =  "../data/heightmaps/plane.txt"
    wall_1 = np.genfromtxt(wall_1_file, delimiter=',', dtype=np.float32)
    wall_2 = np.genfromtxt(wall_2_file, delimiter=',', dtype=np.float32)
    wall_3 = np.genfromtxt(wall_3_file, delimiter=',', dtype=np.float32)
    stairs = np.genfromtxt(stairs_file, delimiter=',', dtype=np.float32)
    plane = np.genfromtxt(plane_file, delimiter=',', dtype=np.float32)
    name_2_np_arr = {'plane' : plane, 'wall_1' : wall_1, 'wall_2' : wall_2, 'wall_3' : wall_3}

    def __init__(self, maps=['plane']):
        self.standard_map_dim = (20, 20)
        if len(maps) == 0:
            return self.plane
        elif len(maps) == 1:
            return self.name_2_np_arr[maps[0]]
        else:
            return self.multi_map_generator(maps)

    def multi_map_generator(self, maps):
        map_arr = np.zeros((self.standard_map_dim[0] * len(maps), self.standard_map_dim[1]))
        for map_id in maps:
            pass

class Height_Map_Generator(Maps):

    def __init__(self, length=20, maps='plane'):
        breakpoint()
        self.map = super(Height_Map_Generator).__init__(maps)

    def generate_map(self):
        pass


if __name__ == "__main__":
    height_map = Height_Map_Generator()
    breakpoint()