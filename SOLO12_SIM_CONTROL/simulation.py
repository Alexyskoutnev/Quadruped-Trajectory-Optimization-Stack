from SOLO12_SIM_CONTROL.generateHeightField import Height_Map_Generator, txt_2_np_reader

import time

import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation

URDF = "./data/urdf/"
# HEIGHT_FIELD = "heightmaps/staircase.txt"
HEIGHT_FIELD = "heightmaps/heightfield.txt"
HEIGHT_FIELD_TEST = "./data/heightmaps/heightfield_test.txt"
HEIGHT_FIELD_FILE = "./data/heightmaps/heightfield_test.txt"
# HEIGHT_FIELD = "heightmaps/walls.txt"
# HEIGHT_FIELD_FILE = "./data/heightmaps/staircase.txt"
# HEIGHT_FIELD_FILE = "./data/heightmaps/walls.txt"
# HEIGHT_FIELD_OUT = "./data/heightmaps/staircase.out"
HEIGHT_FIELD_OUT = "./data/heightmaps/heightfield.out"
# HEIGHT_FIELD_OUT = "./data/heightmaps/walls.out"

class Simulation(object):

    def __init__(self, cfg):
        self._wall = "./data/urdf/wall.urdf"
        self._stairs = "./data/urdf/stair.urdf"
        self._box = "./data/urdf/box.urdf"
        self.p = self.setup(cfg)

    def setup(self, cfg):
        py_client = None

        test = False

        if test:
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10.0)
            self.setup_terrain()

        elif cfg['enviroment'] == "custom":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10.0)
            height_map = Height_Map_Generator(maps=cfg['map_id'])
            height_shift = height_map.height_shift
            tiles = len(cfg['map_id'])
            num_rows, num_cols = height_map.num_rows, height_map.num_cols
            terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,.1], heightfieldTextureScaling=64, heightfieldData=height_map.map.flatten().tolist(), numHeightfieldRows=num_cols, numHeightfieldColumns=num_rows)
            terrain  = p.createMultiBody(0, terrainShape)
            p.resetBasePositionAndOrientation(terrain,[1.0 * (tiles - 1),.0,height_shift+0.001], [0,0,0,1.0])
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
            p.changeVisualShape(terrain, -1, rgbaColor=[0.0,1.0,1.0,1])

        elif cfg['enviroment'] == "testing":
            py_client = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10)
            p.loadURDF("plane.urdf")

        elif cfg['enviroment'] == "plane":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10.0)
            p.loadURDF("plane.urdf")

        elif cfg['enviroment'] == "towr_no_gui":
            py_client = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10)
            p.loadURDF("plane.urdf")

        elif cfg['enviroment'] == "height_terrian":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10)
            p.loadURDF("plane.urdf")
            rot_stair1 =  Rotation.from_euler('xyz', [0, 0, 180], degrees=True)
            rot_wall = Rotation.from_euler('xyz', [0, 0, 0], degrees=True)
            stair1_pos, stair1_rot = [2, 0, 0], rot_stair1.as_quat()
            wall1_pos, wall1_rot = [2, -1.0, 0.4], rot_wall.as_quat()
            wall2_pos, wall2_rot = [2, 1.0,0.4], rot_wall.as_quat()
            p.loadURDF(self._stairs, basePosition = stair1_pos, baseOrientation = stair1_rot, useFixedBase = 1)
            p.loadURDF(self._wall, basePosition = wall2_pos, baseOrientation = wall1_rot, useFixedBase = 1)

        elif cfg['enviroment'] == "towr_track_no_contact":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,0)
            p.loadURDF("plane.urdf")

        elif cfg['enviroment'] == "towr_track_no_contact_no_gui":
            py_client = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,0)
            p.loadURDF("plane.urdf")
            
        elif cfg['enviroment'] == "fixed_in_air":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,0)
            p.loadURDF("plane.urdf")

        return py_client