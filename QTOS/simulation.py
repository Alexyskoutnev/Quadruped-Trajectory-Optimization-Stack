from QTOS.generateHeightField import Height_Map_Generator

import time

import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation

HEIGHT_FIELD = "heightmaps/heightfield.txt"
HEIGHT_FIELD_TEST = "./data/heightmaps/heightfield_test.txt"
HEIGHT_FIELD_FILE = "./data/heightmaps/heightfield_test.txt"
HEIGHT_FIELD_OUT = "./data/heightmaps/heightfield.out"

class Simulation(object):
    """
    This class manages the simulation environment in PyBullet for various scenarios.

    Args:
        cfg (dict): A dictionary containing configuration settings for the simulation environment.
    """

    def __init__(self, cfg):
        """
        Initialize the Simulation object based on the provided configuration.

        Args:
            cfg (dict): A dictionary containing configuration settings for the simulation environment.
        """
        self._wall = "./data/urdf/wall.urdf"
        self._stairs = "./data/urdf/stair.urdf"
        self._box = "./data/urdf/box.urdf"
        self.height_map_generator = None
        self.height_map = None
        self.bool_map = None
        self.num_tiles = 0
        self.p = self.setup(cfg)

    def setup(self, cfg):
        """
        Set up the PyBullet simulation environment based on the provided configuration.

        Args:
            cfg (dict): A dictionary containing configuration settings for the simulation environment.

        Returns:
            pybullet.Client: The PyBullet client for the simulation.
        """

        py_client = None

        if cfg['enviroment'] == "custom":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10.0)
            height_map_generator = Height_Map_Generator(maps=cfg['map_id'], bool_map_search=cfg['bool_map_search'], scale_factor=cfg['mesh_scale'], randomize_env=cfg['random_env'])
            height_shift = height_map_generator.height_shift
            tiles = len(cfg['map_id'])
            self.bool_map = height_map_generator.bool_map
            self.num_tiles = tiles
            self.height_map = height_map_generator.map
            self.resolution_xy = height_map_generator.resolution
            cfg['resolution'] =  height_map_generator.resolution
            num_rows, num_cols = height_map_generator.num_rows, height_map_generator.num_cols
            terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[self.resolution_xy,self.resolution_xy, 1.0], heightfieldTextureScaling=128, heightfieldData=height_map_generator.map.flatten().tolist(), numHeightfieldRows=num_cols, numHeightfieldColumns=num_rows)
            terrain  = p.createMultiBody(0, terrainShape)
            p.resetBasePositionAndOrientation(terrain,[1.0 * (tiles - 1), .0, height_shift+0.0001], [0,0,0,1.0])
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
            p.changeVisualShape(terrain, -1, rgbaColor=[0.0,1.0,1.0,1])
            p.changeDynamics(1, -1, lateralFriction=cfg['friction'])
            p.changeDynamics(terrain, -1, lateralFriction=cfg['friction'])
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        elif cfg['enviroment'] == 'custom_test':
            py_client = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10.0)
            height_map_generator = Height_Map_Generator(maps=cfg['map_id'], bool_map_search=cfg['bool_map_search'], scale_factor=cfg['mesh_scale'], randomize_env=cfg['random_env'])
            height_shift = height_map_generator.height_shift
            tiles = len(cfg['map_id'])
            self.bool_map = height_map_generator.bool_map
            self.num_tiles = tiles
            self.height_map = height_map_generator.map
            self.resolution_xy = height_map_generator.resolution
            cfg['resolution'] =  height_map_generator.resolution
            num_rows, num_cols = height_map_generator.num_rows, height_map_generator.num_cols
            terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[self.resolution_xy,self.resolution_xy, 1.0], heightfieldTextureScaling=128, heightfieldData=height_map_generator.map.flatten().tolist(), numHeightfieldRows=num_cols, numHeightfieldColumns=num_rows)
            terrain  = p.createMultiBody(0, terrainShape)
            p.resetBasePositionAndOrientation(terrain,[1.0 * (tiles - 1), .0, height_shift+0.0001], [0,0,0,1.0])
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
            p.changeVisualShape(terrain, -1, rgbaColor=[0.0,1.0,1.0,1])
            p.changeDynamics(1, -1, lateralFriction=cfg['friction'])
            p.changeDynamics(terrain, -1, lateralFriction=cfg['friction'])
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

        return py_client