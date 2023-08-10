import time

import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation

URDF = "./data/urdf/"

class Simulation(object):

    def __init__(self, simulation_type, timestep=None, setup_terrain=False):
        self._wall = "./data/urdf/wall.urdf"
        self._stairs = "./data/urdf/stair.urdf"
        self._box = "./data/urdf/box.urdf"
        self.timestep = timestep
        self.p = self.setup(sim_config=simulation_type)
        if setup_terrain:
            self.setup_terrain()

    def setup(self, sim_config = "height_terrain"):
        py_client = None

        if sim_config == "testing":
            py_client = p.connect(p.DIRECT)
            # py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10)
            p.loadURDF("plane.urdf")

        elif sim_config == "plane":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10.0)
            p.loadURDF("plane.urdf")


        elif sim_config == "towr_no_gui":
            py_client = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10)
            p.loadURDF("plane.urdf")


        elif sim_config == "plane_record":
            py_client = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10)
            p.loadURDF("plane.urdf")

        elif sim_config == "height_terrian":
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
            # p.loadURDF(self._wall, basePosition = wall1_pos, baseOrientation = wall1_rot, useFixedBase = 1)
            p.loadURDF(self._wall, basePosition = wall2_pos, baseOrientation = wall1_rot, useFixedBase = 1)

        elif sim_config == "obstacles_stairs":
            pass

        elif sim_config == "towr_track_no_contact":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,0)
            p.loadURDF("plane.urdf")

        elif sim_config == "towr_track_no_contact_box":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,0)
            p.loadURDF("plane.urdf")
            box1_pos, box1_rot = [0.35, 0, 0.05], Rotation.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()
            p.loadURDF(self._box, basePosition = box1_pos, baseOrientation = box1_rot, useFixedBase = 1)

        elif sim_config == "towr_track_no_contact_no_gui":
            py_client = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,0)
            p.loadURDF("plane.urdf")
            
        elif sim_config == "fixed_in_air":
            py_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,0)
            p.loadURDF("plane.urdf")

        return py_client

    def setup_terrain(self):
        reading_from_file = True
    
        if not reading_from_file:
            # fill manually
            numHeightfieldRows = 100
            numHeightfieldColumns = 100
            heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns 
            for j in range (numHeightfieldColumns - 1):
                for i in range (numHeightfieldRows - 1):
                    if j > int(numHeightfieldColumns/2) and j < int(numHeightfieldColumns*3/4) and i > int(numHeightfieldRows/2) and i < int(numHeightfieldRows*3/4):
                        height = .5
                    else:
                        height = .0
                    heightfieldData[i+j*numHeightfieldRows]=height
                    heightfieldData[i+1+j*numHeightfieldRows]=height
                    heightfieldData[i+(j+1)*numHeightfieldRows]=height
                    heightfieldData[i+1+(j+1)*numHeightfieldRows]=height
            
            heightfield_data = open("data/heightmaps/generated_heightfield.txt","w")

            for j in range (numHeightfieldColumns):
                for i in range (numHeightfieldRows):
                    heightfield_data.write('{}, '.format(heightfieldData[i+j*numHeightfieldRows]))
                heightfield_data.write("\n")
     
            terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.2,.2,2.0], heightfieldTextureScaling=(numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
            terrain  = p.createMultiBody(0, terrainShape)
            p.resetBasePositionAndOrientation(terrain,[0.5,-1,0], [0,0,0,1])

        if reading_from_file:
            # read from file
            terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,1],fileName = "heightmaps/heightfield.txt", heightfieldTextureScaling=64)
            terrain  = p.createMultiBody(0, terrainShape)
            p.resetBasePositionAndOrientation(terrain,[2,0,.2], [0,0,0,1])
        
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        p.changeVisualShape(terrain, -1, rgbaColor=[1,1,1,1])