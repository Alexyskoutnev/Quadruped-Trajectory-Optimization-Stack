import pybullet as p
import time
import pybullet_data

from scipy.spatial.transform import Rotation
import yaml

from SOLO12_SIM_CONTROL.robot import SOLO12


config = "./data/config/solo12.yml"
URDF = "./data/urdf/solo12.urdf"

def setup_enviroment():
    py_client = p.connect(p.GUI)
    # py_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)
    p.setGravity(0,0,-9.81)
    p.setTimeStep(0.001) 
    return py_client 


cfg = yaml.safe_load(open(config, 'r'))
basePosition = [0, 0, 0]
baseOrientation = [0, 0, 0, 1]
rot_stair1 = Rotation.from_euler('xyz', [0, 0, 180], degrees=True)
stair1_pos = [4, 0, 0]
stair1_ang = rot_stair1.as_quat()
py_client = setup_enviroment()

p.loadURDF("./data/urdf/stair.urdf", basePosition = stair1_pos, baseOrientation = stair1_ang, useFixedBase = 1)
p.loadURDF("./data/urdf/wall.urdf", basePosition = [2, -1.0, 0.4], baseOrientation = [0,0,0,1], useFixedBase = 1)
p.loadURDF("./data/urdf/wall.urdf", basePosition = [2, 1.0,0.4], baseOrientation = [0,0,0,1], useFixedBase = 1)

ROBOT = SOLO12(py_client, URDF, cfg)


p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)
while (1):
  keys = p.getKeyboardEvents()
  print(keys)

  time.sleep(0.01)