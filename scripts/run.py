import pybullet as p
import time
import pybullet_data

import math


URDF = "../data/urdf/solo12.urdf"

def setup_enviroment():
    py_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-10)
    p.setTimeStep(0.001) 
    return py_client 

def importRobot(file=URDF, POSE=([0,0,1], (0.0,0.0,0.0,1.0))):
    planeId = p.loadURDF("plane.urdf")
    solo12 = p.loadURDF(URDF, *POSE)
    return solo12

if __name__ == "__main__":
    py_client = setup_enviroment()
    Pose = ([0,0,0.5], p.getQuaternionFromEuler([0,0,0]))
    robot = importRobot(URDF, Pose)
    for i in range (10000):
        p.stepSimulation()
        time.sleep(1.0/10000.0)
    p.disconnect()