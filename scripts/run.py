
import time
import os
import sys
import math

#third party
import pybullet as p
import pybullet_data
import yaml

#project
from SOLO12_SIM_CONTROL.robot import SOLO12

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"

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
    cfg = yaml.safe_load(open(config, 'r'))
    Pose = ([0,0,0.5], p.getQuaternionFromEuler([0,0,0]))
    robot = importRobot(URDF, Pose)
    ROBOT = SOLO12(py_client, robot, cfg)
    breakpoint()
    for i in range (10000):
        p.stepSimulation()
        time.sleep(1.0/10000.0)
    p.disconnect()