
import time
import os
import sys
import math

#third party
import pybullet as p
import pybullet_data
import yaml
import numpy as np

#project
from SOLO12_SIM_CONTROL.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
r = 0.1


def sampleTraj(robot, r, N=100):
    traj_dic = {}
    traj = list()
    theta = np.linspace(0, 2*np.pi, N)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = 0.2 * np.sin(theta)
    config = robot.get_endeffector_pose()
    for link in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'):
        tf_mtx = transformation_mtx(config[link]['linkWorldPosition'], config[link]['linkWorldOrientation'])
        for val_x, val_y, val_z in zip(x, y, z):
            vec = np.concatenate((np.array([val_x]), np.array([val_y]), np.array([val_z]), np.ones(1)))
            tf_vec = tf_mtx @ vec
            traj.append(tf_vec[:3])
        traj_dic[link] = traj
        traj = list()
    return traj_dic

def setup_enviroment():
    py_client = p.connect(p.GUI)
    # py_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,0)
    p.setTimeStep(0.001) 
    return py_client 

def importRobot(file=URDF, POSE=([0,0,1], (0.0,0.0,0.0,1.0))):
    planeId = p.loadURDF("plane.urdf")
    solo12 = p.loadURDF(URDF, *POSE)
    return solo12

if __name__ == "__main__":
    py_client = setup_enviroment()
    cfg = yaml.safe_load(open(config, 'r'))
    Pose = ([0,0,0.5], p.getQuaternionFromEuler([0,0,1]))
    robot = importRobot(URDF, Pose)
    ROBOT = SOLO12(py_client, robot, cfg)
    traj = sampleTraj(ROBOT, r) 
    init_phase = False
    for key, value in traj.items():
        traj[key] = value * 100
    for i in range (10000):
        if init_phase:
            p.stepSimulation()
            time.sleep(1.0/100000.0)
        else:
            # breakpoint()
            joints_FL = ROBOT.invKinematics(traj['FL_FOOT'][i], ROBOT.EE_index['FL_FOOT'])
            ROBOT.setJointControl(ROBOT.jointidx['FL'], p.POSITION_CONTROL, joints_FL[0:3])
            joints_FR = ROBOT.invKinematics(traj['FR_FOOT'][i], ROBOT.EE_index['FR_FOOT'])
            ROBOT.setJointControl(ROBOT.jointidx['FR'], p.POSITION_CONTROL, joints_FR[3:6])
            joints_HL = ROBOT.invKinematics(traj['HL_FOOT'][i], ROBOT.EE_index['HL_FOOT'])
            ROBOT.setJointControl(ROBOT.jointidx['BL'], p.POSITION_CONTROL, joints_HL[6:9])
            joints_HR = ROBOT.invKinematics(traj['HR_FOOT'][i], ROBOT.EE_index['HR_FOOT'])
            ROBOT.setJointControl(ROBOT.jointidx['BR'], p.POSITION_CONTROL, joints_HR[9:12])
            p.stepSimulation()
            time.sleep(1.0/100000.0)
    p.disconnect()