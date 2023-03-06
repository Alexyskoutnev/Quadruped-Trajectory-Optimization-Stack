
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
from SOLO12_SIM_CONTROL.gaitPlanner import Gait
from SOLO12_SIM_CONTROL.pybulletInterface import PybulletInterface

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
    p.setGravity(0,0,-9.81)
    p.setTimeStep(0.001) 
    # p.setTimeStep(0.002)
    return py_client 

def importRobot(file=URDF, POSE=([0,0,1], (0.0,0.0,0.0,1.0))):
    
    solo12 = p.loadURDF(URDF, *POSE)
    return solo12

if __name__ == "__main__":
    py_client = setup_enviroment()
    pybullet_interface = PybulletInterface()
    cfg = yaml.safe_load(open(config, 'r'))
    Pose = ([0,0,0.5], p.getQuaternionFromEuler([0,0,1]))
    planeId = p.loadURDF("plane.urdf")
    # robot = importRobot(URDF, Pose)
    ROBOT = SOLO12(py_client, URDF, cfg)
    gait = Gait(ROBOT)
    traj = sampleTraj(ROBOT, r) 
    init_phase = False
    trot_phase = True

    pos, angle, velocity, angle_velocity , angle,  stepPeriod = pybullet_interface.robostates(ROBOT.robot)

    # angle = 0
    # angle_velocity = 0.0
    # velocity = 1.0
    offsets = np.array([0.5, 0.0, 0.0, 0.5])
    trot_2_stance_ratio = 0.5
    # stepPeriod = 1.0



    
    cmd = np.zeros((12, 1))
    for i in range (10000):
        pos, angle, velocity, angle_velocity , angle,  stepPeriod = pybullet_interface.robostates(ROBOT.robot)
        if init_phase:
            jointTorques = ROBOT.default_stance_control(cmd, p.TORQUE_CONTROL)
            p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=jointTorques)
            p.stepSimulation()
        elif trot_phase:
            # print("v", velocity, " ang", angle_velocity)
            gait_traj = gait.runTrajectory(velocity, angle, angle_velocity, offsets, stepPeriod, trot_2_stance_ratio)
            # print(f"{i}: {gait_traj}")
            joints_FL = ROBOT.invKinematics(gait_traj['FL_FOOT'], ROBOT.EE_index['FL_FOOT'])
            ROBOT.setJointControl(ROBOT.jointidx['FL'], p.POSITION_CONTROL, joints_FL[0:3])
            joints_FR = ROBOT.invKinematics(gait_traj['FR_FOOT'], ROBOT.EE_index['FR_FOOT'])
            ROBOT.setJointControl(ROBOT.jointidx['FR'], p.POSITION_CONTROL, joints_FR[3:6])
            joints_HL = ROBOT.invKinematics(gait_traj['HL_FOOT'], ROBOT.EE_index['HL_FOOT'])
            ROBOT.setJointControl(ROBOT.jointidx['BL'], p.POSITION_CONTROL, joints_HL[6:9])
            joints_HR = ROBOT.invKinematics(gait_traj['HR_FOOT'], ROBOT.EE_index['HR_FOOT'])
            ROBOT.setJointControl(ROBOT.jointidx['BR'], p.POSITION_CONTROL, joints_HR[9:12])
            p.stepSimulation()
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
            # time.sleep(1.0/10000.0)
        ROBOT.time_step += 1
    p.disconnect()