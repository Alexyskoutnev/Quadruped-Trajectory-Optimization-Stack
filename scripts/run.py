
import time
import os
import sys
import math
import csv

#third party
import pybullet as p
import pybullet_data
import yaml
import numpy as np

#project
from SOLO12_SIM_CONTROL.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv, towr_transform
from SOLO12_SIM_CONTROL.gaitPlanner import Gait
from SOLO12_SIM_CONTROL.pybulletInterface import PybulletInterface
from SOLO12_SIM_CONTROL.simulation import Simulation

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
config_sim = "./data/config/simulation.yml"
cfg = yaml.safe_load(open(config, 'r'))
sim_cfg = yaml.safe_load(open(config_sim, 'r'))
TOWR = "./data/traj/towr.csv"
HZ = sim_cfg['HZ']


def setup_enviroment():
    py_client = p.connect(p.GUI)
    # py_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    # p.setTimeStep(0.001) 
    return py_client 

def importRobot(file=URDF, POSE=([0,0,1], (0.0,0.0,0.0,1.0))):
    solo12 = p.loadURDF(URDF, *POSE)
    return solo12

def simulation():
    type = "plane"
    sim = Simulation(type)
    py_client = sim.setup(type)
    pybullet_interface = PybulletInterface()
    planeId = p.loadURDF("plane.urdf")
    ROBOT = SOLO12(URDF, cfg, fixed=False)
    gait = Gait(ROBOT)
    init_phase = False
    trot_phase = True

    if sim_cfg['mode'] == "towr":
        csv_file = open(TOWR, 'r', newline='')
        reader = csv.reader(csv_file, delimiter=',')
        NUM_TIME_STEPS = sum(1 for row in reader)
        csv_file = open(TOWR, 'r', newline='')
        reader = csv.reader(csv_file, delimiter=',')
        _t = np.linspace(0, NUM_TIME_STEPS/HZ, NUM_TIME_STEPS)

    pos, angle, velocity, angle_velocity , angle,  stepPeriod = pybullet_interface.robostates(ROBOT.robot)
    offsets = np.array([0.5, 0.0, 0.0, 0.5])
    trot_2_stance_ratio = 0.5
    cmd = np.zeros((12, 1))
    for i in range (10000):
        if sim_cfg['mode'] == "bezier":
            pos, angle, velocity, angle_velocity , angle,  stepPeriod = pybullet_interface.robostates(ROBOT.robot)
            if init_phase:
                jointTorques, newCmd = ROBOT.default_stance_control(cmd, p.TORQUE_CONTROL)
                p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=jointTorques)
                p.stepSimulation()
            elif trot_phase:
                gait_traj, newCmd = gait.runTrajectory(velocity, angle, angle_velocity, offsets, stepPeriod, trot_2_stance_ratio)
                joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(gait_traj['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_ang_FL[0:3])
                joints_ang_FR, joints_vel_FR, joints_toq_FR = ROBOT.control(gait_traj['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joints_ang_FR[3:6])
                joints_ang_HL, joints_vel_HL, joints_toq_HL = ROBOT.control(gait_traj['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joints_ang_HL[6:9])
                joints_ang_HR, joints_vel_HR, joints_toq_HR = ROBOT.control(gait_traj['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joints_ang_HR[9:12])
            p.stepSimulation()
            ROBOT.time_step += 1
        elif sim_cfg['mode'] == "towr":
            with open(TOWR, 'r', newline='') as csv_file:
                try: 
                    EE_POSE = np.array([float(x) for x in next(reader)])
                except StopIteration:
                    break
                print("EE_POSE -> ", EE_POSE)
                towr = {"COM": EE_POSE[0:3], "FL_FOOT": {'P' : EE_POSE[3:6], 'D': np.zeros(3)}, "FR_FOOT": {'P': EE_POSE[6:9], 'D': np.zeros(3)}, 
                        "HL_FOOT": {'P': EE_POSE[9:12], 'D' : np.zeros(3)}, "HR_FOOT": {'P': EE_POSE[12:15], 'D': np.zeros(3)}}
                towr = towr_transform(towr)
                joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(towr['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_ang_FL[0:3])
                joints_ang_FR, joints_vel_FR, joints_toq_FR = ROBOT.control(towr['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joints_ang_FR[3:6])
                joints_ang_HL, joints_vel_HL, joints_toq_HL = ROBOT.control(towr['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joints_ang_HL[6:9])
                joints_ang_HR, joints_vel_HR, joints_toq_HR = ROBOT.control(towr['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
                ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joints_ang_HR[9:12])
            p.stepSimulation()
            ROBOT.time_step += 1
    p.disconnect()

# if __name__ == "__main__":
#     type = "plane"
#     sim = Simulation(type)
#     py_client = sim.setup(type)
#     pybullet_interface = PybulletInterface()
#     planeId = p.loadURDF("plane.urdf")
#     ROBOT = SOLO12(URDF, cfg, fixed=False)
#     gait = Gait(ROBOT)
#     init_phase = False
#     trot_phase = True

#     if sim_cfg['mode'] == "towr":
#         csv_file = open(TOWR, 'r', newline='')
#         reader = csv.reader(csv_file, delimiter=',')
#         NUM_TIME_STEPS = sum(1 for row in reader)
#         csv_file = open(TOWR, 'r', newline='')
#         reader = csv.reader(csv_file, delimiter=',')
#         _t = np.linspace(0, NUM_TIME_STEPS/HZ, NUM_TIME_STEPS)

#     pos, angle, velocity, angle_velocity , angle,  stepPeriod = pybullet_interface.robostates(ROBOT.robot)
#     offsets = np.array([0.5, 0.0, 0.0, 0.5])
#     trot_2_stance_ratio = 0.5
#     cmd = np.zeros((12, 1))
#     for i in range (10000):
#         if sim_cfg['mode'] == "bezier":
#             pos, angle, velocity, angle_velocity , angle,  stepPeriod = pybullet_interface.robostates(ROBOT.robot)
#             if init_phase:
#                 jointTorques, newCmd = ROBOT.default_stance_control(cmd, p.TORQUE_CONTROL)
#                 p.setJointMotorControlArray(ROBOT.robot, ROBOT.jointidx['idx'], controlMode=p.TORQUE_CONTROL, forces=jointTorques)
#                 p.stepSimulation()
#             elif trot_phase:
#                 gait_traj, newCmd = gait.runTrajectory(velocity, angle, angle_velocity, offsets, stepPeriod, trot_2_stance_ratio)
#                 joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(gait_traj['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
#                 ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_ang_FL[0:3])
#                 joints_ang_FR, joints_vel_FR, joints_toq_FR = ROBOT.control(gait_traj['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
#                 ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joints_ang_FR[3:6])
#                 joints_ang_HL, joints_vel_HL, joints_toq_HL = ROBOT.control(gait_traj['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
#                 ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joints_ang_HL[6:9])
#                 joints_ang_HR, joints_vel_HR, joints_toq_HR = ROBOT.control(gait_traj['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
#                 ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joints_ang_HR[9:12])
#             p.stepSimulation()
#             ROBOT.time_step += 1
#         elif sim_cfg['mode'] == "towr":
#             with open(TOWR, 'r', newline='') as csv_file:
#                 try: 
#                     EE_POSE = np.array([float(x) for x in next(reader)])
#                 except StopIteration:
#                     break
#                 print("EE_POSE -> ", EE_POSE)
#                 towr = {"COM": EE_POSE[0:3], "FL_FOOT": {'P' : EE_POSE[3:6], 'D': np.zeros(3)}, "FR_FOOT": {'P': EE_POSE[6:9], 'D': np.zeros(3)}, 
#                         "HL_FOOT": {'P': EE_POSE[9:12], 'D' : np.zeros(3)}, "HR_FOOT": {'P': EE_POSE[12:15], 'D': np.zeros(3)}}
#                 towr = towr_transform(towr)
#                 joint_ang_FL, joint_vel_FL, joint_toq_FL = ROBOT.control(towr['FL_FOOT'], ROBOT.EE_index['FL_FOOT'], mode=ROBOT.mode)
#                 ROBOT.setJointControl(ROBOT.jointidx['FL'], ROBOT.mode, joint_ang_FL[0:3])
#                 joints_ang_FR, joints_vel_FR, joints_toq_FR = ROBOT.control(towr['FR_FOOT'], ROBOT.EE_index['FR_FOOT'], mode=ROBOT.mode)
#                 ROBOT.setJointControl(ROBOT.jointidx['FR'], ROBOT.mode, joints_ang_FR[3:6])
#                 joints_ang_HL, joints_vel_HL, joints_toq_HL = ROBOT.control(towr['HL_FOOT'], ROBOT.EE_index['HL_FOOT'], mode=ROBOT.mode)
#                 ROBOT.setJointControl(ROBOT.jointidx['BL'], ROBOT.mode, joints_ang_HL[6:9])
#                 joints_ang_HR, joints_vel_HR, joints_toq_HR = ROBOT.control(towr['HR_FOOT'], ROBOT.EE_index['HR_FOOT'], mode=ROBOT.mode)
#                 ROBOT.setJointControl(ROBOT.jointidx['BR'], ROBOT.mode, joints_ang_HR[9:12])
#             p.stepSimulation()
#             ROBOT.time_step += 1
#     p.disconnect()