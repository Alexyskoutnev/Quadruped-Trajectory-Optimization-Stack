import pybullet as p
import time
import pybullet_data
import numpy as np

# Define the robot's joint angles and velocities or retrieve it from some predefined function. --> calculate p.calculateInverseKinematics(robot, index, pose)
#

robot_id = p.loadURDF("./data/urdf/solo12.urdf")

# set the desired joint angles and velocities. This should be retrieved from calculateInverseKinematics
joint_angles = [0.0, -0.3, 0.0, 1.2, 0.0, -1.6, 0.0]
joint_velocities = [0.0] * 7

#calculate the joint torques using inverse dynamics

joint_torques = p.calculateInverseDynamics(robot_id, joint_angles, joint_velocities)
breakpoint()

# apply the joint torques to the robot
for i in range(7):
    p.setJointMotorControl2(robot_id, i, p.TORQUE_CONTROL, force=joint_torques[i])

    # TO DO WRITE A CONTROLLER FOR THE ROBOT TO CONTINOUSLY UPDATE THE JOINT ANGLES AND VELOCITIES.