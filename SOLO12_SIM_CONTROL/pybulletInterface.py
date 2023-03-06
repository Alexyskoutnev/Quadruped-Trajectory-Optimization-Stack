import pybullet as p
import time
import numpy as np


class PybulletInterface(object):

    def __init__(self):
        
        self.xId = p.addUserDebugParameter('x', -0.1, 0.1, 0.0)
        self.yId = p.addUserDebugParameter('y', -0.1, 0.1, 0.0)
        self.zId = p.addUserDebugParameter('z', -0.1, 0.1, 0.0)
        self.rollId = p.addUserDebugParameter('roll', -np.pi/4, np.pi/4, 0.0)
        self.pitchId = p.addUserDebugParameter('pitch', -np.pi/4, np.pi/4, 0.0)
        self.yawId = p.addUserDebugParameter('yaw', -np.pi/4, np.pi/4, 0.0)
        self.velocityId = p.addUserDebugParameter('velocity', -3.0, 3.0, 0.0)
        self.anglevelocityId = p.addUserDebugParameter('anglevelocity', -1.5, 1.5, 0.0)
        self.angleId = p.addUserDebugParameter('angle', -90, 90, 0.0)
        self.periodId = p.addUserDebugParameter('stepPeriod', 0.1, 3.0, 2.5)

    def robostates(self, boxId):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

        #Add interface to change orientation and position of pybullet camera
        try: 
            pos = np.array([p.readUserDebugParameter(self.xId), p.readUserDebugParameter(self.yId), p.readUserDebugParameter(self.zId)])
            angle = np.array([p.readUserDebugParameter(self.rollId), p.readUserDebugParameter(self.pitchId), p.readUserDebugParameter(self.yawId)])
            velocity = p.readUserDebugParameter(self.velocityId)
            angle_velocity = p.readUserDebugParameter(self.anglevelocityId)
            angle = p.readUserDebugParameter(self.angleId)
            stepPeriod = p.readUserDebugParameter(self.periodId)
        except:
            print("error in pybullet interface")
            pos = np.array([0,0,0])
            angle = np.array([0,0,0])
            velocity = 0.0
            angle_velocity = 0.0
            angle = 0.0
            stepPeriod = 1.0

        return pos, angle, velocity, angle_velocity , angle,  stepPeriod

