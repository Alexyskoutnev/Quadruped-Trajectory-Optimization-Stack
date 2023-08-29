import pybullet as p
import time
import sys
import numpy as np


class RecordInterface(object):
    
    def __init__(self, args, robot):
        print()
        if args.get('camera_yaw'):
            self.camera_yaw = args['camera_yaw']
        else:
             self.camera_yaw = 40
        if args.get('camera_pitch'):
            self.camera_pitch = args['camera_pitch']
        else:
            self.camera_pitch = -15
        if args.get('camera_roll'):
            self.camera_roll = args['camera_roll']
        else:
            self.camera_roll = 0
        if args.get('camera_distance'):
            self.camera_distance = args['camera_distance']
        else:
            self.camera_distance = 1.25
        self.robot = robot

    def update(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robot)
        p.resetDebugVisualizerCamera(cameraDistance=self.camera_distance, cameraYaw=self.camera_yaw, cameraPitch=self.camera_pitch, cameraTargetPosition=cubePos)

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
        self.periodId = p.addUserDebugParameter('stepPeriod', 0.1, 3.0, 2.0)

    def robostates(self, boxId):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

        #Add interface to change orientation and position of pybullet camera
        p.resetDebugVisualizerCamera(cameraDistance=self.camera_distance, cameraYaw=self.camera_yaw, cameraPitch=self.camera_pitch, cameraTargetPosition=cubePos)
        keys = p.getKeyboardEvents()
        for k in keys:
            if keys.get(100): # D
                self.camera_yaw += 1
            if keys.get(97): # A
                self.camera_yaw -= 1
            if keys.get(113): #Q
                self.camera_pitch -= 1
            if keys.get(101): #E
                self.camera_pitch += 1
            if keys.get(122): #W
                self.camera_distance -= 0.01
            if keys.get(120): #S
                self.camera_distance += 0.01
            if keys.get(27):
                p.disconnect()
                sys.exit()
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
        return pos, angle, velocity, angle_velocity, stepPeriod