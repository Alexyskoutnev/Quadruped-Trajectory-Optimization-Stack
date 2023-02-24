import pybullet as p
import numpy as np

def links_to_id(robot):
    _link_name_to_index = {p.getBodyInfo(robot)[0].decode('UTF-8'):-1,}
    for _id in range(p.getNumJoints(robot)):
        _name = p.getJointInfo(robot, _id)[12].decode('UTF-8')
        _link_name_to_index[_name] = _id
    return _link_name_to_index

def link_info(link):
    return {"linkWorldPosition": link[0], "linkWorldOrientation": link[1], "localInertialFramePosition": link[2], "localInertialFrameOrientation": link[3], 
     "worldLinkFramePosition": link[4], "worldLinkFrameOrientation": link[5]}

def q_init_16_arr(q_init):
    "places q_init into a 16 index array to account for fixed joints in urdf"
    indexes = [0,1,2,4,5,6,8,9,10,12,13,14]
    q_init_new = np.zeros(16)
    for i, q_val in zip(indexes, q_init):
        q_init_new[i] = q_val
    return q_init_new


class SOLO12(object):
    def __init__(self, client, robot, config):
        self.client = client
        self.robot = robot
        self.jointidx = {"FL": [0, 1, 2], "FR": [4, 5, 6], "BL": [8, 9, 10], "BR": [12, 13, 14], "idx":  [0,1,2,4,5,6,8,9,10,12,13,14]}
        self.fixjointidx = {"FL": 3, "FR": 7, "BL": 11, "BR": 15, "idx": [3,7,11,15]}
        self.config = config
        self.links = links_to_id(robot)
        # self.q_init = np.array(config['q_init'])
        self.q_init = np.array([0 for i in range(12)])
        self.q_init16 = q_init_16_arr(self.q_init)
        self.q_init[0] = np.pi
        self.q_init = [0 for i in range(12)]
        self.EE = {'FL_FOOT': None, 'FR_FOOT': None, "HL_FOOT": None, "HR_FOOT": None}
        self.EE_index = {'FL_FOOT': 3, 'FR_FOOT': 7, "HL_FOOT": 11, "HR_FOOT": 15}
        
        
        #initial robot pose and configuration
        # breakpoint()
        # p.resetJointStatesMultiDof(self.robot, self.jointIdx, self.q_init)
        for i in self.jointidx['idx']:
            # breakpoint()
            p.resetJointState(self.robot, i, self.q_init16[i])
        # breakpoint()


    def get_link_states(self):
        
        
        pass
        
        
    
    def setJointControl(self, jointsInx, controlMode, cmd_pose, cmd_vel=None, cmd_f=None):
        p.setJointMotorControlArray(self.robot, jointsInx, controlMode, cmd_pose)
        
    def get_endeffector_pose(self):
        EE1, EE2, EE3, EE4 = p.getLinkStates(self.robot,  self.EE_index.values())
        return {"FL_FOOT": link_info(EE1), "FR_FOOT": link_info(EE2), "HL_FOOT": link_info(EE3), "HR_FOOT": link_info(EE4)}
    
    def invKinematics(self, pose, index):
        joint = None
        if len(pose) == 3:
            joint = p.calculateInverseKinematics(self.robot, index, pose)
        elif len(pose) == 2:
            joint = p.calculateInverseKinematics(self.robot, index, pose[0], pose[1])
        return joint
