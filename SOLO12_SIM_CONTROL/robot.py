import pybullet as p

def links_to_id(robot):
    _link_name_to_index = {p.getBodyInfo(robot)[0].decode('UTF-8'):-1,}
    for _id in range(p.getNumJoints(robot)):
        _name = p.getJointInfo(robot, _id)[12].decode('UTF-8')
        _link_name_to_index[_name] = _id
    return _link_name_to_index

def link_info(link):
    return {"linkWorldPosition": link[0], "linkWorldOrientation": link[1], "localInertialFramePosition": link[2], "localInertialFrameOrientation": link[3], 
     "worldLinkFramePosition": link[4], "worldLinkFrameOrientation": link[5]}


class SOLO12(object):
    def __init__(self, client, robot, config):
        self.client = client
        self.robot = robot
        self.config = config
        self.links = links_to_id(robot)
        self.EE = {'FL_FOOT': None, 'FR_FOOT': None, "HL_FOOT": None, "HR_FOOT": None}
        self.EE_index = {'FL_FOOT': 3, 'FR_FOOT': 7, "HL_FOOT": 11, "HR_FOOT": 15}
        
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