import time


import pybullet as p
import numpy as np

from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv

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

def base_frame_tf(mtx, pt):
    vec = np.concatenate((np.array([pt[0]]), np.array([pt[1]]),np.array([pt[2]]), np.ones(1)))
    tf_vec = mtx @ vec
    return tf_vec[:3]


class SOLO12(object):
    def __init__(self, URDF, config):
        self.config = config
        self.robot = p.loadURDF(URDF, config['start_pos'], config['start_ang'],  useFixedBase=0)
        self.jointidx = {"FL": [0, 1, 2], "FR": [4, 5, 6], "BL": [8, 9, 10], "BR": [12, 13, 14], "idx": [0,1,2,4,5,6,8,9,10,12,13,14]}
        self.fixjointidx = {"FL": 3, "FR": 7, "BL": 11, "BR": 15, "idx": [3,7,11,15]}
        self.links = links_to_id(self.robot)
        self.q_init = np.array(config['q_init'])
        # self.q_init = np.array([0 for i in range(12)])
        self.q_init16 = q_init_16_arr(self.q_init)
        self.EE = {'FL_FOOT': None, 'FR_FOOT': None, "HL_FOOT": None, "HR_FOOT": None}
        self.EE_index = {'FL_FOOT': 3, 'FR_FOOT': 7, "HL_FOOT": 11, "HR_FOOT": 15}
        self.time_step = 0
        self.t_max = config['t_max']
        self.modes = {"position": p.POSITION_CONTROL, "velocity": p.VELOCITY_CONTROL, "torque": p.TORQUE_CONTROL}
        self.mode = config['mode']
        
        self.tfBaseMtx = transformation_inv(transformation_mtx(self.CoM_states()['linkWorldPosition'], self.CoM_states()['linkWorldOrientation']))
        self.shift = {'FL_FOOT': base_frame_tf(self.tfBaseMtx, self.get_endeffector_pose()['FL_FOOT']['linkWorldPosition']), 'FR_FOOT': base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['FR_FOOT']['linkWorldPosition']),
                     'HL_FOOT': base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['HL_FOOT']['linkWorldPosition']), 'HR_FOOT': base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['HR_FOOT']['linkWorldPosition'])}
        
        #initial robot pose and configuration
        for i in self.jointidx['idx']:
            p.resetJointState(self.robot, i, self.q_init16[i])

        #reset default controller
        p.setJointMotorControlArray(self.robot, jointIndices=self.jointidx['idx'],
                                      controlMode=p.VELOCITY_CONTROL,
                                      targetVelocities= [0.0 for m in self.jointidx['idx']],
                                      forces= [0.0 for m in self.jointidx['idx']])

    def CoM_states(self):
        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        return {"linkWorldPosition": CoM_pos, "linkWorldOrientation": CoM_angle}


    def get_link_states(self):
        
        
        pass
        
        
    
    def setJointControl(self, jointsInx, controlMode, cmd_pose, cmd_vel=None, cmd_f=None):
        p.setJointMotorControlArray(self.robot, jointsInx, self.modes[controlMode], cmd_pose)
        
    def get_endeffector_pose(self):
        EE1, EE2, EE3, EE4 = p.getLinkStates(self.robot,  self.EE_index.values())
        return {"FL_FOOT": link_info(EE1), "FR_FOOT": link_info(EE2), "HL_FOOT": link_info(EE3), "HR_FOOT": link_info(EE4)}
    
    
    def invDynamics(self, pose, index):
        # breakpoint()
        q_vel = None
        if len(pose) == 3: #Position (3d)
            q_cmd = np.array(p.calculateInverseKinematics(self.robot, index, pose))
        elif len(pose) == 2: #Position (3d) & Angle (4d)
            q_cmd = np.array(p.calculateInverseKinematics(self.robot, index, pose[0], pose[1]))
        q_mes = np.zeros((12, 1))
        v_mes = np.zeros((12, 1))
        jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
        q_mes[:, 0] = [state[0] for state in jointStates]
        v_mes[:, 0] = [state[1] for state in jointStates]
        kp = 0.1
        kd = 0.05 * np.array([[1.0, 0.3, 0.3, 1.0, 0.3, 0.3, 1.0, 0.3, 0.3, 1.0, 0.3, 0.3]]).transpose()
        dt = 0.001
        cpt = self.time_step
        t1 = 4
        ev = dt * cpt
        # breakpoint()
        A3 = 2 * (q_mes - q_cmd.reshape((12, 1))) / t1**3
        A2 = (-3/2) * t1 * A3
        q_des = q_mes + A2*(ev**2) + A3*(ev**3)
        v_des = 2*A2*ev + 3*A3*(ev**2)
        q_toq = kp * (q_des - q_mes) + kd * (v_des - v_mes)
        q_toq[q_toq > self.t_max] = self.t_max
        q_toq[q_toq < -self.t_max] = -self.t_max
        print(f"{cpt}: q_toq -> {q_toq}")
        breakpoint()
        return q_cmd, q_vel, q_toq

    def control(self, pose, index, mode="position"):
        q_cmd = None
        q_vel = None
        q_toq = None
        if mode == 'position':
            q_cmd = self.invKinematics(pose, index)
        elif mode == 'velocity':
            pass
        elif mode == 'torque':
            q_cmd, q_vel, q_toq = self.invDynamics(pose, index)

        return q_cmd, q_vel, q_toq
    
    def invKinematics(self, pose, index):
        joint = None
        if len(pose) == 3: #Position (3d)
            joint = p.calculateInverseKinematics(self.robot, index, pose)
        elif len(pose) == 2: #Position (3d) & Angle (4d)
            joint = p.calculateInverseKinematics(self.robot, index, pose[0], pose[1])
        return joint

    def default_stance_control(self, q_cmd, control=p.POSITION_CONTROL):
        q_mes = np.zeros((12, 1))
        v_mes = np.zeros((12, 1))


        jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
        q_mes[:, 0] = [state[0] for state in jointStates]
        v_mes[:, 0] = [state[1] for state in jointStates]
        print(f"q_mes: {q_mes}")
        print(f"v_mes: {v_mes}")


        kp = 5.0
        kd = 0.05 * np.array([[1.0, 0.3, 0.3, 1.0, 0.3, 0.3, 1.0, 0.3, 0.3, 1.0, 0.3, 0.3]]).transpose()
        dt = 0.001
        cpt = self.time_step
        t1 = 4

        if control == p.TORQUE_CONTROL:
                ev = dt * cpt
                A3 = 2 * (q_mes - q_cmd.reshape((12, 1))) / t1**3
                A2 = (-3/2) * t1 * A3
                q_des = q_mes + A2*(ev**2) + A3*(ev**3)
                v_des = 2*A2*ev + 3*A3*(ev**2)
                jointTorques = kp * (q_des - q_mes) + kd * (v_des - v_mes)
                t_max = 2.5
                jointTorques[jointTorques > t_max] = t_max
                jointTorques[jointTorques < -t_max] = -t_max
                # print(f"cpt {cpt} -> {jointTorques}")
                # breakpoint()

        # while True or np.max(np.abs(q_cmd - q_mes)) > 0.1:

        #     time_d = time.time()

        #     jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
        #     q_mes[:, 0] = [state[0] for state in jointStates]
        #     v_mes[:, 0] = [state[1] for state in jointStates]

        #     print(f"q_mes: {q_mes}")
        #     print(f"v_mes: {v_mes}")


        #     #PD control for joints 
        #     if control == p.POSITION_CONTROL:
        #         ev = dt * cpt
        #         A3 = 2 * (q_mes - q_cmd.reshape((12, 1))) / t1**3
        #         A2 = (-3/2) * t1 * A3
        #         q_des = q_mes + A2*(ev**2) + A3*(ev**3)
        #         v_des = 2*A2*ev + 3*A3*(ev**2)
        #         joint_d = kp * (q_des - q_mes) + kd * (v_des - v_mes)
        #         self.setJointControl(self.jointidx['idx'], control, joint_d)

        #     if control == p.TORQUE_CONTROL:
        #         ev = dt * cpt
        #         A3 = 2 * (q_mes - q_cmd.reshape((12, 1))) / t1**3
        #         A2 = (-3/2) * t1 * A3
        #         q_des = q_mes + A2*(ev**2) + A3*(ev**3)
        #         v_des = 2*A2*ev + 3*A3*(ev**2)
        #         jointTorques = kp * (q_des - q_mes) + kd * (v_des - v_mes)
        #         t_max = 2.5
        #         jointTorques[jointTorques > t_max] = t_max
        #         jointTorques[jointTorques < -t_max] = -t_max
        #         print(f"cpt {cpt} -> {jointTorques}")

        #         # Set control torque for all joints
        #     # p.setJointMotorControlArray(self.robot, self.jointidx['idx'],
        #                                 #   controlMode=p.TORQUE_CONTROL, forces=jointTorques)



                

        #     # p.stepSimulation()
        #     # time.sleep(1.0/100000.0)


        #     cpt += 1 
            
        #     while(time.time() - time_d) < dt:
        #         pass

        return jointTorques

            








