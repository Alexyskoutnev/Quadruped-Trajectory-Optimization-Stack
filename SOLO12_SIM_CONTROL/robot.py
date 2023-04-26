import time


import pybullet as p
import numpy as np

from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv, convert12arr_2_16arr

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


def shift_z(v, shift):
    v[2] += shift
    return v
        
class SOLO12(object):
    def __init__(self, URDF, config, fixed = 0, sim_cfg=None):
        self.config = config
        self.robot = p.loadURDF(URDF, config['start_pos'], config['start_ang'],  useFixedBase=fixed)
        self.jointidx = {"FL": [0, 1, 2], "FR": [4, 5, 6], "BL": [8, 9, 10], "BR": [12, 13, 14], "idx": [0,1,2,4,5,6,8,9,10,12,13,14]}
        self.fixjointidx = {"FL": 3, "FR": 7, "BL": 11, "BR": 15, "idx": [3,7,11,15]}
        self.links = links_to_id(self.robot)
        self.q_init = np.array(config['q_init'])
        self.q_init16 = q_init_16_arr(self.q_init)
        self.EE = {'FL_FOOT': None, 'FR_FOOT': None, "HL_FOOT": None, "HR_FOOT": None}
        self.EE_index = {'FL_FOOT': 3, 'FR_FOOT': 7, "HL_FOOT": 11, "HR_FOOT": 15}
        self.time_step = 0
        self.t_max = config['t_max']
        self.modes = {"P": p.POSITION_CONTROL, "PD": p.VELOCITY_CONTROL, "torque": p.TORQUE_CONTROL}
        self.mode = config['mode']
        self.sim_cfg = sim_cfg
        
        self.tfBaseMtx = transformation_inv(transformation_mtx(self.CoM_states()['linkWorldPosition'], self.CoM_states()['linkWorldOrientation']))
        self.EE_WORLD = {"FL_W_POSE": base_frame_tf(self.tfBaseMtx, self.get_endeffector_pose()['FL_FOOT']['linkWorldPosition']), "FR_W_POSE": base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['FR_FOOT']['linkWorldPosition']),
                                    "HL_W_POSE": base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['HL_FOOT']['linkWorldPosition']), "HR_W_POSE": base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['HR_FOOT']['linkWorldPosition'])}
        if sim_cfg['mode'] == "bezier":
            self.shiftZ = 0.05
        elif sim_cfg['mode'] == "towr":
            self.shiftZ = 0.05
        else:
            self.shiftZ = 0.0
        self.shift = {'FL_FOOT': shift_z(self.EE_WORLD['FL_W_POSE'], self.shiftZ), 'FR_FOOT': shift_z(self.EE_WORLD['FR_W_POSE'], self.shiftZ),
                     'HL_FOOT': shift_z(self.EE_WORLD['HL_W_POSE'], self.shiftZ), 'HR_FOOT': shift_z(self.EE_WORLD['HR_W_POSE'], self.shiftZ)}
        
        #initial robot pose and configuration
        for i in self.jointidx['idx']:
            p.resetJointState(self.robot, i, self.q_init16[i])

        #reset default controller
        p.setJointMotorControlArray(self.robot, jointIndices=self.jointidx['idx'],
                                      controlMode=p.VELOCITY_CONTROL,
                                      targetVelocities= [0.0 for m in self.jointidx['idx']],
                                      forces= [0.0 for m in self.jointidx['idx']])
        
        self.kp = np.ones(12) * 0.5
        self.kd = np.ones(12) * 0.1
        self._joint_ang = None
        self._joint_vel = None
        self._joint_toq = None

    def CoM_states(self):
        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        return {"linkWorldPosition": CoM_pos, "linkWorldOrientation": CoM_angle}

    def get_link_states(self):
        pass
    
    @property
    def jointangles(self):
        return self._joint_ang

    def setJointControl(self, jointsInx, controlMode, cmd_pose, cmd_vel=None, cmd_f=None):
        if 'P' == controlMode or 'PD' == controlMode:
            maxForces = np.ones(len(jointsInx))*10
            posGains = np.ones(len(jointsInx))*0.5
            p.setJointMotorControlArray(self.robot, jointsInx, self.modes['P'], cmd_pose, forces=maxForces, positionGains=posGains)
        elif 'torque' == controlMode:
            p.setJointMotorControlArray(self.robot, jointsInx, controlMode=p.TORQUE_CONTROL, forces=cmd_pose)

        
    def get_endeffector_pose(self):
        EE1, EE2, EE3, EE4 = p.getLinkStates(self.robot,  self.EE_index.values())
        return {"FL_FOOT": link_info(EE1), "FR_FOOT": link_info(EE2), "HL_FOOT": link_info(EE3), "HR_FOOT": link_info(EE4)}
    

    def control(self, cmd, index, mode="P"):
        q_cmd = None
        q_vel = None
        q_toq = None
        if mode == 'P':
            q_cmd, q_vel = self.inv_kinematics(cmd, index, mode=mode)
        elif mode == 'PD':
            q_cmd, q_vel = self.inv_kinematics(cmd, index, mode=mode)
        elif mode == 'torque':
            q_cmd, q_vel, q_toq = self.inv_dynamics(cmd, index)
        return q_cmd, q_vel, q_toq

    def inv_dynamics(self, cmd, index):

        q_cmd, q_vel = self.inv_kinematics(cmd, index, mode = "PD")
        _q_cmd = np.array(q_cmd).reshape((12, 1))
        _q_vel = np.array(q_vel).reshape((12, 1))

        q_mes = np.zeros((12, 1))
        v_mes = np.zeros((12, 1))


        jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
        q_mes[:, 0] = [state[0] for state in jointStates]
        v_mes[:, 0] = [state[1] for state in jointStates]


        kp = 5.0
        kd = 0.01 * np.array([[1.0, 0.3, 0.3, 1.0, 0.3, 0.3, 1.0, 0.3, 0.3, 1.0, 0.3, 0.3]]).transpose()
        dt = 0.001
        cpt = self.time_step
        t1 = 5

        
        ev = dt * cpt
        A3 = 2 * (q_mes - _q_cmd) / t1**3
        A2 = (-3/2) * t1 * A3
        q_des = q_mes + A2*(ev**2) + A3*(ev**3)
        v_des = 2*A2*ev + 3*A3*(ev**2)
        jointTorques = kp * (_q_cmd - q_mes) + kd * (_q_vel - v_mes)
        t_max = 3.0
        jointTorques[jointTorques > t_max] = t_max
        jointTorques[jointTorques < -t_max] = -t_max    

        return q_cmd, q_vel, jointTorques.reshape(12)

    # def invDyanamics(self, cmd, index, kp = 1.0, kd = 1.0):
    #     jointmap = {3: "FL", 7: "FR", 11: "BL", 15: "BR"}
    #     # jointindices = self.jointidx[jointmap[index]]
    #     jointindices = self.jointidx["idx"]
    #     numJoints = len(jointindices)
    #     q_cmd, q_vel = self.inv_kinematics(cmd, index, mode = "PD")
        
    #     q1 = []
    #     qdot1 = []
    #     zeroAcceleration = []



    #     # curPos, curOrn = p.getBasePositionAndOrientation(self.robot)
    #     # q1 = [curPos[0], curPos[1], curPos[2], curOrn[0], curOrn[1], curOrn[2], curOrn[3]]
    #     # baseLinVel, baseAngVel = p.getBaseVelocity(self.robot)
    #     # qdot1 = [baseLinVel[0], baseLinVel[1], baseLinVel[2], baseAngVel[0], baseAngVel[1], baseAngVel[2], 0]
    #     # q_err = [0, 0, 0, 0, 0, 0, 0]
    #     # qIndex = 7
    #     # qdotIndex = 7
    #     # zeroAccelerations = [0, 0, 0, 0, 0, 0, 0]
    #     for i in range(numJoints):
    #         jointStates = p.getJointStateMultiDof(self.robot, jointindices[i])
    #         q1.append(jointStates[0])
    #         qdot1.append(jointStates[1])
    #         zeroAcceleration.append(0)

    #     q = np.array(q1)
    #     qdot = np.array(qdot1)
    #     qdes = np.array(q_cmd)
    #     qdotdes = np.array(q_vel)

    #     q_err = qdes - q
    #     q_dot_err = qdotdes - qdot

    #     Kp = np.diagflat(kp)
    #     Kd = np.diagflat(kd)

    #     p_term = kp * (q_err - q)
    #     d_term = kd * (q_dot_err - qdot)

    #     breakpoint()

    #     q_ = [q[0] for q in q1]

    #     M = np.array(p.calculateMassMatrix(self.robot, q_))

    #     q1 = [q[0] for q in q1] 
    #     q1 += [0.0] * 4
    #     q1[3] = 0
    #     q1[7] = 0
    #     q1[11] = 0
    #     q1[15] = 0
    #     # qdot1 = [q[0] for q in [0.1]*1]
    #     qdot1 = [0.1] * 16
    #     qdot1[3] = 0
    #     qdot1[7] = 0
    #     qdot1[11] = 0
    #     qdot1[15] = 0

    #     zeroAcceleration = [0.0] * 16
        
        
    #     breakpoint()
        
    #     G = p.calculateInverseDynamics(self.robot, q1, qdot1, zeroAcceleration)
        

    #     breakpoint()


    #         # js = p.getJointStateMultiDof(self.robot, jointindices[i])
    #         # jointPos = js[0]
    #         # jointVel = js[1]
    #         # q1 += jointPos
    #         # if len(js[0]) == 1:
    #         #     desiredPos = q_cmd[jointindices[i]]
    #         #     qdiff = desiredPos - jointPos[0]
    #         #     q_err.append(qdiff)
    #         #     zeroAccelerations.append(0.)
    #         #     qdot1 += jointVel
    #         #     qIndex += 1
    #         #     qdotIndex += 1
            
    #     breakpoint()


    #     # q_cmd, q_vel, q_tor = None, None, None
    #     return q_cmd, q_vel, q_tor
    def inv_kinematics_multi(self, cmds, indices, mode = 'P'):
        assert(len(indices) == 4)
        joint_position = np.zeros(12)
        joint_velocity = np.zeros(12)
        idx_2_EE = {3 : "FL_FOOT", 7: "FR_FOOT", 11: "HL_FOOT", 15: "HR_FOOT"}
        for idx in indices:
            if idx_2_EE[idx] == "FL_FOOT":
                joint_position[0:3] = self.inv_kinematics(cmds[idx_2_EE[idx]], idx, mode)[0][0:3]
            elif idx_2_EE[idx] == "FR_FOOT":
                joint_position[3:6] = self.inv_kinematics(cmds[idx_2_EE[idx]], idx, mode)[0][3:6]
            elif idx_2_EE[idx] == "HL_FOOT":
                joint_position[6:9] = self.inv_kinematics(cmds[idx_2_EE[idx]], idx, mode)[0][6:9]
            elif idx_2_EE[idx] == "HR_FOOT":
                joint_position[9:12] = self.inv_kinematics(cmds[idx_2_EE[idx]], idx, mode)[0][9:12]
        self._joint_ang = joint_position #More accurate update ??
        return joint_position

    def inv_kinematics(self, cmd, index, mode = 'P'):
        joint_position = None
        joint_velocity = None
        if mode == 'P':
            if len(cmd['P']) == 3: #Position (3d)
                joint_position = p.calculateInverseKinematics(self.robot, index, cmd["P"])
            elif len(cmd['P']) == 2: #Position (3d) & Angle (4d)
                joint_position = p.calculateInverseKinematics(self.robot, index, cmd["P"][0], cmd["P"][1])
        elif mode == "PD":
            joint_velocity = np.zeros(12)
            if len(cmd['P']) == 3: #Position (3d)
                joint_position = p.calculateInverseKinematics(self.robot, index, cmd["P"])
            elif len(cmd['P']) == 2: #Position (3d) & Angle (4d)
                joint_position = p.calculateInverseKinematics(self.robot, index, cmd["P"][0], cmd["P"][1])
            velocities = []
            if index == 3: #FL
                for state in p.getJointStates(self.robot, [0, 1, 2]):
                    velocities.append(state[1])
                for i, idx in enumerate([0, 1, 2]):
                    joint_velocity[idx] = velocities[i]
            elif index == 7: #FR
                for state in p.getJointStates(self.robot, [4, 5, 6]):
                    velocities.append(state[1])
                for i, idx in enumerate([3, 4, 5]):
                    joint_velocity[idx] = velocities[i]
            elif index == 11: #HL
                for state in p.getJointStates(self.robot, [8, 9, 10]):
                    velocities.append(state[1])
                for i, idx in enumerate([6, 7, 8]):
                    joint_velocity[idx] = velocities[i]
            elif index == 15:
                for state in p.getJointStates(self.robot, [12, 13, 14]):
                    velocities.append(state[1])
                for i, idx in enumerate([9, 10, 11]):
                    joint_velocity[idx] = velocities[i]
        self._joint_ang = joint_position #COULD BE POTENTIALY NOT 100% accurate if we update joint 4 but we want 1 (old joint reference)
        self._joint_vel = joint_velocity
        return joint_position, joint_velocity

    def default_stance_control(self, q_cmd, control=p.TORQUE_CONTROL):

        q_mes = np.zeros((12, 1))
        v_mes = np.zeros((12, 1))
        jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
        q_mes[:, 0] = [state[0] for state in jointStates]
        v_mes[:, 0] = [state[1] for state in jointStates]
        kp = 5.0
        kd = 0.10 * np.array([[1.0, 0.3, 0.3, 1.0, 0.3, 0.3, 1.0, 0.3, 0.3, 1.0, 0.3, 0.3]]).transpose()
        dt = 0.001
        cpt = self.time_step
        t1 = 4

        if control == p.TORQUE_CONTROL:
                ev = dt * cpt
                A3 = 2 * (q_mes - q_cmd.reshape((12, 1))) / t1**3
                A2 = (-3/2) * t1 * A3
                q_des = q_mes + A2*(ev**2) + A3*(ev**3)
                v_des = 2*A2*ev + 3*A3*(ev**2)
                jointTorques = kp * (q_cmd.reshape((12, 1)) - q_mes) + kd * (v_des - v_mes)
                t_max = 2.5
                jointTorques[jointTorques > t_max] = t_max
                jointTorques[jointTorques < -t_max] = -t_max

        return jointTorques

            








