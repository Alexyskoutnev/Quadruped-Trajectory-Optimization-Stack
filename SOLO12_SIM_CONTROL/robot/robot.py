import time

import pybullet as p
import numpy as np

from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv, convert12arr_2_16arr, create_cmd
from SOLO12_SIM_CONTROL.robot.robot_motor import MotorModel

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
            self.shift = {'FL_FOOT': shift_z(self.EE_WORLD['FL_W_POSE'], self.shiftZ), 'FR_FOOT': shift_z(self.EE_WORLD['FR_W_POSE'], self.shiftZ),
                     'HL_FOOT': shift_z(self.EE_WORLD['HL_W_POSE'], self.shiftZ), 'HR_FOOT': shift_z(self.EE_WORLD['HR_W_POSE'], self.shiftZ)}
        elif sim_cfg['mode'] == "towr":
            self.shiftZ = 0.05
            self.shift = {'FL_FOOT': shift_z(self.EE_WORLD['FL_W_POSE'], self.shiftZ), 'FR_FOOT': shift_z(self.EE_WORLD['FR_W_POSE'], self.shiftZ),
                     'HL_FOOT': shift_z(self.EE_WORLD['HL_W_POSE'], self.shiftZ), 'HR_FOOT': shift_z(self.EE_WORLD['HR_W_POSE'], self.shiftZ)}
        else:
            self.shift = {'FL_FOOT': np.zeros(3), 'FR_FOOT': np.zeros(3),
                        'HL_FOOT': np.zeros(3), 'HR_FOOT': np.zeros(3)}
        #initial robot pose and configuration
        for i in self.jointidx['idx']:
            p.resetJointState(self.robot, i, self.q_init16[i])
        #reset default controller
        p.setJointMotorControlArray(self.robot, jointIndices=self.jointidx['idx'],
                                      controlMode=p.VELOCITY_CONTROL,
                                      targetVelocities= [0.0 for m in self.jointidx['idx']],
                                      forces= [0.0 for m in self.jointidx['idx']])
        self._kp = config['kp']
        self._kd = config['kd']
        self._motor = MotorModel(self._kp, self._kd)
        self._joint_ang = None
        self._joint_vel = None
        self._joint_toq = None

    def CoM_states(self):
        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        return {"linkWorldPosition": CoM_pos, "linkWorldOrientation": CoM_angle}

    @property
    def state(self):
        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        EE = self.get_endeffector_pose()
        return {"COM": CoM_pos, "linkWorldOrientation": CoM_angle, "FL_FOOT": EE['FL_FOOT']['linkWorldPosition'], 
                "FR_FOOT": EE['FR_FOOT']['linkWorldPosition'], "HL_FOOT": EE['HL_FOOT']['linkWorldPosition'], "HR_FOOT": EE['HR_FOOT']['linkWorldPosition']}
    
    @property
    def jointangles(self):
        return self._joint_ang

    def setJointControl(self, jointsInx, controlMode, cmd_pose, cmd_vel=None, cmd_f=None):
        if 'P' == controlMode or 'PD' == controlMode:
            # maxForces = np.ones(len(jointsInx))*5
            maxForces = np.ones(len(jointsInx))*2
            posGains = np.ones(len(jointsInx))*self._kp
            p.setJointMotorControlArray(self.robot, jointsInx, self.modes['P'], cmd_pose, positionGains=posGains)
        elif 'torque' == controlMode:
            p.setJointMotorControlArray(self.robot, jointsInx, controlMode=p.TORQUE_CONTROL, forces=cmd_pose)

    def get_endeffector_pose(self):
        EE1, EE2, EE3, EE4 = p.getLinkStates(self.robot,  self.EE_index.values())
        return {"FL_FOOT": link_info(EE1), "FR_FOOT": link_info(EE2), "HL_FOOT": link_info(EE3), "HR_FOOT": link_info(EE4)}
    
    def control(self, cmd, index, mode="P"):
        """_summary_

        Args:
            cmd (_type_): _description_
            index (_type_): _description_
            mode (str, optional): _description_. Defaults to "P".

        Returns:
            _type_: _description_
        """
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

    # def _inv_dynamics_multi(self, cmds, indices):
    #     q_cmd, q_vel = self.inv_kinematics(cmds, indices, mode = "PD")
    #     q_cmd = np.array(q_cmd).reshape(12)
    #     q_vel = np.array(q_vel).reshape(12)
    #     q_mes = np.zeros(12)
    #     v_mes = np.zeros(12)
    #     jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
    #     q_mes[:] = [state[0] for state in jointStates]
    #     v_mes[:] = [state[1] for state in jointStates]
    #     q_toq = self._motor.convert_to_torque(q_cmd, q_mes, v_mes)
    #     return q_cmd, q_vel, q_toq


    def inv_dynamics(self, cmd, index):
        """Inverse dynamics controller based on joint angle and velocity commands 
           to produced motor torque commands.
        Args:
            cmd (array): 3d desired position
            index (int): joint indicies to control
        Returns:
            q_cmd (np.array): desired joints angle for low-level controller 
            q_vel (np.array): desired joint velocities for low-level controller
            q_toq (np.array): desired feedforward torques for low-level contller
        """
        q_cmd, q_vel = self.inv_kinematics(cmd, index, mode = "PD")
        q_cmd = np.array(q_cmd).reshape(12)
        q_vel = np.array(q_vel).reshape(12)
        q_mes = np.zeros(12)
        v_mes = np.zeros(12)
        jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
        q_mes[:] = [state[0] for state in jointStates]
        v_mes[:] = [state[1] for state in jointStates]
        q_toq = self._motor.convert_to_torque(q_cmd, q_mes, v_mes)
        return q_cmd, q_vel, q_toq
        
    def inv_kinematics_multi(self, cmds, indices, mode = 'P'):
        assert(len(indices) == 4)
        breakpoint()
        joint_position = np.zeros(12)
        joint_velocity = np.zeros(12)
        idx_2_EE = {3 : "FL_FOOT", 7: "FR_FOOT", 11: "HL_FOOT", 15: "HR_FOOT"}
        for idx in indices:
            breakpoint()
            if idx_2_EE[idx] == "FL_FOOT":
                joint_position[0:3] = self.inv_kinematics(cmds[idx_2_EE[idx]], idx, mode)[0][0:3]
            elif idx_2_EE[idx] == "FR_FOOT":
                joint_position[3:6] = self.inv_kinematics(cmds[idx_2_EE[idx]], idx, mode)[0][3:6]
            elif idx_2_EE[idx] == "HL_FOOT":
                joint_position[6:9] = self.inv_kinematics(cmds[idx_2_EE[idx]], idx, mode)[0][6:9]
            elif idx_2_EE[idx] == "HR_FOOT":
                joint_position[9:12] = self.inv_kinematics(cmds[idx_2_EE[idx]], idx, mode)[0][9:12]
        self._joint_ang = joint_position
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

    def default_stance_control(self):
        """Robot default stance standing still
        Returns:
            q_cmd (np.array): Stance joint angles
            q_vel (np.array): Stance joint velocities
            q_toq (np.array): Stance motor torques
        """
        q_cmd = self.q_init
        q_vel = np.zeros(12)
        q_mes = np.zeros(12)
        v_mes = np.zeros(12)
        jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
        q_mes[:] = [state[0] for state in jointStates]
        v_mes[:] = [state[1] for state in jointStates]
        q_toq = self._motor.convert_to_torque(q_cmd, q_mes, v_mes)
        return q_cmd, q_vel, q_toq

            








