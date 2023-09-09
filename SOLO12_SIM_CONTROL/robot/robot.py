import time
import copy

import pybullet as p
import numpy as np
import pinocchio as pin

from SOLO12_SIM_CONTROL.utils import transformation_mtx, transformation_inv, convert12arr_2_16arr, create_cmd, trajectory_2_local_frame, EE_NAMES
from SOLO12_SIM_CONTROL.robot.robot_motor import MotorModel
from SOLO12_SIM_CONTROL.stateEstimation import State_Estimation
import SOLO12_SIM_CONTROL.config.global_cfg as global_cfg


def links_to_id(robot):
    """Helper function to retrieve the joint info of a robot into a dictionary

    Args:
        robot (pybullet obj): pybullet robot object

    Returns:
        dict: dictionary filled with corresponding link name and id
    """
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
    """Helper function to transform a vec from one frame to another

    Args:
        mtx (np.array): transformation matrix
        pt (np.array): position vector

    Returns:
        np.array: transformed position vector 
    """
    vec = np.concatenate((np.array([pt[0]]), np.array([pt[1]]),np.array([pt[2]]), np.ones(1)))
    tf_vec = mtx @ vec
    return tf_vec[:3]

def shift_z(v, shift):
    """Helper function to shift bezier curve into fix position and adjust the z-axis variable

    Args:
        v (np.array or list): 3d vec to shift
        shift (float or int): scalar value to shift the z-coord

    Returns:
        np.array or list: resulting 3d vec from shift
    """
    v[2] += shift
    return v
        
class SOLO12(object):
    def __init__(self, URDF, config, fixed = 0, sim_cfg=None, loader=None):
        self.config = config
        self.ROBOT = loader
        self.robot = self.ROBOT.robot
        self.jointidx = {"FL": [0, 1, 2], "FR": [4, 5, 6], "BL": [8, 9, 10], "BR": [12, 13, 14], "idx": [0,1,2,4,5,6,8,9,10,12,13,14]}
        self.fixjointidqx = {"FL": 3, "FR": 7, "BL": 11, "BR": 15, "idx": [3,7,11,15]}
        self.links = links_to_id(self.robot)
        self.q_init = np.array(config['q_init'])
        self.q_init16 = q_init_16_arr(self.q_init)
        self.EE = {'FL_FOOT': None, 'FR_FOOT': None, "HL_FOOT": None, "HR_FOOT": None}
        self.EE_index = {'FL_FOOT': 3, 'FR_FOOT': 7, "HL_FOOT": 11, "HR_FOOT": 15, 'all': (3, 7, 11, 15)}
        self.time_step = 0
        self.t_max = config['t_max']
        self.modes = {"P": p.POSITION_CONTROL, "PD": p.VELOCITY_CONTROL, "torque": p.TORQUE_CONTROL}
        self.mode = config['mode']
        self.sim_cfg = sim_cfg
        self.tfBaseMtx = transformation_inv(transformation_mtx(self.CoM_states()['linkWorldPosition'], self.CoM_states()['linkWorldOrientation']))
        self.EE_WORLD = {"FL_W_POSE": base_frame_tf(self.tfBaseMtx, self.get_endeffector_pose()['FL_FOOT']['linkWorldPosition']), "FR_W_POSE": base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['FR_FOOT']['linkWorldPosition']),
                                    "HL_W_POSE": base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['HL_FOOT']['linkWorldPosition']), "HR_W_POSE": base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['HR_FOOT']['linkWorldPosition'])}
        if sim_cfg['mode'] == "bezier":
            self.shiftZ = 0.12
            self.shift = {'FL_FOOT': shift_z(self.EE_WORLD['FL_W_POSE'], self.shiftZ), 'FR_FOOT': shift_z(self.EE_WORLD['FR_W_POSE'], self.shiftZ),
                     'HL_FOOT': shift_z(self.EE_WORLD['HL_W_POSE'], self.shiftZ), 'HR_FOOT': shift_z(self.EE_WORLD['HR_W_POSE'], self.shiftZ)}
        elif sim_cfg['mode'] == "towr":
            self.shiftZ = 0.00
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
        
        revoluteJointIndices = self.jointidx['idx']
        # Enable torque control for revolute joints
        jointTorques = [0.0 for m in revoluteJointIndices]
        p.setJointMotorControlArray(self.robot, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)
        self.offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._kp = config['kp']
        self._kd = config['kd']
        self._motor = MotorModel(self._kp, self._kd, config['hip_gain_scale'], config['knee_gain_scale'], config['ankle_gain_scale'], toq_max=config['t_max'])
        self._joint_ang = np.zeros(12)
        self._joint_vel = np.zeros(12)
        self._joint_toq = np.zeros(12)
        self._joint_ang_ref = None
        self._joint_vel_ref = None
        self._joint_toq_ref = None
        self._joint_state = None
        self._time_step = config['timestep']
        self.q_ref = self.q_init
        self.stateEstimator = State_Estimation(config['start_pos'])
        self.CoM_acceleration = np.array([0, 0, 0])
        self.CoM_vel = np.array([0, 0, 0])
        self._update()


    def CoM_states(self):
        """Returns the Center of mass and orientation of robot

        Returns:
            dict: The CoM and orientation of robot
        """
        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        return {"linkWorldPosition": CoM_pos, "linkWorldOrientation": CoM_angle}

    @property
    def state_np(self):
        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        EE = self.get_endeffector_pose()
        CoM_pos = np.array(CoM_pos)
        CoM_angle = p.getEulerFromQuaternion(np.array(CoM_angle))
        EE_1 = np.array(EE['FL_FOOT']['linkWorldPosition'])
        EE_2 = np.array(EE['FR_FOOT']['linkWorldPosition'])
        EE_3 = np.array(EE['HL_FOOT']['linkWorldPosition'])
        EE_4 = np.array(EE['HR_FOOT']['linkWorldPosition'])
        time_step = np.array(self.time)
        state = np.hstack((time_step, CoM_pos, CoM_angle, EE_1, EE_2, EE_3, EE_4))
        return state

    @property
    def state_np_estimated(self):
        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        EE = self.get_endeffector_pose()
        CoM_pos = self.state_estimated['COM']
        CoM_angle = p.getEulerFromQuaternion(np.array(CoM_angle))
        EE_1 = np.array(EE['FL_FOOT']['linkWorldPosition'])
        EE_2 = np.array(EE['FR_FOOT']['linkWorldPosition'])
        EE_3 = np.array(EE['HL_FOOT']['linkWorldPosition'])
        EE_4 = np.array(EE['HR_FOOT']['linkWorldPosition'])
        time_step = np.array(self.time)
        state = np.hstack((time_step, CoM_pos, CoM_angle, EE_1, EE_2, EE_3, EE_4))
        return state
     
    @property
    def state(self):
        """Return the known state of the robot

        Returns:
            dict: return the current CoM, orientation, endeffector positions
        """
        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        EE = self.get_endeffector_pose()
        return {"COM": CoM_pos, "linkWorldOrientation": CoM_angle, "FL_FOOT": EE['FL_FOOT']['linkWorldPosition'], 
                "FR_FOOT": EE['FR_FOOT']['linkWorldPosition'], "HL_FOOT": EE['HL_FOOT']['linkWorldPosition'], "HR_FOOT": EE['HR_FOOT']['linkWorldPosition']}
    
    @property
    def state_estimated(self):
        """Return the known state of the robot

        Returns:
            dict: return the current CoM, orientation, endeffector positions
        """
        timestep = self.time
        lin_vel, ang_vel = p.getBaseVelocity(self.robot)
        updated_accelation = np.array([lin_vel[0] - self.CoM_vel[0], lin_vel[1] - self.CoM_vel[1], lin_vel[2] - self.CoM_vel[2]])
        self.CoM_acceleration = updated_accelation
        self.stateEstimator.update(updated_accelation, self.time)
        CoM_pos, CoM_vel = self.stateEstimator.CoM, self.stateEstimator.CoM_vel
        self.CoM_vel = CoM_vel
        return {"COM": CoM_pos, "COM_VEL": CoM_vel}

    @property
    def csv_entry(self):
        """Helper function to quickly extract robot joint states into csv
           format
        Returns:
            np.array: 36-idx array [12 ... joint angles 12 ... joint velocity 12 ... joint torques] 
        """
        csv_entry = np.hstack([self._joint_ang, self._joint_vel, self._joint_toq])
        return csv_entry

    @property
    def jointangles(self):
        """The current joint angle of each motor

        Returns:
            np.array: 12-idx array of joint angles
        """
        return self._joint_ang

    @property
    def traj_vec(self):
        """Converts the robot's current endeffector global position values into 12D np.array

        Returns:
            np.array: numpy array of end effector positions
        """
        EE = self.get_endeffector_pose()
        CoM = self.CoM_states()
        traj_vec = np.concatenate((CoM['linkWorldPosition'],  p.getEulerFromQuaternion(CoM['linkWorldOrientation']), EE['FL_FOOT']['linkWorldPosition'], EE['FR_FOOT']['linkWorldPosition'], EE['HL_FOOT']['linkWorldPosition'], EE['HR_FOOT']['linkWorldPosition']))
        return traj_vec

    @property
    def traj_vec_estimated(self):
        EE = self.get_endeffector_pose()
        CoM = self.state_estimated
        traj_vec = np.concatenate((CoM['COM'],  np.zeros(3), EE['FL_FOOT']['linkWorldPosition'], EE['FR_FOOT']['linkWorldPosition'], EE['HL_FOOT']['linkWorldPosition'], EE['HR_FOOT']['linkWorldPosition']))
        return traj_vec


    @property
    def jointstate(self):
        """Returns the joint angle, velocity, and torque of each motor

        Returns:
            dict: 12-idx array of joint angle, joint velocity, and joint torque
        """
        self._update()
        return {'q_cmd': self._joint_ang, "q_vel": self._joint_vel, "q_toq": self._joint_toq}

    @property
    def time(self):
        """Internal run time for robot based on simulation frequency

        Returns:
            float: runtime since start of simulation
        """
        return self.time_step * self._time_step

    def set_joint_control(self, jointsInx, controlMode, cmd_pose, cmd_vel=None, cmd_toq=None):
        """Function to interface with joint state for the bullet engine

        Args:
            jointsInx (list[int]): the joints in effect in the bullet simulator
            controlMode (string): the control scheme to use
            cmd_pose (tuple(floats)): desired joint state
            cmd_vel (tuple(floats)): desired joint velocity state. Defaults to None.
            cmd_toq (tuple(floats)): desired torque state. Defaults to None.
        """
        if 'P' == controlMode or 'PD' == controlMode:
            posGains = np.ones(len(jointsInx))*self._kp
            p.setJointMotorControlArray(self.robot, jointsInx, self.modes['P'], cmd_pose, positionGains=posGains)
        elif 'PD' == controlMode:
            posGains = np.ones(len(jointsInx))*self._kp
            velGains = np.ones(len(jointsInx))*self._kd
            p.setJointMotorControlArray(self.robot, jointsInx, self.modes['PD'], cmd_pose, cmd_vel, positionGains=posGains, velocityGains=velGains)
        elif 'torque' == controlMode:
            p.setJointMotorControlArray(self.robot, jointsInx, controlMode=p.TORQUE_CONTROL, forces=cmd_toq)

    def set_joint_control_multi(self, indices, controlMode, cmd_q, cmd_vel=None, cmd_toq=None):
        """Function to interface with joint states within bullet engine

        Args:
            jointsInx (list[int]): the joints in effect in the bullet simulator
            controlMode (string): the control scheme to use
            cmd_pose (tuple(floats)): desired joint state
            cmd_vel (tuple(floats)): desired joint velocity state. Defaults to None.
            cmd_toq (tuple(floats)): desired torque state. Defaults to None.
        """
        cmd_q += self.offsets #adding slight offsets to joint angles
        for i in range(0, 12, 3):
            idx = indices[i:i+3]
            q_cmd_temp = cmd_q[i:i+3]
            vel_cmd_temp = cmd_vel[i:i+3]
            toq_cmd_temp = cmd_toq[i:i+3]
            self.set_joint_control(idx, controlMode, q_cmd_temp, vel_cmd_temp, toq_cmd_temp)

    def get_endeffector_pose(self):
        """Returns the current pose of each endeffector

        Returns:
            dict(np.array): The current 6d vec relating to the global position and orientation of each endeffector
        """
        EE1, EE2, EE3, EE4 = p.getLinkStates(self.robot,  self.EE_index['all'])
        return {"FL_FOOT": link_info(EE1), "FR_FOOT": link_info(EE2), "HL_FOOT": link_info(EE3), "HR_FOOT": link_info(EE4)}
    
    def control(self, cmd, index, mode="P"):
        """Function that receives desired command in a particular control mode and
           returns the resulting joint position, joint velocity, and joint torque
           for each endeffector

        Args:
            cmd (dict): dict of desired position, vel, toq of each endeffector
            index (int): the index of the endeffector
            mode (str, optional): The control scheme of the robot. Defaults to "P".

        Returns:
            tuple(np.array): returns a tuple of realized joint angle, joint velocity, and joint torques
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

    def end_effector_correction(self, desired_EE, measured_EE):
        pass

    def control_multi(self, cmds, indices, mode="P", usePin = False):
        """Helper function to find the control outputs for each end-effector

        Args:
            cmds (dict): dict of each endeffector ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"] desired command
            indices (list): list of index position of each endeffector
            mode (str, optional): the control mode of the robot ['P', 'PD', 'torque']. Defaults to "P".

        Returns:
            tuple (np.array): returns a tuple of joint angle, joint velocity, joint torque commands
        """
        
        EE_cmds = {key : value for key, value in cmds.items() if key in EE_NAMES}
        self._update()
        if self.config['use_pinocchio'] and (mode == "P" or mode == "PD"):
            q_cmd = np.zeros(12)
            q_vel = np.zeros(12)
            q_toq = np.zeros(12)
            i = 0
            for cmd, idx in zip(EE_cmds.values(), indices):
                q_cmd_temp, q_vel_temp, q_toq_temp = self.control(cmd, idx, mode)
                if (q_cmd_temp is not None):
                    q_cmd[i:i+3] = q_cmd_temp[i:i+3]
                i += 3
            cmds_local = copy.deepcopy(EE_cmds)
            trajectory_2_local_frame(self, EE_cmds)
            q_cmd_pin, q_vel = self.inv_kinematics_pin(cmds_local) 
            q_cmd += (q_cmd - q_cmd_pin)
        elif self.config['use_pinocchio'] and mode == "torque":
            q_cmd = np.zeros(12)
            q_vel = np.zeros(12)
            q_toq = np.zeros(12)
            i = 0
            for cmd, idx in zip(EE_cmds.values(), indices):
                q_cmd_temp, q_vel_temp, q_toq_temp = self.control(cmd, idx, mode)
                if (q_cmd_temp is not None):
                    q_cmd[i:i+3] = q_cmd_temp[i:i+3]
                i += 3
            cmds_local = copy.deepcopy(EE_cmds)
            trajectory_2_local_frame(self, cmds_local)
            q_cmd_pin, q_vel = self.inv_kinematics_pin(cmds_local) 
            q_cmd += (q_cmd - q_cmd_pin)
            self._update()
            q_mes, v_mes = self.get_PD_values()
            q_toq = self._motor.convert_to_torque_v1(q_cmd, q_mes, v_mes, q_vel)
        else:
            q_cmd = np.zeros(12)
            q_vel = np.zeros(12)
            q_toq = np.zeros(12)
            i = 0
            for cmd, idx in zip(EE_cmds.values(), indices):
                q_cmd_temp, q_vel_temp, q_toq_temp = self.control(cmd, idx, mode)
                if (q_cmd_temp is not None):
                    q_cmd[i:i+3] = q_cmd_temp[i:i+3]
                if (q_vel_temp is not None):
                    q_vel[i:i+3] = q_vel_temp[i:i+3]
                if (q_toq_temp is not None):
                    q_toq[i:i+3] = q_toq_temp[i:i+3]
                i += 3
        self._joint_ang_ref = q_cmd
        self._joint_vel_ref = q_vel
        self._joint_toq_ref = q_toq
        return q_cmd, q_vel, q_toq

    def get_PD_values(self):
        """Retrieves the observed joint angle and joint velocity used for PD torque controller

        Returns:
            tuple(np.array, np.array): measured joint angle and joint velocity
        """
        self._update()
        observation_P = []
        observation_P.extend(self._joint_ang)
        observation_D = []
        observation_D.extend(self._joint_vel)
        return (np.array(observation_P), np.array(observation_D))

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
        self._update()
        q_mes, v_mes = self.get_PD_values()
        q_cmd, q_vel = self.inv_kinematics(cmd, index, mode = "PD")
        q_toq = self._motor.convert_to_torque(q_cmd, q_mes, v_mes)
        return q_cmd, q_vel, q_toq

    def inv_dynamics_pin(self, q_cmd, q_vel_cmd, q_toq_cmd):
        """Template for Pinocchio based inverse dynamics [NOT DONE]

        Raises:
            NotImplementedError: _description_
        """
        
        #computing mass matrix

        M = pin.crba(self.ROBOT.model, self.ROBOT.data, self._joint_ang)
        toq = pin.rnea(self.ROBOT.model, self.ROBOT.data, self._joint_ang, self._joint_vel, self._joint_acc)

        Kp_tau = 8.
        Kd_tau = 0.2
        torques_ref = -Kp_tau * (q_cmd - self._joint_ang) - Kd_tau * q_vel_cmd

        self.q_dot2_ref = np.linalg.inv(M[6:, 6:]) * (torques_ref - toq[6:])

        q_vel_cmd += self.q_dot2_ref * self._time_step
        q_toq = self._motor.convert_to_torque_ff(q_cmd, self._joint_ang, self._joint_vel, q_vel_cmd, toq)

        return q_cmd, q_vel_cmd, q_toq
        
    def inv_kinematics_multi(self, cmds, indices, mode = 'P'):
        """Helper function that utilizes bullet inverse kinematics algorithm to find the optimal
           joint position and joint velocity for multiple desired command

        Args:
            cmd (dict): dict filled with desired position, and velocity commands
            index (int): the indices relating to the actuating joint
            mode (str, optional): the control scheme used. Defaults to 'P'.

        Returns:
            tuple (float): returns the resulting joint positions and velocities from the inverse kinematics algorithm
        """
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
        return joint_position

    def inv_kinematics(self, cmd, index, mode = 'P'):
        """Helper function that utilizes bullet inverse kinematics algorithm to find the optimal
           joint position and joint velocity for a single desired command

        Args:
            cmd (dict): dict filled with desired position, and velocity command
            index (int): the index relating to the actuating joint
            mode (str, optional): the control scheme used. Defaults to 'P'.

        Returns:
            tuple (float): returns the resulting joint position and velocity from the inverse kinematics algorithm
        """
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
        self._joint_ang_ref = joint_position #COULD BE POTENTIALY NOT 100% accurate if we update joint 4 but we want 1 (old joint reference)
        self._joint_vel_ref = joint_velocity
        return joint_position, joint_velocity

    def inv_kinematics_pin(self, cmd):
        """Template for Pinocchio based inverse kinematics [NOT DONE]

        Raises:
            NotImplementedError: _description_
        """

        K = 10.

        ID_FL = self.ROBOT.model.getFrameId("FL_FOOT")
        ID_FR = self.ROBOT.model.getFrameId("FR_FOOT")
        ID_HL = self.ROBOT.model.getFrameId("HL_FOOT")
        ID_HR = self.ROBOT.model.getFrameId("HR_FOOT")

        pin.forwardKinematics(self.ROBOT.model, self.ROBOT.data, self._joint_ang) #Update the state of robot for Pinocchio
        pin.updateFramePlacements(self.ROBOT.model, self.ROBOT.data)

        xyz_FL = self.ROBOT.data.oMf[ID_FL].translation
        xyz_FR = self.ROBOT.data.oMf[ID_FR].translation
        xyz_HL = self.ROBOT.data.oMf[ID_HL].translation
        xyz_HR = self.ROBOT.data.oMf[ID_HR].translation

        err_FL = np.reshape(xyz_FL[0:2] - cmd['FL_FOOT']['P'][0:2], (2, 1))
        err_FR = np.reshape(xyz_FR[0:2] - cmd['FR_FOOT']['P'][0:2], (2, 1))
        err_HL = np.reshape(xyz_HL[0:2] - cmd['HL_FOOT']['P'][0:2], (2, 1))
        err_HR = np.reshape(xyz_HR[0:2] - cmd['HR_FOOT']['P'][0:2], (2, 1))

        o_FL = self.ROBOT.data.oMf[ID_FL].rotation
        o_FR = self.ROBOT.data.oMf[ID_FR].rotation
        o_HL = self.ROBOT.data.oMf[ID_HL].rotation
        o_HR = self.ROBOT.data.oMf[ID_HR].rotation

        fJ_FL3 = pin.computeFrameJacobian(self.ROBOT.model, self.ROBOT.data, self._joint_ang, ID_FL)[:3, -12:]  # Take only the translation terms
        oJ_FL3 = o_FL @ fJ_FL3  #Transforms to rotation to global frame
        oJ_FLxz = oJ_FL3[0:2, -12:]

        fJ_FR3 = pin.computeFrameJacobian(self.ROBOT.model, self.ROBOT.data, self._joint_ang, ID_FR)[:3, -12:]
        oJ_FR3 = o_FR @ fJ_FR3
        oJ_FRxz = oJ_FR3[0:2, -12:]

        fJ_HL3 = pin.computeFrameJacobian(self.ROBOT.model, self.ROBOT.data, self._joint_ang, ID_HL)[:3, -12:]
        oJ_HL3 = o_HL @ fJ_HL3
        oJ_HLxz = oJ_HL3[0:2, -12:]

        fJ_HR3 = pin.computeFrameJacobian(self.ROBOT.model, self.ROBOT.data, self._joint_ang, ID_HR)[:3, -12:]
        oJ_HR3 = o_HR @ fJ_HR3
        oJ_HRxz = oJ_HR3[0:2, -12:]

        nu = np.vstack([err_FL, err_FR, err_HL, err_HR])
        J = np.vstack([oJ_FLxz, oJ_FRxz, oJ_HLxz, oJ_HRxz])
        q_dot_cmd = - K * np.linalg.pinv(J) @ nu
        q_dot_cmd = np.reshape(q_dot_cmd, (12, ))

        q_cmd = pin.integrate(self.ROBOT.model, self._joint_ang, q_dot_cmd * self._time_step)

        return q_cmd, q_dot_cmd

    def default_stance_control(self, q_init = None):
        """Robot default stance standing still

        Returns:
            q_cmd (np.array): Stance joint angles
            q_vel (np.array): Stance joint velocities
            q_toq (np.array): Stance motor torques
        """
        if q_init is None:
            q_cmd = self.q_init
            q_vel = np.zeros(12)
            q_mes = np.zeros(12)
            v_mes = np.zeros(12)
            jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
            q_mes[:] = [state[0] for state in jointStates]
            v_mes[:] = [state[1] for state in jointStates]
            self._motor.set_motor_gains(5, 0.01)
            q_toq = self._motor.convert_to_torque(q_cmd, q_mes, v_mes)
            self._joint_ang_ref = q_cmd
            self._joint_vel_ref = q_vel
            self._joint_toq_ref = q_toq
        else:
            q_cmd = q_init
            q_vel = np.zeros(12)
            q_mes = np.zeros(12)
            v_mes = np.zeros(12)
            jointStates = p.getJointStates(self.robot, self.jointidx['idx'])
            q_mes[:] = [state[0] for state in jointStates]
            v_mes[:] = [state[1] for state in jointStates]
            self._motor.set_motor_gains(5, 0.01)
            q_toq = self._motor.convert_to_torque(q_cmd, q_mes, v_mes)
            self._joint_ang_ref = q_cmd
            self._joint_vel_ref = q_vel
            self._joint_toq_ref = q_toq

        return q_cmd, q_vel, q_toq

    def update(self):
        """Updates the global varaibles corresponding to robot
        """
        global_cfg.ROBOT_CFG.state = self.state_np
        # global_cfg.ROBOT_CFG.state = self.state_np_estimated #state estimation of robot state


    def _update(self):
        """Current joint angles, joint velocity, base state, and base velocity of the robot 
            in the bullet simulator.
        """
        base_state = p.getBasePositionAndOrientation(self.robot)
        base_vel = p.getBaseVelocity(self.robot)
        self._joint_state = p.getJointStates(self.robot, self.jointidx['idx'])
        self._joint_ang = np.array([state[0] for state in self._joint_state])
        self._joint_vel = np.array([state[1] for state in self._joint_state])
        