import time
import copy

#third-party libraries
import pybullet as p
import numpy as np
import pinocchio as pin

#QTOS modules
from QTOS.utils import *
from QTOS.robot.robot_motor import MotorModel
import QTOS.config.global_cfg as global_cfg
        
class SOLO12(object):
    def __init__(self, config, sim_cfg=None, loader=None):
        """
        Initialize the SOLO12 robot object.

        Args:
            URDF: The URDF file for the robot.
            config (dict): Configuration parameters for the robot.
            fixed (int, optional): Fixed parameter description. Defaults to 0.
            sim_cfg: Simulation configuration (if applicable).
            loader: Loader description (if applicable).
        """
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
        self.modes = {"P": p.POSITION_CONTROL, "PD": p.VELOCITY_CONTROL, "torque": p.TORQUE_CONTROL}
        self.mode = config['mode']
        self.sim_cfg = sim_cfg
        self.tfBaseMtx = transformation_inv(transformation_mtx(self.CoM_states()['linkWorldPosition'], self.CoM_states()['linkWorldOrientation']))
        self.EE_WORLD = {"FL_W_POSE": base_frame_tf(self.tfBaseMtx, self.get_endeffector_pose()['FL_FOOT']['linkWorldPosition']), "FR_W_POSE": base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['FR_FOOT']['linkWorldPosition']),
                                    "HL_W_POSE": base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['HL_FOOT']['linkWorldPosition']), "HR_W_POSE": base_frame_tf(self.tfBaseMtx , self.get_endeffector_pose()['HR_FOOT']['linkWorldPosition'])}
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
        self._joint_ang_ref = np.zeros(12)
        self._joint_vel_ref = np.zeros(12)
        self._joint_toq_ref = np.zeros(12)
        self._joint_state = None
        self._time_step = config['timestep']
        self.convergence_gain = 10
        self._update()

    def CoM_states(self):
        """
        Returns the Center of Mass (CoM) and orientation of the robot.

        Returns:
            dict: Dictionary containing CoM position and orientation.
        """
        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        return {"linkWorldPosition": CoM_pos, "linkWorldOrientation": CoM_angle}

    @property
    def state_np(self):
        """
        Get the robot's state as a NumPy array.

        Returns:
            np.array: NumPy array containing various robot state values.
        """
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
    def state(self):
        """
        Get the known state of the robot.

        Returns:
            dict: Dictionary containing CoM, orientation, and end-effector positions.
        """

        CoM_pos, CoM_angle = p.getBasePositionAndOrientation(self.robot)
        EE = self.get_endeffector_pose()
        return {"COM": CoM_pos, "linkWorldOrientation": CoM_angle, "FL_FOOT": EE['FL_FOOT']['linkWorldPosition'], 
                "FR_FOOT": EE['FR_FOOT']['linkWorldPosition'], "HL_FOOT": EE['HL_FOOT']['linkWorldPosition'], "HR_FOOT": EE['HR_FOOT']['linkWorldPosition']}
    
    @property
    def csv_entry(self):
        """Helper function to quickly extract robot joint states into csv
           format
        Returns:
            np.array: 36-idx array [12 ... joint angles 12 ... joint velocity 12 ... joint torques] 
        """
        csv_entry = np.hstack([self._joint_ang_ref, self._joint_vel_ref, self._joint_toq_ref])
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
        """
        Property to compute the estimated trajectory vector.
        
        Returns:
            numpy.ndarray: A 1D NumPy array representing the estimated trajectory vector.
        """
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
            q_toq = self._motor.convert_to_torque(q_cmd, q_mes, v_mes, q_vel)
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
        return joint_position, joint_velocity

    def calculate_rotation_jacobian(self, joint_ang, frames, cmd):
        """
        Calculate the rotation Jacobian for specific frames.

        This method computes the rotation Jacobian for a set of frames specified in the 'frames' list using the joint angles
        and desired end-effector positions provided in the 'cmd' dictionary.

        Parameters:
            joint_ang (numpy.ndarray): The joint angles of the robot.
            frames (list): A list of frame IDs for which the rotation Jacobian is calculated.
            cmd (dict): A dictionary containing desired end-effector positions for each frame.

        Returns:
            numpy.ndarray: A 2Nx12 numpy array representing the rotation Jacobian for each specified frame, where N is the number of frames.
        """
        J = np.zeros((8, 12))
        for i, frame_id in enumerate(frames):
            ID = self.ROBOT.model.getFrameId(frame_id)
            fJ = pin.computeFrameJacobian(self.ROBOT.model, self.ROBOT.data, joint_ang, ID)[:3, -12:]
            oJ = self.ROBOT.data.oMf[ID].rotation @ fJ
            J[2*i:2*(i+1), -12:] = oJ[0:2, -12:]
        return J

    def calculate_xyz_error(self, frames, cmd):
        """
        Calculate the XYZ error for specific frames.

        This method computes the XYZ error for a set of frames specified in the 'frames' list by comparing the desired positions
        in the 'cmd' dictionary with the actual positions obtained from Pinocchio.

        Parameters:
            frames (list): A list of frame IDs for which XYZ error is calculated.
            cmd (dict): A dictionary containing desired end-effector positions for each frame.

        Returns:
            numpy.ndarray: A 2Nx1 numpy array containing the XYZ error for each specified frame, where N is the number of frames.
        """
        xyz_cmd = np.zeros((8, 1))
        for i, frame_id in enumerate(frames):
            ID = self.ROBOT.model.getFrameId(frame_id)
            frame_translation = self.ROBOT.data.oMf[ID].translation
            x = np.reshape(cmd[frame_id]['P'][0:2] - frame_translation[0:2], (2, 1))
            xyz_cmd[2*i:2*(i + 1)] = x
        return xyz_cmd

    def calculate_joint_commands(self, J, nu):
        """
        Calculate joint-space velocity command.

        Parameters:
            J (numpy.ndarray): The Jacobian matrix.
            nu (numpy.ndarray): The error vector.

        Returns:
            numpy.ndarray: A 12-element numpy array representing the joint velocity commands.
        """
        return (self.convergence_gain * np.linalg.pinv(J) @ nu).reshape(12 )
    
    def integrate_joint_commands(self, joint_ang, q_dot_cmd, time_step):
        """
        Integrate joint commands to obtain joint position commands.

        Parameters:
            robot (pinocchio.RobotWrapper): The robot model.
            joint_ang (numpy.ndarray): The current joint angles.
            q_dot_cmd (numpy.ndarray): The joint velocity commands.
            time_step (float): The time step.

        Returns:
            numpy.ndarray: A 12-element numpy array representing the joint position commands.
        """
        return pin.integrate(self.ROBOT.model, joint_ang, q_dot_cmd * time_step)

    def inv_kinematics_pin(self, cmd):
        """
        Perform inverse kinematics using Pinocchio library for controlling end-effector positions.

        This method calculates joint commands to achieve the desired end-effector positions specified in the 'cmd' dictionary.

        Parameters:
            cmd (dict): A dictionary containing desired end-effector positions for each foot.

        Returns:
            Tuple: A tuple containing joint commands (q_cmd) and joint velocity commands (q_dot_cmd) to achieve the desired positions.

        Notes:
            This method performs the following steps:
            1. Computes the current end-effector positions using the Pinocchio library.
            2. Calculates the position errors between the current and desired end-effector positions.
            3. Computes the Jacobian matrices for each end-effector.
            4. Solves for joint velocity commands using the calculated Jacobians and position errors.
            5. Integrates joint velocity commands to obtain joint position commands.
            
            The 'cmd' dictionary should have keys 'FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT', each containing 'P' (position) information.
        
        Returns:
            Tuple: A tuple containing joint commands (q_cmd) and joint velocity commands (q_dot_cmd) to achieve the desired positions.

        """
        pin.forwardKinematics(self.ROBOT.model, self.ROBOT.data, self._joint_ang)
        pin.updateFramePlacements(self.ROBOT.model, self.ROBOT.data)
        nu = np.vstack(self.calculate_xyz_error(self.EE.keys(), cmd))
        J = np.vstack(self.calculate_rotation_jacobian(self._joint_ang, self.EE.keys(), cmd))
        q_cmd_dot = self.calculate_joint_commands(J, nu)
        q_cmd = self.integrate_joint_commands(self._joint_ang, q_cmd_dot, self._time_step)
        return q_cmd, q_cmd_dot

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
        else:
            if self.mode == p.POSITION_CONTROL or self.mode == p.VELOCITY_CONTROL:
                q_cmd = self.q_init * 1.5
                q_vel = np.zeros(12)
                q_mes, v_mes = self.get_PD_values()
                self._motor.set_motor_gains(10, 0.05)
                q_toq = self._motor.convert_to_torque(q_cmd, q_mes, v_mes, q_vel)
            elif self.ROBOT:
                q_cmd = q_init 
                q_vel = np.zeros(12)
                q_mes, v_mes = self.get_PD_values()
                self._motor.set_motor_gains(5, 0.1)
                q_toq = self._motor.convert_to_torque(q_cmd, q_mes, v_mes, q_vel)
        self._joint_ang_ref = q_cmd
        self._joint_vel_ref = q_vel
        self._joint_toq_ref = q_toq
        return q_cmd, q_vel, q_toq

    def update(self):
        """Updates the global varaibles corresponding to robot
        """
        global_cfg.ROBOT_CFG.state = self.state_np

    def _update(self):
        """Current joint angles, joint velocity, base state, and base velocity of the robot 
            in the bullet simulator.
        """
        base_state = p.getBasePositionAndOrientation(self.robot)
        base_vel = p.getBaseVelocity(self.robot)
        self._joint_state = p.getJointStates(self.robot, self.jointidx['idx'])
        self._joint_toq = np.array([state[3] for state in self._joint_state])
        self._joint_ang = np.array([state[0] for state in self._joint_state])
        self._joint_vel = np.array([state[1] for state in self._joint_state])
        