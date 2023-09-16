# Third party code
#
# The following code are copied or modified from:
# https://github.com/google-research/motion_imitation

import numpy as np


class MOTOR(object):
    NUM_MOTORS = 12 
    VOLTAGE_CLIPPING = 50 #based on A1 robot
    OBSERVED_TORQUE_LIMIT = 3.0 #based on SOLO12 Robot
    MOTOR_VOLTAGE = 16.0 #based on A1 robot
    MOTOR_RESISTANCE = 0.186 #based on A1 robot
    MOTOR_TORQUE_CONSTANT = 0.025 #based on solo12 robot
    MOTOR_VISCOUS_DAMPING = 0
    MOTOR_SPEED_LIMIT = MOTOR_VOLTAGE / (MOTOR_VISCOUS_DAMPING +
                                        MOTOR_TORQUE_CONSTANT)
    MOTOR_POS_LB = 0.5
    MOTOR_POS_UB = 2.5

    GAINS_REF = {"FL_HIP": 35, "FL_ELBOW": 25, "FL_ANKLE": 25, "FR_HIP": 35, "FR_ELBOW": 25, "FR_ANKLE": 25,
         "HL_HIP": 35, "HL_ELBOW": 25, "HL_ANKLE": 25, "HR_HIP": 35, "HR_ELBOW": 25, "HR_ANKLE": 25}

    GAINS_P = [GAINS_REF['FL_HIP'], GAINS_REF['FL_ELBOW'], GAINS_REF['FL_ANKLE'],
         GAINS_REF['FR_HIP'], GAINS_REF['FR_ELBOW'], GAINS_REF['FR_ANKLE'],
         GAINS_REF['HL_HIP'], GAINS_REF['HL_ELBOW'], GAINS_REF['HL_ANKLE'],
         GAINS_REF['HR_HIP'], GAINS_REF['HR_ELBOW'], GAINS_REF['HR_ANKLE']]

class MotorModel(object):
    def __init__(self, kp=1.2, kd=0, hip_scale=1.0, knee_scale=1.0, ankle_scale=1.0, motor_mode=None, toq_max=None) -> None:
        """
        Initialize a MotorModel object.

        Parameters:
            kp (float, optional): Proportional gain for the motor model. Defaults to 1.2.
            kd (int, optional): Derivative gain for the motor model. Defaults to 0.
            hip_scale (float, optional): Scaling factor for hip motors. Defaults to 1.0.
            knee_scale (float, optional): Scaling factor for knee motors. Defaults to 1.0.
            ankle_scale (float, optional): Scaling factor for ankle motors. Defaults to 1.0.
            motor_mode (type, optional): Description of motor mode (if applicable). Defaults to None.
            toq_max (type, optional): Maximum torque limit (if applicable). Defaults to None.
        """
        self.hip_scale = hip_scale
        self.knee_scale = knee_scale
        self.ankle_scale = ankle_scale
        self._kp = self.UPDATE_GAIT(kp, hip_scale, knee_scale, ankle_scale)
        self._kd = self.UPDATE_GAIT(kd, hip_scale, knee_scale, ankle_scale)
        self._strength_ratio = [1.0] * MOTOR.NUM_MOTORS
        if toq_max:
            MOTOR.OBSERVED_TORQUE_LIMIT = toq_max

    def set_motor_gains(self, kp, kd):
        """
        Set the motor positional and derivative gains.

        Parameters:
            kp (float): Proportional gain for the motor model.
            kd (float): Derivative gain for the motor model.
        """
        self._kp = kp
        self._kd = kd

    def convert_to_torque(self, motor_ang_cmd, motor_ang, motor_vel, motor_vel_cmd=0.0):
        """
        Convert the motor position signal to torque.

        Parameters:
            motor_ang_cmd (type): Description of the motor angle command.
            motor_ang (type): Description of the current motor angle.
            motor_vel (type): Description of the current motor velocity.
        """
        kp = self._kp
        kd = self._kd
        desired_motor_angle = motor_ang_cmd
        desired_motor_velocities = motor_vel_cmd
        motor_torque = kp * (desired_motor_angle - motor_ang) + kd * (desired_motor_velocities - motor_vel)

        return np.clip(motor_torque, -1.0 * MOTOR.OBSERVED_TORQUE_LIMIT, MOTOR.OBSERVED_TORQUE_LIMIT)

    def convert_to_torque_ff(self, motor_ang_cmd, motor_ang, motor_vel, motor_vel_cmd, toq_ff):
        """
        Convert the motor position signal to torque with feedforward term.

        Parameters:
            motor_ang_cmd (type): Description of the motor angle command.
            motor_ang (type): Description of the current motor angle.
            motor_vel (type): Description of the current motor velocity.
            motor_vel_cmd (type): Description of the motor velocity command.
            toq_ff (type): Description of the feedforward torque term.
        """
        kp = self._kp
        kd = self._kd
        desired_motor_angle = motor_ang_cmd
        desired_motor_velocities = motor_vel_cmd
        motor_torque = kp * (desired_motor_angle - motor_ang) + kd * (desired_motor_velocities - motor_vel) + toq_ff
        return np.clip(motor_torque, -1.0 * MOTOR.OBSERVED_TORQUE_LIMIT, MOTOR.OBSERVED_TORQUE_LIMIT)

    def _convert_to_torque_from_pwm(self, pwm):
        """
        Convert a PWM signal to motor torque.

        Parameters:
            pwm (type): Description of the PWM signal.
        """
        observed_torque = np.clip(MOTOR.MOTOR_TORQUE_CONSTANT * np.asarray(pwm) 
                                  * MOTOR.MOTOR_VOLTAGE / MOTOR.MOTOR_RESISTANCE, 
                                  -MOTOR.OBSERVED_TORQUE_LIMIT, MOTOR.OBSERVED_TORQUE_LIMIT)
        return observed_torque

    @classmethod
    def UPDATE_GAIT(self, gain, hip_scale=2.0, knee_scale=1.0, ankle_scale=1.0):
        """
        Update gait gains.

        Parameters:
            gain: The gain to be updated.
            hip_scale (float, optional): Scaling factor for hip motors. Defaults to 2.0.
            knee_scale (float, optional): Scaling factor for knee motors. Defaults to 1.0.
            ankle_scale (float, optional): Scaling factor for ankle motors. Defaults to 1.0.

        Returns:
            gains: Updated gains.
        """
        gains = np.ones(MOTOR.NUM_MOTORS) * gain
        for i in (0, 3, 6, 9):
            gains[i] *= hip_scale
        for i in (1, 4, 7, 10):
            gains[i] *= knee_scale
        for i in (2, 5, 8, 11):
            gains[i] *= ankle_scale
        return gains




        




        
        



        

    
    