# Third party code
#
# The following code are copied or modified from:
# https://github.com/google-research/motion_imitation

import numpy as np

class MOTOR(object):
    NUM_MOTORS = 12 
    VOLTAGE_CLIPPING = 50 #based on A1 robot
    OBSERVED_TORQUE_LIMIT = 5.7 #based on A1 robot
    MOTOR_VOLTAGE = 16.0 #based on A1 robot
    MOTOR_RESISTANCE = 0.186 #based on A1 robot
    MOTOR_TORQUE_CONSTANT = 0.025 #based on solo12 robot
    MOTOR_VISCOUS_DAMPING = 0
    MOTOR_SPEED_LIMIT = MOTOR_VOLTAGE / (MOTOR_VISCOUS_DAMPING +
                                        MOTOR_TORQUE_CONSTANT)
    MOTOR_POS_LB = 0.5
    MOTOR_POS_UB = 2.5

class MotorModel(object):
    def __init__(self, kp=1.2, kd=0, motor_mode=None) -> None:
        """_summary_

        Args:
            kp (float, optional): _description_. Defaults to 1.2.
            kd (int, optional): _description_. Defaults to 0.
            motor_mode (_type_, optional): _description_. Defaults to None.
        """
        self._kp = kp
        self._kd = kd
        self._strength_ratio = [1.0] * MOTOR.NUM_MOTORS

    def set_motor_gains(self, kp, kd):
        """Set the motor postional and derivative gains

        Args:
            kp (float): Positioanl gain
            kd (float): Derivative gain
        """
        self._kp = kp
        self._kd = kd

    def convert_to_torque(self, motor_ang_cmd, motor_ang, motor_vel):
        """Convert the motor position signal to torque

        Args:
            motor_cmd (_type_): _description_
            motor_ang (_type_): _description_
            motor_vel (_type_): _description_
        """
        kp = self._kp
        kd = self._kd
        pwm = np.clip(-1 * kp  * (motor_ang - motor_ang_cmd) - kd * (motor_vel), -1, 1)
        return self._convert_to_torque_from_pwm(pwm)

    def _convert_to_torque_from_pwm(self, pwm):
        """converting a pwm signal to motor torque

        Args:
            pwm (_type_): _description_
            true_motor_vel (_type_): _description_
        """
        observed_torque = np.clip(MOTOR.MOTOR_TORQUE_CONSTANT * np.asarray(pwm) 
                                  * MOTOR.MOTOR_VOLTAGE / MOTOR.MOTOR_RESISTANCE, 
                                  -MOTOR.OBSERVED_TORQUE_LIMIT, MOTOR.OBSERVED_TORQUE_LIMIT)
        return observed_torque


        
        



        

    
    