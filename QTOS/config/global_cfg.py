import sys

import numpy as np

from QTOS.containers import FIFOQueue

class ROBOT_CFG:
    """
    Configuration class for robot-related variables and settings.

    This class stores various robot-related variables and settings, including joint states,
    global position, orientation, end-effector positions, runtime, and more.

    Attributes:
        robot: A reference to the robot object.
        joint_state: A dictionary containing joint state information with keys 'q_cmd', 'q_vel', and 'q_toq'.
        global_COM_xyz: A list representing the global center of mass coordinates [x, y, z].
        global_COM_ang: A list representing the global center of mass orientation in Euler angles [roll, pitch, yaw].
        last_POSE: A list representing the last pose of the robot [x, y, z].
        robot_goal: A list representing the goal position of the robot [x, y, z].
        EE: A dictionary containing end-effector positions with keys 'FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'.
        runtime: A float representing the runtime of the robot.
        state: A NumPy array representing the robot state.
        robot_states: A list to store robot state history.
    """
    robot = None
    joint_state = {'q_cmd': [0.0]*12, 'q_vel': [0.0]*12, 'q_toq': [0.0]*12}
    global_COM_xyz = [0.0, 0, 0.25]
    global_COM_ang = [0, 0, 0]
    last_POSE = [0, 0, 0.24]
    robot_goal = [0, 0, 0]
    EE = {"FL_FOOT": [0.20, 0.19, 0.0], "FR_FOOT": [0.20, -0.19, 0.0],
          "HL_FOOT": [-0.20, 0.19, 0.0] , "HR_FOOT": [-0.20, -0.19, 0.0]}
    runtime = 0.0
    state = np.zeros(19)
    robot_states = []

class RUN:
    """
    Configuration class for run-related variables and states.

    This class stores various run-related variables and states, including the current step,
    update state, wait state, stance state, update thread state, and done state.

    Attributes:
        step: An integer representing the current step number.
        _update: A boolean indicating the update state.
        _wait: A boolean indicating the wait state.
        _stance: A boolean indicating the stance state.
        _run_update_thread: A boolean indicating the state of the run update thread.
        _done: A boolean indicating the completion state.
    """
    step = 0
    _update = True
    _wait = False
    _stance = False
    _run_update_thread = True
    _done = False

class PLANNER:
    """
    Configuration class for planner-related variables and settings.

    This class stores various planner-related variables and settings, including the 'set_straight_correction'
    flag and a FIFO queue for 'mpc_goal_points'.
    """
    set_straight_correction = False
    mpc_goal_points = FIFOQueue()

def print_vars(stream=sys.__stdout__):
    """
    Print robot and run configuration variables.

    Args:
        output_stream: The output stream where the variables will be printed (default is sys.stdout).
    """
    def print_section(title, variables):
        print(f"========={title}==========", file=stream)
        for var_name, var_value in variables.items():
            print(f"{var_name}: {var_value}", file=stream)

    robot_cfg_vars = {
        "Global Position": ROBOT_CFG.global_COM_xyz,
        "GLOBAL Orientation": ROBOT_CFG.global_COM_ang,
        "Last known POSE": ROBOT_CFG.last_POSE,
        "Robot Goal": ROBOT_CFG.robot_goal,
        "EE": ROBOT_CFG.EE,
        "Robot Runtime": ROBOT_CFG.runtime
    }

    run_vars = {
        "STEP NUM": RUN.step,
        "UPDATE STATE": RUN._update,
        "STANCE STATE": RUN._stance,
        "WAIT STATE": RUN._wait,
        "RUN UPDATE THREAD STATE": RUN._run_update_thread,
        "DONE STATE": RUN._done
    }

    print_section("ROBOT_CFG GLOBAL VARS", robot_cfg_vars)
    print_section("RUN GLOBAL VARS", run_vars)
    print("\n", file=stream)