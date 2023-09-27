import os
import yaml
import pybullet as p
import numpy as np
import math
import csv
import copy
import subprocess
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation

EE_NAMES = ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT')

scripts =  {'copy_tmp': 'cp /tmp/towr.csv ./data/traj/towr.csv',
            'copy': 'docker cp <id>:root/catkin_ws/src/towr_solo12/towr/build/traj.csv ./data/traj/towr.csv',
            'run': 'docker exec <id> ./main',
            'info': 'docker ps -f ancestor=towr',
            'data': 'docker cp <id>:root/catkin_ws/src/towr_solo12/towr/build/traj.csv /tmp/towr.csv',
            'delete': 'rm ./data/traj/towr.csv',
            'heightfield_rm' : 'docker exec -t <id> rm /root/catkin_ws/src/towr_solo12/towr/data/heightfields/from_pybullet/towr_heightfield.txt',
            'heightfield_copy': 'docker cp ./data/heightfields/from_pybullet/towr_heightfield.txt <id>:root/catkin_ws/src/towr_solo12/towr/data/heightfields/from_pybullet/towr_heightfield.txt',
            'touch_file' : 'touch ./data/traj/towr.csv',
}

_flags = ['-g', '-s', '-s_ang', '-s_vel', '-e1', '-e2', '-e3', '-e4', '-t', '-r', '-resolution', 's_vel', 's_ang_vel', '-duration']

def create_cmd(q_ang=None, q_vel=None):
    """
    Create a command dictionary for controlling a robot's joints.

    This function creates a dictionary that represents a command for controlling a robot's joints. It can be used to specify desired joint angles and velocities.

    Parameters:
        q_ang (list or numpy.ndarray, optional): Desired joint angles for the robot. Should be a list or array of length 12.
        q_vel (list or numpy.ndarray, optional): Desired joint velocities for the robot. Should be a list or array of length 12.

    Returns:
        dict: A dictionary with keys for different robot components and values representing joint angle or velocity commands.
    """
    cmd = {"FL_FOOT": {"P": np.zeros(3), "D": np.zeros(3)}, "FR_FOOT": {"P": np.zeros(3), "D": np.zeros(3)},
            "HL_FOOT": {"P": np.zeros(3), "D": np.zeros(3)}, "HR_FOOT": {"P": np.zeros(3), "D": np.zeros(3)}}
    if q_ang is not None:
        assert(len(q_ang) >= 12)
        for i in range(4):
            if i == 0:
                cmd['FL_FOOT']['P'] = q_ang[0:3]
            elif i == 1:
                cmd['FR_FOOT']['P'] = q_ang[3:6]
            elif i == 2:
                cmd['HL_FOOT']['P'] = q_ang[6:9]
            elif i == 3:
                cmd['HR_FOOT']['P'] = q_ang[9:12]
    if q_vel is not None:
        assert(len(q_vel) == 12)
        for i in range(4):
            if i == 0:
                cmd['FL_FOOT']['D'] = q_ang[0:3]
            elif i == 1:
                cmd['FR_FOOT']['D'] = q_ang[3:6]
            elif i == 2:
                cmd['HL_FOOT']['D'] = q_ang[6:9]
            elif i == 3:
                cmd['HR_FOOT']['D'] = q_ang[9:12]
    return cmd

def create_cmd_pose():
    """
    Create a command pose dictionary for controlling a robot's end effectors and center of mass.

    This function creates a dictionary that represents a command pose for controlling a robot's end effectors (feet), center of mass (COM), COM velocity, and foot forces.

    Returns:
        dict: A dictionary with keys for "COM," "COM_VEL," "FL_FOOT," "FR_FOOT," "HL_FOOT," "HR_FOOT," "FL_FOOT_FORCE," "FR_FOOT_FORCE," "HL_FOOT_FORCE," and "HR_FOOT_FORCE."
    """
    return {"COM": np.zeros(6), "FL_FOOT": {"P": np.zeros(3), "D": np.zeros(3)}, "FR_FOOT": {"P": np.zeros(3), "D": np.zeros(3)},
            "HL_FOOT": {"P": np.zeros(3), "D": np.zeros(3)}, "HR_FOOT": {"P": np.zeros(3), "D": np.zeros(3)}, "COM_VEL" : np.zeros(6),
            "FL_FOOT_FORCE": np.zeros(3), "FR_FOOT_FORCE": np.zeros(3), "HL_FOOT_FORCE": np.zeros(3), "HR_FOOT_FORCE": np.zeros(3)}

def vec_to_cmd(vec, mode= "P", joint_cnt=3):
    """
    Convert a flattened vector to a command dictionary.

    This function takes a flattened vector and converts it into a dictionary that represents a command for controlling a robot's joints or end effectors. The input vector is assumed to contain either joint angle or position information.

    Parameters:
        vec (list or numpy.ndarray): A flattened vector containing joint angle or position information.
        mode (str, optional): The mode for specifying the command, either "P" for position or "D" for joint angles.
        joint_cnt (int): The number of joints or components in the vector for each element of the command.

    Returns:
        dict: A dictionary representing the command with keys for different robot components.

    """
    cmd = create_cmd()
    for i in range(len(vec)//joint_cnt):
        if i == 0:
            cmd["FL_FOOT"][mode] = vec[0:3]
        elif i == 1:
            cmd["FR_FOOT"][mode] = vec[3:6]
        elif i == 2:
            cmd["HL_FOOT"][mode] = vec[6:9]
        elif i == 3:
            cmd["HR_FOOT"][mode] = vec[9:12]
    return cmd

def vec_to_cmd_pose(vec, mode="P", joint_cnt=3, z_shift=0.0):
    """
    Convert a flattened vector to a command pose dictionary.

    This function takes a flattened vector and converts it into a dictionary that represents a command pose for controlling a robot or system. The input vector is assumed to contain position and orientation information for various components of the robot or system.

    Parameters:
        vec (list or numpy.ndarray): A flattened vector containing the pose information.
        mode (str): The mode for specifying pose information, either "P" for position or "D" for orientation.
        joint_cnt (int): The number of joints or components in the vector for each element of the command pose.
        z_shift (float): A value to adjust the Z-coordinate of certain components (e.g., foot positions) in the command pose.

    Returns:
        dict: A dictionary representing the command pose with keys for different components, such as "COM," "FL_FOOT," "FR_FOOT," etc.
    """
    cmd = create_cmd_pose()
    for i in range(len(vec)//3):
        if i == 0:
            cmd["COM"] = vec[0:6]
        elif i == 1:
            cmd["FL_FOOT"][mode] = vec[6:9]
            cmd["FL_FOOT"][mode][2] += z_shift
        elif i == 2:
            cmd["FR_FOOT"][mode] = vec[9:12]
            cmd["FR_FOOT"][mode][2] += z_shift
        elif i == 3:
            cmd["HL_FOOT"][mode] = vec[12:15]
            cmd["HL_FOOT"][mode][2] += z_shift
        elif i == 4:
            cmd["HR_FOOT"][mode] = vec[15:18]
            cmd["HR_FOOT"][mode][2] += z_shift
        elif i == 5:
            cmd["COM_VEL"] = vec[18:24]
        elif i == 7:
            cmd["FL_FOOT_FORCE"] = vec[24:27]
        elif i == 8:
            cmd["FR_FOOT_FORCE"] = vec[27:30]
        elif i == 9:
            cmd["HL_FOOT_FORCE"] = vec[30:33]
        elif i == 10:
            cmd["HR_FOOT_FORCE"] = vec[33:36]
    return cmd

def combine(*vectors):
    """
    Combine multiple 3D vectors into a single 12-element vector.

    This function takes multiple 3D vectors as input and combines them into a single 12-element vector.

    Parameters:
        *vectors: Variable number of 3D vectors to combine.

    Returns:
        numpy.ndarray: A 12-element vector containing the combined values.
    """
    v = np.zeros(12)
    cnt = len(vectors)
    i = 1
    while (i < cnt + 1):
        if vectors[i - 1] == None:
            v[(i-1)*3:i*3] = np.zeros(3)
        else:
            v[(i-1)*3:i*3] = vectors[i - 1][(i-1)*3:i*3]
        i += 1
    return v

def transformation_mtx(t, R):
    """
    Create a 4x4 transformation matrix given translation and rotation information.

    This function creates a 4x4 transformation matrix that represents a transformation from one frame to another frame in 3D space. It takes translation and rotation information as input and constructs the transformation matrix accordingly.

    Parameters:
        t (list or numpy.ndarray): A list or array containing the translation [x, y, z].
        R (list, numpy.ndarray, or quaternion): Rotation information, which can be provided in different formats.
            - If R is a list or numpy array of size (3, 3), it represents a 3x3 rotation matrix.
            - If R is a list or numpy array of size (4,), it represents a quaternion [x, y, z, w].
            - If R is a list of size (3,), it represents Euler angles [roll, pitch, yaw].

    Returns:
        numpy.ndarray: A 4x4 transformation matrix that combines the translation and rotation.
    """
    mtx = np.eye(4)
    if type(R) == list or type(R) == np.ndarray:
        r = Rotation.from_euler('xyz', [R[0], R[1], R[2]])
        R = r.as_matrix()
        mtx[0:3,0:3] = R
    elif len(R) == 3:
        mtx[:3][:3] = R
    elif len(R) == 4:
        mtx[0][:3]= p.getMatrixFromQuaternion(R)[:3]
        mtx[1][:3]= p.getMatrixFromQuaternion(R)[3:6]
        mtx[2][:3] = p.getMatrixFromQuaternion(R)[6:9]
    mtx[:,3][:3] = t
    return mtx

def transformation_inv(M):
    """
    Create a 4x4 transformation matrix from translation and rotation components.

    This function creates a 4x4 transformation matrix from translation and rotation components.

    Parameters:
        t (numpy.ndarray or list): Translation vector [x, y, z].
        R (numpy.ndarray or list): Rotation matrix (3x3) or quaternion [x, y, z, w].

    Returns:
        numpy.ndarray: A 4x4 transformation matrix.
    """
    mtx = np.eye(4)
    R_inv = np.linalg.inv(M[:3, :3])
    R_inv_t = -np.linalg.inv(M[:3, :3])@M[:,3][:3]
    mtx[:3, :3] = R_inv
    mtx[:,3][:3]= R_inv_t
    return mtx

def transformation_multi(M, v):
    """
    Compute the inverse transformation matrix.

    This function computes the inverse transformation matrix of a given transformation matrix.

    Parameters:
        M (numpy.ndarray): A 4x4 transformation matrix.

    Returns:
        numpy.ndarray: The inverse of the input transformation matrix.
    """
    if type(v) is list:
        v = np.array(v + [1])
    result = (M @ v)[0:3]
    return result

def euler_to_quaternion(yaw, pitch, roll):
    """
    Convert Euler angles to a quaternion.

    This function converts Euler angles (yaw, pitch, roll) to a quaternion representation.

    Parameters:
        yaw (float): Yaw angle in radians.
        pitch (float): Pitch angle in radians.
        roll (float): Roll angle in radians.

    Returns:
        list: A list representing the quaternion in the [x, y, z, w] format.
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]
    
def trajectory_2_world_frame(robot, traj, bezier=False, towr=False, original_traj=None):
    """
    Transform a trajectory from the base frame to the global frame of the PyBullet environment.

    This function transforms a trajectory represented in the robot's base frame to the global frame of the PyBullet environment.

    Parameters:
        robot: The robot object.
        traj (dict): A dictionary containing trajectory data for each foot in the robot's base frame.
        bezier (bool): A boolean indicating whether the trajectory uses Bezier control points.
        towr (bool): A boolean indicating whether the trajectory uses TOWR control points.
        original_traj: An optional original trajectory used for reference.

    Returns:
        dict: A dictionary containing the transformed trajectory data in the global frame.
    """
    config = robot.CoM_states() #Query the state of robot in global frame
    before_tf_towr = copy.deepcopy(traj)
    for link in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'):
        for mode in ("P", "D"):
            if mode == "P":
                # tf_mtx = transformation_mtx(config['linkWorldPosition'], np.zeros(3))
                tf_mtx = transformation_mtx(config['linkWorldPosition'], config['linkWorldOrientation'])
            if mode == "D" and bezier:
                tf_mtx = transformation_mtx(np.zeros(3), config['linkWorldOrientation'])
            
            vec = np.concatenate((np.array([traj[link][mode][0]]), 
                                    np.array([traj[link][mode][1]]),  
                                    np.array([traj[link][mode][2]]), 
                                    np.ones(1)))

            if bezier:
                vec = np.concatenate((np.array([traj[link][mode][0] + robot.shift[link][0]]), 
                                    np.array([traj[link][mode][1] + robot.shift[link][1]]),  
                                    np.array([traj[link][mode][2] + robot.shift[link][2]]), 
                                    np.ones(1)))
            elif towr:
                vec = np.concatenate((np.array([traj[link][mode][0]]), 
                                    np.array([traj[link][mode][1]]), 
                                    np.array([traj[link][mode][2]]), 
                                    np.ones(1)))
            tf_vec = tf_mtx @ vec            
            traj[link][mode] = tf_vec[:3]
    return traj

def trajectory_2_local_frame(robot, traj, bezier=False, towr=False, original_traj=None):
    """
    Transform a trajectory from the global frame to the robot's local frame.

    This function transforms a trajectory represented in the global frame of the PyBullet environment to the robot's local frame.

    Parameters:
        robot: The robot object.
        traj (dict): A dictionary containing trajectory data in the global frame.
        bezier (bool): A boolean indicating whether the trajectory uses Bezier control points.
        towr (bool): A boolean indicating whether the trajectory uses TOWR control points.
        original_traj: An optional original trajectory used for reference.

    Returns:
        dict: A dictionary containing the transformed trajectory data in the robot's local frame.
    """
    config = robot.CoM_states() #Query the state of robot in global frame
    before_tf_towr = copy.deepcopy(traj)
    for link in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'):
        for mode in ("P", "D"):
            if mode == "P":
                tf_mtx = transformation_inv(transformation_mtx(config['linkWorldPosition'], config['linkWorldOrientation']))
            if mode == "D" and bezier:
                tf_mtx = transformation_inv(transformation_mtx(np.zeros(3), config['linkWorldOrientation']))
            
            vec = np.concatenate((np.array([traj[link][mode][0]]), 
                                    np.array([traj[link][mode][1]]),  
                                    np.array([traj[link][mode][2]]), 
                                    np.ones(1)))
            tf_vec = tf_mtx @ vec            
            traj[link][mode] = tf_vec[:3]
    return traj
    
def tf_2_world_frame(traj, CoM):
    """
    Transform a trajectory from the robot's local frame to the global frame.

    This function transforms a trajectory represented in the robot's local frame to the global frame of the PyBullet environment.

    Parameters:
        traj (list or numpy.ndarray): A trajectory vector in the robot's local frame.
        CoM (dict): The state of the robot's center of mass (CoM) in the global frame.

    Returns:
        list or numpy.ndarray: The transformed trajectory vector in the global frame.
    """
    traj[0] = traj[0] - CoM['linkWorldPosition'][0]
    traj[1] = traj[1] - CoM['linkWorldPosition'][1]
    traj[2] = 0
    return traj

def sampleTraj(robot, r=0.1, N=100):
    """
    Sample a trajectory for each foot end-effector of a robot.

    This function generates a trajectory for each foot end-effector of a robot by sampling points on a circular path.
    The trajectory is specified as a dictionary containing positions for each foot at each sampled point.

    Parameters:
        robot: An object representing the robot.
        r (float): The radius of the circular path (default is 0.1 units).
        N (int): The number of points to sample on the circular path (default is 100).

    Returns:
        dict: A dictionary containing trajectories for each foot end-effector.
            The keys are 'FL_FOOT', 'FR_FOOT', 'HL_FOOT', and 'HR_FOOT', and the values are lists of 3D positions.

    """
    traj_dic = {}
    traj = list()
    theta = np.linspace(0, 2*np.pi, N)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = 0.2 * np.sin(theta)
    config = robot.get_endeffector_pose()
    for link in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'):
        tf_mtx = transformation_mtx(config[link]['linkWorldPosition'], config[link]['linkWorldOrientation'])
        for val_x, val_y, val_z in zip(x, y, z):
            vec = np.concatenate((np.array([val_x]), np.array([val_y]), np.array([val_z]), np.ones(1)))
            tf_vec = tf_mtx @ vec
            traj.append(tf_vec[:3])
        traj_dic[link] = traj
        traj = list()
    return traj_dic

def convert12arr_2_16arr(arr):
    """
    Convert a 12-element array into a 16-element array with zero padding.

    This function takes a 12-element array and expands it into a 16-element array by adding zero padding.
    The resulting 16-element array has a specific pattern where every fourth element is zero.

    Parameters:
        arr (numpy.ndarray): A 12-element NumPy array to be converted.

    Returns:
        numpy.ndarray: A 16-element NumPy array with zero-padding.
    """
    arr16 = np.zeros(16,)
    idx = 0
    for i in range(4):
        for j in range(3):
            arr16[idx] = arr[i*3 + j]
            idx += 1
        idx += 1
    return arr16

def towr_transform(robot, traj, towr=True, ee_shift=0.015):
    """Helper function to transform 'raw towr data' from world frame to 
       base frame on robot. Then the base frame is converted back to world 
       frame in Pybullet enviroment to help fix misalignment if towr trajectory runs
       faster than what the simulator can track.
       
    Args:
        robot (_type_): _description_
        traj (_type_): _description_

    Returns:
        _type_: _description_
    """
    sim_z_height = robot.CoM_states()['linkWorldPosition'][2]
    CoM = traj['COM'][0:3]
    original_traj = copy.deepcopy(traj)
    tfMtx =  transformation_inv(transformation_mtx(CoM, traj['COM'][3:6]))
    for t in traj:
        if t in EE_NAMES:
            vec = np.concatenate((traj[t]['P'], np.ones(1)))
            t_vec = tfMtx @ vec
            traj[t]['P'] = t_vec[0:3]
            traj[t]['P'][2] += ee_shift
    traj = trajectory_2_world_frame(robot, traj, towr=towr, original_traj=original_traj)
    return traj

def norm(v1, v2):
    """return L2 norm of two vectors

    Args:
        v1 (_type_): 1st n-d vector 
        v2 (_type_): 2nd n-d vector

    Returns:
        _type_: L2 norm
    """
    l2 = 0.0
    for i, j in zip(v1, v2):
        l2 += (i - j)**2
    return math.sqrt(l2)


def nearestPoint(point, file):
    """Finds the nearest near neighoring csv row corresponding to the
       desired input point (robot COM)

    Args:
        point (_type_): robot CoM
        file (_type_): trajectory file

    Returns:
        _type_: truncated csv file starting at input point
    """
    csv_file = csv.reader(file, delimiter=',')
    l2 = math.inf
    start_idx = 0
    for i, entry in enumerate(csv_file):
        compare_pt = tuple(float(s) for s in entry[0:3])
        if l2 > norm(compare_pt, point):
            start_idx = i
            l2 = norm(compare_pt, point)
    file.seek(0)
    csv_file = csv.reader(file, delimiter=',')
    for _ in range(0, start_idx):
        next(csv_file)
    return csv_file

def percentage_look_ahead(file, percent=0.5):
    """Looks aheads of desired percentage into the trajectory

    Args:
        file (_type_): trajectory csv file
        percent (float, optional): start percentage in trajectory. Defaults to 0.5.
    """
    p = 0
    reader = csv.reader(file)
    no_lines= len(list(reader))
    file.seek(0)
    while (p < percent):
        p += float(1/(no_lines))
        next(reader)
    return reader

def look_ahead(file, start_time=0.0, timesteps=6000, decimal_roundoff=3):
    """
    Look ahead in a CSV file to find a specific starting time and retrieve subsequent data.

    This function reads a CSV file and looks for a specific starting time (or closest available time) to begin reading data.
    It then retrieves a specified number of timesteps of data from that starting point.

    Parameters:
        file (file-like object): The CSV file-like object to read from.
        start_time (float, optional): The desired starting time. Default is 0.0.
        timesteps (int, optional): The number of timesteps (rows) of data to retrieve after the starting time. Default is 6000.
        decimal_roundoff (int, optional): The number of decimal places to round the timestamp for comparison. Default is 3.

    Returns:
        csv.reader: A CSV reader object positioned at the requested starting time.
        int: The index of the row where the reader is positioned.
    """
    reader = csv.reader(file)
    stop_idx = 0
    while (True):
        t = next(reader)[0]
        stop_idx += 1
        if start_time <= round(float(t), ndigits=decimal_roundoff):
            break
    for i in range(timesteps - 1):
        next(reader)
    return reader, stop_idx

def zero_filter(x, tol=1e-4):
    """
    Filter out values in an iterable that are close to zero.

    This function takes an iterable `x` and a tolerance `tol` and filters out values that are close to zero within the specified tolerance.
    Values with absolute magnitude less than `tol` are replaced with zero.

    Parameters:
        x (iterable): The input iterable containing numeric values to be filtered.
        tol (float, optional): The tolerance level to determine if a value is close to zero. Default is 1e-4.

    Returns:
        iterable: An iterable with values filtered by replacing near-zero values with zero.

    """
    for i, val in enumerate(x):
        if abs(val) < tol:
            x[i] = 0
    return x

def is_numeric(s):
    """
    Check if a string can be converted to a numeric value.

    This function checks whether a given string can be successfully converted to a numeric (float) value.

    Parameters:
        s (str): The input string to be checked.

    Returns:
        bool: True if the string can be converted to a numeric value, False otherwise.
    """

    try:
        float(s)
        return True
    except ValueError:
        return False

def txt_2_np_reader(file, delimiter=','):
    """
    Read data from a text file and convert it to a NumPy array.

    This function reads data from a text file, where each line represents a row of data with values separated by a delimiter.
    It then converts the data into a NumPy array.

    Parameters:
        file (str): The path to the text file containing the data.
        delimiter (str, optional): The delimiter used to separate values in each line. Default is ',' (comma).

    Returns:
        numpy.ndarray: A NumPy array containing the data read from the text file.
    """
    data = []
    with open(file, 'r') as f:
        reader = f.readlines()
        for row in reader:
            _row = row.strip().split(delimiter)
            _row = [float(x) for x in _row if is_numeric(x)]
            data.append(_row)
    return np.array(data)

def save_height_grid_map(height_map, save_file="./data/plots/height_map.png"):
    """
    Save a height grid map as an image file.

    This function takes a height grid map represented as a 2D numpy array and saves it as an image file in PNG format.

    Parameters:
        height_map (numpy.ndarray): A 2D numpy array representing a height grid map.
        save_file (str): The file path where the image will be saved. Default is "./data/plots/height_map.png".

    Notes:
        This function creates an image of the height grid map with a specified width and height. It reverses the map,
        sets axis labels, hides axes, and saves the image with high DPI and tight bounding box settings.

    Args:
        height_map (numpy.ndarray): A 2D numpy array representing a height grid map.
        save_file (str, optional): The file path where the image will be saved. Default is "./data/plots/height_map.png".
    """
    dpi = 600
    width, height = 10, 6
    fig, ax = plt.subplots(figsize=(width, height))
    height_map = height_map[::-1, :]
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('off')
    ax.imshow(height_map, cmap='gray_r', origin='upper', extent=(0, len(height_map[0]), 0, len(height_map)))
    plt.savefig(save_file, dpi=dpi, bbox_inches='tight', pad_inches=0.1)
    plt.close()

def save_bool_map(bool_map, save_file="./data/plots/bool_map.png"):
    """
    Save a boolean map as an image file.

    This function takes a boolean map represented as a 2D numpy array and saves it as an image file in PNG format.

    Parameters:
        bool_map (numpy.ndarray): A 2D numpy array representing a boolean map.
        save_file (str): The file path where the image will be saved. Default is "./data/plots/bool_map.png".

    Notes:
        This function creates an image of the boolean map with a specified width and height. It reverses the map,
        sets axis labels, hides axes, and saves the image with high DPI and tight bounding box settings.

    Args:
        bool_map (numpy.ndarray): A 2D numpy array representing a boolean map.
        save_file (str, optional): The file path where the image will be saved. Default is "./data/plots/bool_map.png".
    """
    dpi = 600
    width, height = 10, 6
    fig, ax = plt.subplots(figsize=(width, height))
    bool_map = bool_map[::-1, :]
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('off')
    ax.imshow(bool_map, cmap='binary', origin='upper', extent=(0, len(bool_map[0]), 0, len(bool_map)))
    plt.savefig(save_file, dpi=dpi, bbox_inches='tight', pad_inches=0.1)
    plt.close()


def cmd_args(args):
    """Commandline parser to run python subprocesses correctly

    Args:
        args (dict): user + config inputs for simulation and solver

    Return:
        _cmd : parsed command line string that can be ran as a excutable
    """

    def _bracket_rm(s):
        return s.replace("[", "").replace("]", "")

    def _remove_comma(s):
        return s.replace(",", "")
    
    def _filter(s):
        return _bracket_rm(_remove_comma(str(s)))

    _cmd = ""

    for key, value in args.items():
        if key in _flags and value:
            _cmd += key + " " + _filter(value)
            _cmd += " "

    return _cmd

def parse_scripts(scripts_dic, docker_id):
    """Helper function that parsers through the user command arguments

    Args:
        scripts_dic (dict): The available Bash script commands for the user
        docker_id (dict): The ID of the Docker container running the local planner

    Returns:
        scripts_dic: Formated Bash scripts commands for Python process execution
    """
    for script_name, script in scripts_dic.items():
        scripts_dic[script_name] = script.replace("<id>", docker_id)
    return scripts_dic

def DockerInfo():
    """Helper function that finds the ID of the Docker runnning on the host system
    """
    p = subprocess.run([scripts['info']], shell=True, capture_output=True, text=True)
    output = p.stdout.replace('\n', ' ')
    dockerid, idx = output.split(), output.split().index('towr') - 1
    return dockerid[idx]


def experimentInfo(experiement_name, record_traj=False, test=False):
    """Helper function to setup different terrain experiments to test the Q-TOS Stack

    Args:
        experiement_name (string): name of experiment the user is trying to test

    Returns:
        sim_cfg: The corresponding configuration file to setup + run the the experiment
    """
    sim_cfg = None
    if record_traj:
        file_path = os.path.join("./data/config", "record.yml")
        sim_cfg = yaml.safe_load(open(file_path, 'r'))
    else:
        experiment_names = {"default": "simulation.yml", "exp_1": "experiment_1_straight_line.yml",
                            "exp_2": "experiment_2_climbing.yml", "exp_3": "experiment_3_collision_avoidance.yml",
                            "exp_4" : "experiment_4_rough_terrain.yml", "exp_5": "experiment_5_extreme_climbing.yml",
                            "exp_6" : "experiment_6_stairs.yml", "FSS_Plot" : "create_FSS_plots.yml",
                            "exp_7" : "experiment_7_climb_obstacle.yml", "exp_8" : "experiment_8_dynamic_terrain.yml",
                            "exp_9" : "experiment_9_continous_walking.yml", "exp_10" : "experiment_10_continous_climbing.yml",
                            "test" : "simulation_QTOS_test.yml"}
        if test:
            file_path = os.path.join("./test/data/config", experiment_names[experiement_name])
        else:
            file_path = os.path.join("./data/config", experiment_names[experiement_name])
        sim_cfg = yaml.safe_load(open(file_path, 'r'))
    return sim_cfg



def links_to_id(robot):
    """
    Helper function to retrieve the joint info of a robot into a dictionary.

    Args:
        robot (pybullet obj): pybullet robot object

    Returns:
        dict: Dictionary filled with corresponding link name and id.
    """
    _link_name_to_index = {p.getBodyInfo(robot)[0].decode('UTF-8'):-1,}
    for _id in range(p.getNumJoints(robot)):
        _name = p.getJointInfo(robot, _id)[12].decode('UTF-8')
        _link_name_to_index[_name] = _id
    return _link_name_to_index

def link_info(link):
    """
    Retrieve information about a robot link.

    Args:
        link (tuple): A tuple containing information about a robot link.

    Returns:
        dict: A dictionary containing various properties of the link, such as position and orientation in the world frame,
        and local inertial frame information.
    """
    return {"linkWorldPosition": link[0], "linkWorldOrientation": link[1], "localInertialFramePosition": link[2], "localInertialFrameOrientation": link[3], 
     "worldLinkFramePosition": link[4], "worldLinkFrameOrientation": link[5]}

def q_init_16_arr(q_init):
    """
    Place q_init values into a 16-index array to account for fixed joints in URDF.

    Args:
        q_init (list or np.array): Initial joint values.

    Returns:
        np.array: An array with 16 elements, with the values from q_init at specific indexes.
    """
    indexes = [0,1,2,4,5,6,8,9,10,12,13,14]
    q_init_new = np.zeros(16)
    for i, q_val in zip(indexes, q_init):
        q_init_new[i] = q_val
    return q_init_new

def base_frame_tf(mtx, pt):
    """
    Helper function to transform a vector from one frame to another using a transformation matrix.

    Args:
        mtx (np.array): Transformation matrix.
        pt (np.array): Position vector.

    Returns:
        np.array: Transformed position vector.
    """
    vec = np.concatenate((np.array([pt[0]]), np.array([pt[1]]),np.array([pt[2]]), np.ones(1)))
    tf_vec = mtx @ vec
    return tf_vec[:3]

def shift_z(v, shift):
    """
    Helper function to shift a 3D vector and adjust the z-axis variable.

    Args:
        v (np.array or list): 3D vector to shift.
        shift (float or int): Scalar value to shift the z-coordinate.

    Returns:
        np.array or list: The resulting 3D vector after the shift.
    """
    v[2] += shift
    return v