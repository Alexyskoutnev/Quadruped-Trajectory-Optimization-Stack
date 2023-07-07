import pybullet as p
import numpy as np
import math
import csv

from scipy.spatial.transform import Rotation

def create_cmd(q_ang=None, q_vel=None):
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
    return {"COM": np.zeros(6), "FL_FOOT": {"P": np.zeros(3), "D": np.zeros(3)}, "FR_FOOT": {"P": np.zeros(3), "D": np.zeros(3)},
            "HL_FOOT": {"P": np.zeros(3), "D": np.zeros(3)}, "HR_FOOT": {"P": np.zeros(3), "D": np.zeros(3)}}

def vec_to_cmd(vec, mode= "P", joint_cnt=3):
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

def vec_to_cmd_pose(vec, mode="P", joint_cnt=3):
    cmd = create_cmd_pose()
    for i in range(len(vec)//3):
        if i == 0:
            cmd["COM"] = vec[0:6]
        elif i == 1:
            cmd["FL_FOOT"][mode] = vec[6:9]
        elif i == 2:
            cmd["FR_FOOT"][mode] = vec[9:12]
        elif i == 3:
            cmd["HL_FOOT"][mode] = vec[12:15]
        elif i == 4:
            cmd["HR_FOOT"][mode] = vec[15:18]
    return cmd

def combine(*vectors):
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
    mtx = np.eye(4)
    if type(R) == list:
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
    mtx = np.eye(4)
    R_inv = np.linalg.inv(M[:3, :3])
    R_inv_t = -np.linalg.inv(M[:3, :3])@M[:,3][:3]
    mtx[:3, :3] = R_inv
    mtx[:,3][:3]= R_inv_t
    return mtx

def transformation_multi(M, v):
    if type(v) is list:
        v = np.array(v + [1])
    result = (M @ v)[0:3]
    return result

def euler_to_quaternion(yaw, pitch, roll):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
    
def trajectory_2_world_frame(robot, traj):
    config = robot.CoM_states()
    for link in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'):
        for mode in ("P", "D"):
            if mode == "P":
                tf_mtx = transformation_mtx(config['linkWorldPosition'], config['linkWorldOrientation'])
            if mode == "D":
                tf_mtx = transformation_mtx(np.zeros(3), config['linkWorldOrientation'])
            vec = np.concatenate((np.array([traj[link][mode][0] + robot.shift[link][0]]), np.array([traj[link][mode][1] + robot.shift[link][1]]), np.array([traj[link][mode][2] + robot.shift[link][2]]), np.ones(1)))
            tf_vec = tf_mtx @ vec
            traj[link][mode] = tf_vec[:3]
    return traj
    
def tf_2_world_frame(traj, CoM):
    traj[0] = traj[0] - CoM['linkWorldPosition'][0]
    traj[1] = traj[1] - CoM['linkWorldPosition'][1]
    # traj[2] = traj[2] - CoM[2]['linkWorldPosition']
    traj[2] = 0
    return traj

def sampleTraj(robot, r=0.1, N=100):
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
    arr16 = np.zeros(16,)
    idx = 0
    for i in range(4):
        for j in range(3):
            arr16[idx] = arr[i*3 + j]
            idx += 1
        idx += 1
    return arr16

def towr_transform(robot, traj):
    for t in traj:
        if t == 'FL_FOOT':
            if traj['FL_FOOT']['P'][2] < 0 or math.isclose(traj['FL_FOOT']['P'][2], 0, rel_tol=1e-3):
                traj['FL_FOOT']['P'][2] = 0
            traj['FL_FOOT']['P'][0] =  traj['FL_FOOT']['P'][0] - traj['COM'][0:3][0]
            traj['FL_FOOT']['P'][1] =  traj['FL_FOOT']['P'][1] - traj['COM'][0:3][1]
            traj['FL_FOOT']['P'][2] = abs(traj['FR_FOOT']['P'][2])
        elif t == "FR_FOOT":
            if traj['FR_FOOT']['P'][2] < 0 or math.isclose(traj['FR_FOOT']['P'][2], 0, rel_tol=1e-3):
                traj['FR_FOOT']['P'][2] = 0
            traj['FR_FOOT']['P'][0] =  traj['FR_FOOT']['P'][0] - traj['COM'][0:3][0]
            traj['FR_FOOT']['P'][1] =  traj['FR_FOOT']['P'][1] - traj['COM'][0:3][1]
            traj['FR_FOOT']['P'][2] = abs(traj['FR_FOOT']['P'][2])
        elif t == "HL_FOOT":
            if traj['HL_FOOT']['P'][2] < 0 or math.isclose(traj['HL_FOOT']['P'][2], 0, rel_tol = 1e-3):
                traj['HL_FOOT']['P'][2] = 0
            traj['HL_FOOT']['P'][0] =  traj['HL_FOOT']['P'][0] - traj['COM'][0:3][0]
            traj['HL_FOOT']['P'][1] = traj['HL_FOOT']['P'][1] - traj['COM'][0:3][1]
            traj['HL_FOOT']['P'][2] = abs(traj['HL_FOOT']['P'][2])
        elif t == "HR_FOOT":
            if traj['HR_FOOT']['P'][2] < 0 or math.isclose(traj['HR_FOOT']['P'][2], 0, rel_tol = 1e-3):
                 traj['HR_FOOT']['P'][2] = 0
            traj['HR_FOOT']['P'][0] =  traj['HR_FOOT']['P'][0] - traj['COM'][0:3][0]
            traj['HR_FOOT']['P'][1] =  traj['HR_FOOT']['P'][1] - traj['COM'][0:3][1]
            traj['HR_FOOT']['P'][2] = abs(traj['HR_FOOT']['P'][2])
    del traj['COM']
    return trajectory_2_world_frame(robot, traj)

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

def look_ahead(file, start_time=0.0, timesteps=600, decimal_roundoff=2):
    reader = csv.reader(file)
    stop_idx = 0
    while (True):
        t = next(reader)[0]
        stop_idx += 1
        if start_time <= round(float(t), ndigits=decimal_roundoff): #Think of a more robust way to check start times??
            break
    for i in range(timesteps - 1):
        next(reader)
    return reader, stop_idx

def zero_filter(x, tol=1e-4):
    for i, val in enumerate(x):
        if abs(val) < tol:
            x[i] = 0
    return x