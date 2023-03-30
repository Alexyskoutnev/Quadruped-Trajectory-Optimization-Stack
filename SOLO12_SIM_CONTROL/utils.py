import pybullet as p
import numpy as np



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
    if len(R) == 3:
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

def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
    
def trajectory_2_world_frame(robot, traj):
    traj_dict = {}
    # config = robot.get_endeffector_pose()
    config = robot.CoM_states()
    for link in ('FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'):
        # print("link -> ", link)
        # breakpoint()
        tf_mtx = transformation_mtx(config['linkWorldPosition'], config['linkWorldOrientation'])
        # breakpoint()
        vec = np.concatenate((np.array([traj[link][0] + robot.shift[link][0]]), np.array([traj[link][1] + robot.shift[link][1]]), np.array([traj[link][2] + robot.shift[link][2]]), np.ones(1)))
        tf_vec = tf_mtx @ vec
        traj_dict[link] = tf_vec[:3]
    return traj_dict
    
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
