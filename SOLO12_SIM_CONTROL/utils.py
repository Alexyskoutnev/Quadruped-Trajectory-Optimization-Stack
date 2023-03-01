import pybullet as p
import numpy as np

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
    
