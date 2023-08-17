import pybullet
import time
import pybullet_data
import yaml
import csv
import numpy
from numpy import genfromtxt

from SOLO12_SIM_CONTROL.robot.robot import SOLO12
from SOLO12_SIM_CONTROL.utils import *
from SOLO12_SIM_CONTROL.visual import Visual_Planner

solo12_urdf_fname = "./data/urdf/solo12.urdf"
config_fname = "./data/config/solo12.yml"
config_sim_fname = "./data/config/simulation.yml"
terrain_fname = "./data/heightmaps/heightfield.txt"
traj_fname = "./data/traj/towr.csv"
URDF = "./data/urdf/solo12.urdf"
#traj_fname = "data/traj/lift_one_foot.csv"
# traj_fname = "data/traj/lift_two_feet.csv"
# traj_fname=  "data/traj/test.csv"

cfg = yaml.safe_load(open(config_fname, 'r'))
sim_cfg = yaml.safe_load(open(config_sim_fname, 'r'))
#heightfield_pos = [1.5,0,0]

physicsClient = pybullet.connect(p.GUI)
pybullet.setGravity(0,0,-10)

# """ terrain """
# terrainShape = pybullet.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,1], fileName = terrain_fname, heightfieldTextureScaling=64)
# terrain = pybullet.createMultiBody(0, terrainShape)
# pybullet.resetBasePositionAndOrientation(terrain, sim_cfg["terrain_offset"], [0,0,0,1])
# pybullet.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
# pybullet.changeVisualShape(terrain, -1, rgbaColor=[1,1,1,1])

""" example objects """
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = pybullet.loadURDF("plane.urdf")
#solo12Id = pybullet.loadURDF(solo12_urdf_fname, cfg["start_pos"], cfg["start_ang"], useFixedBase=0)
""" using SOLO12 class """
robot = SOLO12(URDF, cfg, fixed=sim_cfg['fix-base'], sim_cfg=sim_cfg)
reader = csv.reader(open(traj_fname, 'r', newline=''))

step_size = 500
look_ahead = 5000
v_planner = Visual_Planner(traj_fname, step_size, look_ahead)
v_planner.plot_CoM_plan_init(0)
"""main loop """
t_idx = 0


last_loop_time = time.time()

traj = genfromtxt(traj_fname, delimiter=',')

while (t_idx < sim_cfg["SIM_STEPS"]):
   loop_time = time.time() - last_loop_time

   if loop_time > sim_cfg['TIMESTEPS']:
      last_loop_time = time.time()

   """ read from csv, transform to joint space and track """
   time_step, EE_POSE = traj[t_idx, 0], traj[t_idx, 1:]
   towr_traj = towr_transform(robot, vec_to_cmd_pose(EE_POSE))
   joint_ang, joint_vel, joint_toq = robot.control_multi(towr_traj, robot.EE_index['all'], mode=robot.mode, usePin=False)
   robot.set_joint_control_multi(robot.jointidx['idx'], robot.mode, joint_ang, joint_vel, joint_toq)

   """ read from csv, transform to joint space and set position  """ 
   #COM = EE_POSE[0:6]
   #pybullet.resetBasePositionAndOrientation(robot.robot, COM[0:3], p.getQuaternionFromEuler(COM[3:6]))
   # v_planner.plot_CoM_plan(robot.time_step)
   pybullet.stepSimulation()
   t_idx += 1
   robot.time_step += 1
   v_planner.CoM_step(robot.time)
 
pybullet.disconnect()