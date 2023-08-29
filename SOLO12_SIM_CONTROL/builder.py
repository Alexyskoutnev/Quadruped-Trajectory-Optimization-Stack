from SOLO12_SIM_CONTROL.robot.robot import SOLO12
from SOLO12_SIM_CONTROL.simulation import Simulation

import yaml

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
config_sim = "./data/config/simulation.yml"
cfg = yaml.safe_load(open(config, 'r'))
sim_cfg = yaml.safe_load(open(config_sim, 'r'))

def builder(cfg=cfg, sim_cfg=sim_cfg):
    SIMULATION = Simulation(sim_cfg)
    ROBOT = SOLO12(URDF, cfg, sim_cfg=sim_cfg)
    return {"robot": ROBOT, "sim": SIMULATION, "args": sim_cfg}