from QTOS.robot.robot import SOLO12
from QTOS.simulation import Simulation

import yaml
import pinocchio as pin
import numpy as np
import pybullet as p

URDF = "./data/urdf/solo12.urdf"
config = "./data/config/solo12.yml"
config_sim = "./data/config/simulation.yml"
_cfg = yaml.safe_load(open(config, 'r'))
_sim_cfg = yaml.safe_load(open(config_sim, 'r'))

class Loader:
    def __init__(self, urdf_path, config, fixed = 0) -> None:
        if config['use_pinocchio']:
            _load = pin.buildModelFromUrdf(urdf_path)
            self.model = _load
            self.data = _load.createData()
        self.robot = p.loadURDF(urdf_path, config['start_pos'], config['start_ang'], useFixedBase=fixed)
        self.q0 = np.array(config['q_init'])
    
    def __repr__(self) -> str:
        return str(self.robot)

def builder(cfg=_cfg, sim_cfg=None):
    SIMULATION = Simulation(sim_cfg)
    loader = Loader(URDF, cfg)
    ROBOT = SOLO12(URDF, cfg, sim_cfg=sim_cfg, loader=loader)
    return {"robot": ROBOT, "sim": SIMULATION, "args": sim_cfg}