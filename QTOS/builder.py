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
        """Initialize a Loader object for loading a robot model.

        Args:
            urdf_path (str): Path to the URDF file of the robot.
            config (dict): Configuration parameters for the robot.
            fixed (int, optional): Whether to use a fixed base for the robot. Defaults to 0 (not fixed).
        """
        if config['use_pinocchio']:
            _load = pin.buildModelFromUrdf(urdf_path)
            self.model = _load
            self.data = _load.createData()
        self.robot = p.loadURDF(urdf_path, config['start_pos'], config['start_ang'], useFixedBase=fixed)
        self.q0 = np.array(config['q_init'])
    
    def __repr__(self) -> str:
        """Return a string representation of the loaded robot model.

        Returns:
            str: A string representation of the robot.
        """
        return str(self.robot)

def builder(cfg=_cfg, sim_cfg=None):
    """Build a simulation environment with a robot.

    Args:
        cfg (dict): Configuration parameters for the robot.
        sim_cfg (dict, optional): Configuration parameters for the simulation. Defaults to None.

    Returns:
        dict: A dictionary containing the robot, simulation, and arguments.
    """
    SIMULATION = Simulation(sim_cfg)
    loader = Loader(URDF, cfg)
    ROBOT = SOLO12(URDF, cfg, sim_cfg=sim_cfg, loader=loader)
    return {"robot": ROBOT, "sim": SIMULATION, "args": sim_cfg}