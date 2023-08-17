from collections import deque

import SOLO12_SIM_CONTROL.config.global_cfg

import numpy as np
import pybullet as p
import pybullet_data

TRAJ = "../data/traj/towr.csv"

class FIFOQueue:
    def __init__(self):
        self.queue =deque()

    def enqueue(self, item):
        self.queue.append(item)

    def dequeue(self):
        if not self.is_empty():
            return self.queue.popleft()
        else:
            raise IndexError("Visual Queue ID is empty")

    def is_empty(self):
        return len(self.queue) == 0

    def size(self):
        return len(self.queue)

class Visual_Planner:
    def __init__(self, traj_file, step_size=500, lookahead=5000) -> None:
        self.CoM_id = FIFOQueue()
        self.CoM_radius = 0.025
        self.CoM_orientation = [0, 0, 0, 1]
        self.foot_id = FIFOQueue()
        self.traj_file = traj_file
        self.step_size = step_size
        self.look_ahead = lookahead

    def load_plan(self, timestep, lookahead=5000):
        data = np.loadtxt(self.traj_file, delimiter=',')
        new_traj = data[data[:,0] >= timestep][:,1:][0:lookahead]
        return new_traj

    def plot_CoM_plan_init(self, timestep):
        plan = self.load_plan(timestep)
        radius = self.CoM_id
        orientation = self.CoM_orientation
        num_com_points = self.lookahead // self.step_size
        for i in range(1, num_com_points):
            visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                        radius=self.CoM_radius,
                                        rgbaColor=[0.1, 1, 0, 1])
            CoM = list(plan[i*self.step_size][0:3])
            print(f"[{i}] COM [{CoM}]")
            visual_body_id = p.createMultiBody(baseVisualShapeIndex=visual_shape_id,
                                            basePosition=CoM,
                                            baseOrientation=orientation)
            self.CoM_id.enqueue(visual_body_id)

    def plot_Com_plan(self, timestep, step_size=250, lookahead=2000):
        plan = self.load_plan(timestep)
        length_q = self.CoM_id.size()
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                        radius=self.CoM_radius,
                                        rgbaColor=[0.1, 1, 0, 1])
        CoM = list(plan[length_q*step_size][0:3])
        print(f"[{length_q}] COM [{CoM}]")

        visual_body_id = p.createMultiBody(baseVisualShapeIndex=visual_shape_id,
                                            basePosition=CoM,
                                            baseOrientation=self.CoM_orientation)
        self.CoM_id.enqueue(visual_body_id)

    def plot_foot_plan(self):
        raise NotImplemented

    def delete_CoM_one(self):
        if not self.CoM_id.is_empty():
            p.removeBody(self.CoM_id.dequeue())

    def delete_CoM_plan_all(self):
        for id in self.CoM_id:
            p.removeBody(id)

    def delete_foot_plan_all(self):
        for id in self.foot_id:
            p.removeBody(id)

    def delete_foot_plan_one(self):
        if not self.foot_id.is_empty():
            p.removeBody(self.foot_id.dequeue())

    def CoM_step(self, timestep):
        self.plot_Com_plan(timestep)
        self.delete_CoM_one()

if __name__ == "__main__":
    timestep = 0.395
    # plan = load_plan(TRAJ, timestep)
    pass