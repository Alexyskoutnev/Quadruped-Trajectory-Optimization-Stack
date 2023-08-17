import time 
import pybullet as p
import pybullet_data

# Connect to the simulation
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the path for resource files

# Create a visual shape (e.g., sphere)
for i in range(5):
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                        radius=0.1,
                                        rgbaColor=[1, 0, 0, 1])

    # Create a position and orientation for the visual shape
    position = [0+(i/10), 0+(i/10), 1]
    orientation = [0, 0, 0, 1]

    # Create a multi-body with the visual shape
    visual_body_id = p.createMultiBody(baseVisualShapeIndex=visual_shape_id,
                                    basePosition=position,
                                    baseOrientation=orientation)

# Step the simulation
for _ in range(100000):
    p.stepSimulation()
    time.sleep(0.001)

# Disconnect from the simulation
p.disconnect()