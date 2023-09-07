import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the cube parameters
cube_center = np.array([2, 2, 2])  # Center of the cube
cube_size = 2.0  # Size of the cube (edge length)

# Define the start and end goals
start_position = np.array([0, 0, 0])
end_position = np.array([4, 4, 4])

# Define the initial position of the point
initial_position = start_position

# Define the objective function to minimize the distance to the cube's surface
def objective_function(position):
    distance_to_center = np.linalg.norm(position - cube_center)
    distance_to_surface = distance_to_center - cube_size / 2
    return distance_to_surface

# Define the constraint to avoid collision with the cube
def collision_constraint(position):
    return np.linalg.norm(position - cube_center) - cube_size / 2

# Formulate the optimization problem
constraints = [{'type': 'ineq', 'fun': collision_constraint}]
result = minimize(objective_function, initial_position, constraints=constraints, method='SLSQP')

# Extract the optimized trajectory
optimized_trajectory = result.x

# Visualization using Matplotlib
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the cube
r = cube_size / 2
x, y, z = cube_center
ax.plot([x - r, x + r, x + r, x - r, x - r],
        [y - r, y - r, y + r, y + r, y - r],
        [z - r, z - r, z - r, z - r, z - r], color='b')

# Plot the start and end goals
ax.scatter(start_position[0], start_position[1], start_position[2], color='g', label='Start Goal', marker='o', s=100)
ax.scatter(end_position[0], end_position[1], end_position[2], color='y', label='End Goal', marker='o', s=100)

# Plot the optimized trajectory
ax.plot(optimized_trajectory[0], optimized_trajectory[1], optimized_trajectory[2], label='Optimized Trajectory', color='r', marker='o')

# Set plot limits and labels
ax.set_xlim(0, 4)
ax.set_ylim(0, 4)
ax.set_zlim(0, 4)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.legend()
plt.show()