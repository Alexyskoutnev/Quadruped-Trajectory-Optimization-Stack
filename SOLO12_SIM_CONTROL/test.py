import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import osqp
import scipy as sp
from scipy.sparse import csc_matrix

# # Define the cost function to minimize trajectory length
def cost_function_track_OSQP(expert_traj):
    n = expert_traj.shape[0] * expert_traj.shape[1]
    delta = 1.0

    residual = -expert_traj
    huber_loss = np.sum(np.where(np.abs(residual) <= delta, 0.5 * residual**2, delta * (np.abs(residual) - 0.5 * delta)))
    P = csc_matrix(np.identity(n))  # Identity matrix for the quadratic term
    q = -huber_loss  # Coefficients for the linear term
    return P, q

def cost_function_spread(x):
    _x = x.reshape((-1, 3))
    return np.sum(np.linalg.norm(np.diff(_x, axis=0), axis=1))

def cost_function_track(x, expert_traj):
    error = np.linalg.norm(x - expert_traj, ord=1)
    return error

def smoothness_penalty(x):
    jerk = np.diff(np.diff(x, axis=0), axis=0)
    jerk_norm = np.linalg.norm(jerk, axis=1)
    penalty = np.sum(jerk_norm**2)
    return penalty

def spacing_penalty(x):
    squared_distances = np.sum(np.diff(x, axis=0) ** 2, axis=1)
    regularization_term = np.sum(squared_distances)
    return regularization_term

def length_penalty(x):
    return np.sum(np.linalg.norm(np.diff(x, axis=0), axis=1))

def cost_function(x, expert_traj, sphere_radius, sphere_center):
        potential_field_values = calculate_potential_field(x, sphere_radius, sphere_center)
        obstacle_cost = np.sum(potential_field_values)
        _x = x.reshape((-1, 3))
        jerk_cost = smoothness_penalty(_x)
        track_cost = cost_function_track(_x, expert_traj)
        traj_length_cost = length_penalty(_x)
        spacing_cost = spacing_penalty(_x)
        total_cost = obstacle_cost * 3.0 + spacing_cost * 2.0 + traj_length_cost + jerk_cost * 2.5  + track_cost 
        return total_cost

def calculate_potential_field(x, sphere_radius, sphere_center):
    _x = x.reshape((-1, 3))
    distances = np.linalg.norm(_x - sphere_center, axis=1)
    potential_field_values = np.where(distances < sphere_radius, 1.0 / distances, 0.0)
    return potential_field_values

def cost_function_with_penalty(x):
    cost = cost_function(x)
    _x = x.reshape((-1, 3))
    penalty = 0.0
    for point in _x:
        distance = np.linalg.norm(point - sphere_center)
        if distance < sphere_radius:
            penalty += (sphere_radius - distance) ** 2
    return cost + penalty

if __name__ == "__main__":
    # Define start and end goals
    start_pos = np.array([-5, -5, 1])
    end_pos = np.array([5, 5, 1])

    # Define the number of nodes for the trajectory
    num_nodes = 100

    expert_traj = np.linspace(start_pos, end_pos, num_nodes)

    # Generate an initial guess for the trajectory nodes
    init_guess = np.linspace(start_pos, end_pos, num_nodes).flatten()
    # init_guess = np.zeros(num_nodes * 3)

    # Define the center and radius of the avoidance sphere
    sphere_center = np.array([0.1, 0.1, 0.1])
    sphere_radius = 2.0

    # Define the constraints including start, end, and sphere avoidance
    constraints = [
        # {'type': 'eq', 'fun': lambda x: x[0:3] - start_pos},
        # {'type': 'eq', 'fun': lambda x: x[-3:] - end_pos},
    #     {'type': 'ineq', 'fun': sphere_avoidance_constraint}
    ]
    # #     {'type': 'ineq', 'fun': lambda x: sphere_avoidance_constraint(x, sphere_center, sphere_radius)}
    # ]

    #OSQP formulation
    # sp.random.seed(1)
    # P, q = cost_function_track_OSQP(expert_traj)

    A = np.eye(init_guess.shape[0])
    b = init_guess + 1
    P = csc_matrix(np.dot(A.T, A))
    q = -np.dot(A.T, b)
    A_eq = csc_matrix(A)  # Empty sparse matrix
    b_eq = init_guess
    l = None # Empty array
    u = None  # Empty array

    #Sphere Constraint
    center = np.array([0.0, 0.0])  # Sphere center
    radius = 2.0  # Sphere radius
    n = A.shape[1]  # Number of decision variables
    A_sphere = np.vstack([2.0 * csc_matrix(np.eye(n)), -2.0 * csc_matrix(np.eye(n))])
    b_sphere = np.hstack([2.0 * (center + radius), -2.0 * (center - radius)])

    A_constraints = np.vstack([A_eq, A_sphere])
    b_constraints = np.hstack([b_eq, b_sphere])

    l_constraints = np.hstack([b_eq, b_sphere])  # Lower bounds
    u_constraints = np.inf * np.ones_like(l_constraints)  # Upper bounds

    breakpoint()

    prob = osqp.OSQP()

    prob.setup(P, q, A_constraints, l_constraints, u_constraints)

    res = prob.solve()
    optimized_trajectory = res.x.reshape((-1, 3))


    # Optimize the trajectory
    # result = minimize(cost_function, init_guess, args=(expert_traj, sphere_radius, sphere_center), options={'maxiter': 1000}, method="SLSQP")  # Pass expert_traj as an argument
    # optimized_trajectory = result.x.reshape((-1, 3))
    breakpoint()

    # Plot the optimized trajectory and the sphere constraint
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(optimized_trajectory[:, 0], optimized_trajectory[:, 1], optimized_trajectory[:, 2], marker='o')

    # Plot the sphere constraint
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = sphere_center[0] + sphere_radius * np.outer(np.cos(u), np.sin(v))
    y = sphere_center[1] + sphere_radius * np.outer(np.sin(u), np.sin(v))
    z = sphere_center[2] + sphere_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='red', alpha=0.3)

    # Set plot limits and labels
    ax.set_xlim(0, 4)
    ax.set_ylim(0, 5)
    ax.set_zlim(0, 2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()