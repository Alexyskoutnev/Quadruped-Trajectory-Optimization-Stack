
import numpy as np
import matplotlib.pyplot as plt
import heapq
from scipy.optimize import minimize

TOWR_HEIGHT_MAP = "../data/heightmaps/test_heightfield_towr.txt"

def heuristic(a, b):
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def astar(map, start, goal, height_bound=1.0):

    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    close_set = set()
    came_from = {}
    gscore = {start: 0 }
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    i = 0

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            print("Found a path")
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return [start] + data[::-1]
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            _g_score = gscore[current] + heuristic(current,  neighbor) #Current min cost from start node [start node] to current [n]
            if 0 <= neighbor[0] < map.shape[0] and 0 <= neighbor[1] < map.shape[1]: #Check for a feasible solution
                if map[neighbor[0]][neighbor[1]] >= height_bound:
                    continue
            else:
                continue
            if neighbor in close_set and _g_score >= gscore.get(neighbor, 0): #skipping if neighbor has been visited again
                continue
            if _g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]: #
                came_from[neighbor] = current #store the path where n+1 node came from 
                gscore[neighbor] = _g_score
                fscore[neighbor] = _g_score + heuristic(neighbor, goal) # f(n) = g(n) + h(n) Compute the total cost of the node 
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return None

def obj_function(path, map, start, goal):
    total_cost = 0
    current = start
    for i in path:
        total_cost += np.linalg.norm(np.array(current) - np.array(i))
        current = i
    total_cost += np.linalg.norm(np.array(current) - np.array(goal))
    return total_cost

def path_optimizer(map, start, goal):
    bounds = [(0, map.shape[0] - 1), (0, map.shape[1] - 1)] * (map.size - 2)
    breakpoint()
    initial_guess = [start] * (map.size - 2)

    result = minimize(obj_function, initial_guess, args=(map, start, goal), bounds=bounds, method='TNC')

    if result.success:
        path = [start] + result.x + [goal]
        return path
    else:
        return None

def is_numeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def plot_height_map(start_pos, end_pos, map, path):
    # Create a grid of x and y coordinates for the map
    x = np.arange(0, map.shape[1], 1)
    y = np.arange(0, map.shape[0], 1)

    # Create a meshgrid for x and y coordinates
    X, Y = np.meshgrid(x, y)

    # Plot the grid map using Matplotlib's contourf function
    plt.figure(figsize=(8, 6))
    plt.contourf(X, Y, map, cmap='terrain')

    dots_x = [start_pos[0], end_pos[0]]
    dots_y = [start_pos[1], end_pos[1]]
    plt.scatter(dots_x, dots_y, color='red', marker='o', label='Dots')

    plt.colorbar(label='Height')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.show()


def txt_2_np_reader(file, delimiter=','):
    data = []
    with open(file, 'r') as f:
        reader = f.readlines()
        for row in reader:
            _row = row.strip().split(delimiter)
            _row = [float(x) for x in _row if is_numeric(x)]
            data.append(_row)
    return np.transpose(np.array(data))

if __name__ == "__main__":
    # map = txt_2_np_reader(TOWR_HEIGHT_MAP)
    # start = (0, 10)
    # goal = (30, 10) 
    map = np.array([[0, 0, 0, 0, 0],
                     [0, 1, 0, 1, 0],
                     [0, 0, 0, 0, 0],
                     [0, 1, 0, 1, 0],
                     [0, 0, 0, 0, 0]])

    start = (0, 0)
    goal = (4, 4)
    path = astar(map, start, goal)
    

    plot_height_map(start, goal, map, path)


    heights = np.array([[0.0, 0.0, 0.0, 0.4],
                    [0.5, 0.0, 0.7, 0.8],
                    [0.9, 0.0, 1.1, 1.2],
                    [1.3, 0.0, 0.2, 1.6]])

    start = (0, 0)
    goal = (3, 3)
    path = astar(map, start, goal, height_bound=0.5)
    # plot_height_map(start_pos, goal_pos, map)
    breakpoint()