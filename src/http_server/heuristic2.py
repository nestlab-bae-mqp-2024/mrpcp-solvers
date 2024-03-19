from queue import PriorityQueue
import math
import numpy as np
import random

from collections import Counter

from matplotlib import pyplot, colors, pyplot as plt

from src.http_server.mrpcp import convertToWorldPath

# Global variables
all_nodes = set()
nodes_covered = set()  # nodes covered is a set of every node that has been covered so far
alpha = 0.05
robot_failure_percent = 0.1  # percent of robots that will fail as a value from 0 to 1


def run_heuristic_solver(k, q_k, n_a, rp, l, d, job_id):
    """
    This function runs the Heuristic solver function with the provided parameters.
    :return: The optimized paths and the world path
    """
    # robot fuel is a list storing each robot's fuel at the present moment
    robot_fuel = [l for ki in range(k)]

    robot_paths = generate_robot_paths_redundancy(n_a, k, rp, robot_fuel, l)

    visualize_paths_brute_force(k, n_a, robot_paths, d)

    print("Heuristic solution completed...returning paths to server endpoint /solve")
    worldPath = convertToWorldPath(n_a, d, robot_paths)
    print("The optimized paths are: ", robot_paths)
    print("The optimized paths converted to world path are: ", worldPath)
    print("Returning solution to be sent to a json file...")

    print("Recalculated paths: ", robot_paths)
    pass


def recalcRobotPaths(previous_node_path, current_robot_positions, rp, n_a, l, d):
    """
    This function recalculates the paths based on the current positions and where the failed robot starts back at the origin.
    :param previous_node_path:
    :param current_robot_positions:
    :param rp:
    :param n_a:
    :param l:
    :param d:
    :return: The recalculated node and world paths
    """
    k = len(current_robot_positions)  # number of robots
    robot_fuel = [l for ki in range(k)]  # fuel capacity of each robot
    robot_paths = previous_node_path  # previous robot paths

    # initialize all nodes
    initAllNodes(n_a)

    new_robot_paths = generate_robot_paths_redundancy(n_a, k, rp, robot_paths, robot_fuel, l)

    visualize_paths_brute_force(k, n_a, new_robot_paths)

    print("Heuristic recalculation completed...returning paths to server endpoint /solve")
    worldPath = convertToWorldPath(n_a, d, new_robot_paths)
    print("The optimized paths are: ", new_robot_paths)
    print("The optimized paths converted to world path are: ", worldPath)
    print("Returning solution to be sent to a json file...")

    print("Recalculated paths: ", new_robot_paths)

    return new_robot_paths


def initAllNodes(k, nk):
    """
    # set containing all possible nodes in the map -- not a particularly efficient way of doing this
    :param nk:
    :return:
    """
    n_a = k * nk
    for x in range(0, n_a):
        for y in range(0, n_a):
            all_nodes.add((x, y))
    return all_nodes


def neighbors(curr, n_a):
    """
    THE MAP IS ASSUMED TO BE STRUCTURED SUCH THAT (0,0) IS THE DEPOT AND THE LOWER LEFT HAND CORNER OF THE GRID
    takes in a tuple representing node that's neighbors are desired
    :param curr:
    :param n_a:
    :return:
    """
    ns = [(curr[0] + 1, curr[1]), (curr[0] - 1, curr[1]), (curr[0], curr[1] + 1), (curr[0], curr[1] - 1),
          (curr[0] + 1, curr[1] + 1), (curr[0] - 1, curr[1] - 1), (curr[0] + 1, curr[1] - 1),
          (curr[0] - 1, curr[1] + 1)]
    neighborslist = []
    for n in ns:
        if 0 <= n[0] < n_a > n[1] >= 0:
            neighborslist.append(n)
    return neighborslist


def heuristic(node):
    if node in nodes_covered:
        return -1
    else:
        return -10


def a_star_search(start, goal, n_a):
    """
    https://www.redblobgames.com/pathfinding/a-star/implementation.html#python-astar
    :param n_a:
    :param start:
    :param goal:
    :param alpha:
    :return:
    """
    robot_failed = False
    frontier = PriorityQueue()
    frontier.put((0, (start[0], start[1])))

    came_from = dict()
    cost_so_far = dict()
    came_from[(start[0], start[1])] = None
    cost_so_far[(start[0], start[1])] = 0

    while not frontier.empty():
        current = frontier.get()
        if current == (goal[0], goal[1]):
            break

        for next in neighbors(current[1], n_a):
            new_cost = cost_so_far[current[1]] + math.dist(current[1], next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next)

                frontier.put((priority, next))
                came_from[next] = current[1]

    curr_val = goal
    final_path = []
    dist = 0
    while curr_val != start:
        final_path.append(curr_val)

        dist = dist + math.dist(curr_val, came_from[curr_val])
        curr_val = came_from[curr_val]

    final_path.append(start)
    final_path.reverse()

    return final_path, dist


def calculate_costmap(n_a):
    """
    define a costmap of the field in terms of distance from depot, in the form
    :param n_a:
    :return:
    """
    map = np.zeros((n_a, n_a))
    for x in range(0, n_a):
        for y in range(0, n_a):
            map[x][y] = math.dist((0, 0), (x, y))

    return map


# finds k (# of robots) of the largest values in the costmap
# not super useful for this just anticipate it being useful i think
def find_max(n_a, k):
    map = calculate_costmap()
    max = [[0, (0, 0)] for r in range(k)]
    for x in range(0, n_a):
        for y in range(0, n_a):
            curr = map[x][y]
            if curr > max[2][0]:
                if curr > max[1][0]:
                    if curr > max[0][0]:
                        max[2] = max[1]
                        max[1] = max[0]
                        max[0] = [curr, (x, y)]
                    else:
                        max[2] = max[1]
                        max[1] = [curr, (x, y)]
                else:
                    max[2] = [curr, (x, y)]
    return max


def generate_robot_paths_redundancy(n_a, k, rp, robot_paths, robot_fuel, l):
    last_node = [(0, 0) for ki in range(k)]
    nodes_seen = []
    while n_a * n_a - len(nodes_covered) > 0:
        for ki in range(0, k):
            goal = (0, 0)
            while goal in nodes_covered and math.dist(goal, (0, 0)) < robot_fuel[ki] and len(
                    nodes_covered) < n_a * n_a:  # if goal is already covered, find a different one
                nodes_uncovered = [item for item in all_nodes if item not in nodes_covered]
                goal = random.choice(nodes_uncovered)

            path, distance_travelled = a_star_search(last_node[ki], goal, n_a)

            robot_paths[ki] = robot_paths[ki] + path

            [nodes_seen.append(p) for p in path]

            counted_nodes_seen = Counter(nodes_seen)
            for n in nodes_seen:
                if counted_nodes_seen[n] > rp:
                    nodes_covered.add(n)

            robot_fuel[ki] = robot_fuel[ki] - distance_travelled

            last_node[ki] = robot_paths[ki][len(robot_paths[ki]) - 1]

            # if robot_failed:
            #     # print("ROBOT", ki, "FAILED")
            #     last_node[ki] = (0, 0)

            # managing fuel levels
            if (0, 0) == last_node[ki]:
                robot_fuel[ki] = l

    return robot_paths


def visualize_paths_brute_force(k, n_a, robot_paths, save_path=None):
    for ki in range(k):
        fig = pyplot.figure()
        fig.suptitle(f"Path for robot #{ki}")

        past_node = (0, 0)
        [pyplot.scatter(x, y, c='blue', s=10) for x in range(0, n_a) for y in range(0, n_a)]

        for node in robot_paths[ki]:
            pyplot.scatter(node[0], node[1], c="purple", s=8)
            pyplot.plot([node[0], past_node[0]], [node[1], past_node[1]], color="purple", linewidth=1)

            past_node = (node[0], node[1])

        pyplot.grid()
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()


def animate(z, n_a, k, robot_paths):
    pyplot.ion()
    heatmap = np.zeros((n_a, n_a))
    for ki in range(k):
        if z < len(robot_paths[ki]):
            (x, y) = robot_paths[ki][z]
            heatmap[x][y] = heatmap[x][y] + 1

    plt = pyplot.imshow(heatmap[:, :], norm=colors.Normalize(0, 25))

    return plt


def generateHeatmap(n_a):
    pyplot.ion()
    heatmap = np.zeros((n_a, n_a))
    figure = pyplot.figure(figsize=(5, 5))
    plt = pyplot.imshow(heatmap[:, :], norm=colors.Normalize(0, 25))

    pyplot.colorbar().set_ticks([0, 25])


# print(heatmap)
# number_of_steps = max(len(robot_paths[ki]) for ki in range(k))
# ani = animation.FuncAnimation(figure, animate, interval=number_of_steps, frames=number_of_steps)
# ani.save("data/" + str(n_a) + "n" + str(k) + "r" + str(edge_length) + ".gif", fps=10)


# sending to ArGoS
"""
import xml.etree.ElementTree as ET

mytree = ET.parse('diffusion_1_positioning.argos')
myroot = mytree.getroot()
print(myroot.attrib)

edge = [x/10 for x in range(int(-10*edge_length/2) , int(10*edge_length/2)+1, int(10*edge_length/(nodes_per_axis-1)))]
path_string = "-1 -1, "
for r in robot_paths[0]:
    path_string = path_string + str(edge[r[0]]) + " " + str(edge[r[1]]) + ", "

for x in myroot.iter('params'):
  print(x.get('path'))
  x.set('path', path_string)
  x.set('path_length', str(1+len(robot_paths[0])))

mytree.write('diffusion_2_positioning.argos')
"""
