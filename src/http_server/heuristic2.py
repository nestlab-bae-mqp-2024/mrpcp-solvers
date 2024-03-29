from queue import PriorityQueue
import math
import numpy as np
import random

from collections import Counter

from matplotlib import pyplot, colors

from src.http_server.mrpcp import convertToWorldPath

from src.http_server.utils.visualize import visualize_coverage, visualize_heatmap
# from src.http_server.mrpcp import convertToWorldPath

# Global variables
all_nodes = set()
nodes_covered = set()  # nodes covered is a set of every node that has been covered so far
alpha = 0.05
robot_failure_percent = 0.1  # percent of robots that will fail as a value from 0 to 1


def generate_robot_paths_redundancy(num_of_robots: int,
                                    nodes_to_robot_ratio: int,
                                    square_side_dist: float,
                                    fuel_capacity_ratio: float,
                                    failure_rate: int,
                                    failed_robot_id: int = None,
                                    curr_robots_pos: list = None,
                                    curr_fuel_levels: list = None,
                                    visualization_path: str = None):
    """
    This function solves the MRPCP problem using the heuristic approach with redundancy and failure rate. This is NOT recalculation
    :return: The optimized paths and the world path
    """
    # Define the parameters
    print("Initializing parameters...")
    k = num_of_robots  # number of robots
    n_a = k * nodes_to_robot_ratio  # number of targets in an axis
    d = square_side_dist  # Chose the length of distance of each side of the square arena
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    MDBF = 100.0  # Mean Distance Between Failures
    alpha = 0.00001 * failure_rate
    rpp = alpha * MDBF  # redundancy parameter percentage
    rp = np.ceil(k * rpp) + 1

    # Fuel Capacity Parameters
    max_fuel_cost_to_node = d * np.sqrt(2)  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L_min = max_fuel_cost_to_node * 2  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L = L_min * fuel_capacity_ratio  # Fuel capacity (1 unit of fuel = 1 unit of distance)

    # Initialize the nodes
    print("Initializing nodes...")
    initAllNodes(k, nodes_to_robot_ratio)
    dist_betw_each_node = square_side_dist / (n_a - 1)

    # logic for if recalc or not
    if failed_robot_id is None and curr_fuel_levels is None and curr_robots_pos is None:
        print("Conducting heuristic2")
        print("Initializing robot fuel levels...")
        robot_fuel = [L for ki in range(k)]  # robot fuel is a list storing each robot's fuel at the present moment
        print("Initializing last node...")
        last_node = [(0,0) for ki in range(k)]
    else:
        print("Conducting recalculation")
        world_posns = convertToWorldPath(n_a, square_side_dist, curr_robots_pos)
        robot_fuel = curr_fuel_levels
        last_node = [(round((square_side_dist / 2 + world_posns[ki][0]) / (dist_betw_each_node)),
                      round((square_side_dist / 2 + world_posns[ki][1]) / (dist_betw_each_node))) for ki in range(k)]
        last_node[failed_robot_id] = (0, 0)

    robot_paths = [[] for ki in range(k)]
    nodes_seen = []

    while n_a * n_a - len(nodes_covered) > 0:
        for ki in range(0, k):
            goal = (0, 0)
            while goal in nodes_covered and math.dist(goal, (0, 0)) < robot_fuel[ki] and len(
                    nodes_covered) < n_a * n_a:  # if goal is already covered, find a different one
                nodes_uncovered = [item for item in all_nodes if item not in nodes_covered]

                max_dist = 0
                goal = (0, 0)
                for n in nodes_uncovered:
                    if math.dist((0, 0), n) > max_dist:
                        max_dist = math.dist((0, 0), n)
                        goal = n

            path, distance_travelled, robot_failed = a_star_search(last_node[ki], goal, n_a)

            robot_paths[ki] = robot_paths[ki] + path

            [nodes_seen.append(p) for p in path]

            counted_nodes_seen = Counter(nodes_seen)

            for n in nodes_seen:
                if counted_nodes_seen[n] >= rp:
                    nodes_covered.add(n)

            robot_fuel[ki] = robot_fuel[ki] - distance_travelled

            last_node[ki] = robot_paths[ki][len(robot_paths[ki]) - 1]

            # managing fuel levels
            if (0, 0) == last_node[ki]:
                robot_fuel[ki] = L

    world_path = [[] for ki in range(k)]

    for ki in range(0, k):
        path = []
        for item in robot_paths[ki]:
            path.append([dist_betw_each_node * item[0] - square_side_dist / 2,
                         dist_betw_each_node * item[1] - square_side_dist / 2])
        world_path[ki] = path

    visualize_paths_brute_force(n_a, robot_paths, visualization_path)
    visualize_coverage(20, 1000, n_a, square_side_dist, robot_paths, None, visualization_path)
    visualize_heatmap(20, 1000, n_a, square_side_dist, robot_paths, None, visualization_path)
    return robot_paths, world_path


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


# takes in a tuple representing node that's neighbors are desired
def neighbors(curr, nodes_per_axis):
    ns = [(curr[0] + 1, curr[1]), (curr[0] - 1, curr[1]), (curr[0], curr[1] + 1), (curr[0], curr[1] - 1),
          (curr[0] + 1, curr[1] + 1), (curr[0] - 1, curr[1] - 1), (curr[0] + 1, curr[1] - 1),
          (curr[0] - 1, curr[1] + 1)]
    neighbors = []
    for n in ns:
        if n[0] < nodes_per_axis and n[0] >= 0 and n[1] < nodes_per_axis and n[1] >= 0:
            neighbors.append(n)
    return neighbors


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

        neighbor_list = neighbors(current[1], n_a)
        for next in neighbor_list:
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

    i = len(final_path)
    # if random.random() < alpha:
    #    i = random.randrange(1, len(final_path)+1, 1)
    #    robot_failed = True

    return final_path[:i], dist, robot_failed

def visualize_paths_brute_force(n_a, robot_paths, visualization_path=None):
    num_rows = (len(robot_paths) + 1) // 2  # Two plots per row
    fig, axs = pyplot.subplots(num_rows, 2, figsize=(10, 5 * num_rows))  # Adjust the figure size as needed


    if len(robot_paths) > 2:
        axs = axs.flatten()

    for ki in range(len(robot_paths)):
        ax = axs[ki]

        past_node = (0, 0)
        [ax.scatter(x, y, c='blue', s=10) for x in range(0, n_a) for y in range(0, n_a)]

        for node in robot_paths[ki]:
            ax.scatter(node[0], node[1], c="purple", s=8)
            ax.plot([node[0], past_node[0]], [node[1], past_node[1]], color="purple", linewidth=1)

            past_node = (node[0], node[1])

        ax.set_title(f"Robot #{ki}")
        ax.grid()
        ax.legend()

    if visualization_path:
        pyplot.savefig(visualization_path.replace("visualization.png", "h2_visualization.png"))
    else:
        pyplot.show()

# %%
