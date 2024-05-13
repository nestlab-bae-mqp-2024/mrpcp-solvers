from queue import PriorityQueue
import math
from typing import Dict

import numpy as np
from collections import Counter
import time

from src.http_server.utils.conversions import convertToWorldPath

# Global variables
all_nodes = set()
nodes_covered = set()  # nodes covered is a set of every node that has been covered so far


def generate_robot_paths_redundancy(num_of_robots: int,
                                    nodes_per_axis: int,
                                    square_side_dist: float,
                                    fuel_capacity_ratio: float,
                                    rp: int,
                                    curr_robots_pos: list = None,
                                    curr_fuel_levels: list = None,
                                    metadata: Dict = None):
    """
    This function solves the MRPCP problem using the heuristic approach with redundancy and failure rate. This is NOT recalculation
    :return: The optimized paths and the world path
    """
    if metadata is None:
        metadata = {}

    # Define the parameters
    print("Initializing parameters...")
    # 0. Set params
    # Chose number of robots
    k = num_of_robots
    # Chose the number of targets in an axis
    n_a = nodes_per_axis
    # Chose the length of distance of each side of the square arena
    ssd = square_side_dist
    # Chose the length of distance between the outermost nodes of each axis
    sr = ssd / (np.sqrt(2.) * n_a)
    d = ssd - np.sqrt(2.) * sr
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    rp = min(rp, k)
    # Fuel Capacity Parameters
    max_fuel_cost_to_node = d * np.sqrt(2)  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L_min = max_fuel_cost_to_node * 2  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L = L_min * fuel_capacity_ratio  # Fuel capacity (1 unit of fuel = 1 unit of distance)

    # meta data given params
    metadata["k"] = k
    metadata["n_a"] = n_a
    metadata["ssd"] = ssd
    metadata["ssd_discrete"] = d
    metadata["fcr"] = fuel_capacity_ratio
    metadata["rp"] = rp
    # metadata derived params
    metadata["L_min"] = L
    metadata["mode"] = "h2"

    # Initialize the nodes
    print("Initializing nodes...")

    all_nodes = initAllNodes(n_a)

    dist_betw_each_node = square_side_dist / (n_a - 1)

    # logic for if recalc or not
    if curr_fuel_levels is None and curr_robots_pos is None:
        print("Conducting heuristic2")
        print("Initializing robot fuel levels...")
        robot_fuel = [L for ki in range(k)]  # robot fuel is a list storing each robot's fuel at the present moment
        print("Initializing last node...")
        last_node = [(0, 0) for ki in range(k)]
    else:
        print("Conducting recalculation")
        world_posns = curr_robots_pos
        robot_fuel = curr_fuel_levels
        last_node = [(round((square_side_dist / 2 + world_posns[ki][0]) / (dist_betw_each_node)),
                      round((square_side_dist / 2 + world_posns[ki][1]) / (dist_betw_each_node))) for ki in range(k)]
        metadata["mode"] = "recalc"
        print(metadata)

    robot_paths = [[] for ki in range(k)]
    nodes_seen = []

    nodes_covered = set()  # nodes covered is a set of every node that has been covered so far

    nodes_uncovered = []

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
        world_path[ki] = [path]

    return robot_paths, world_path, metadata


def initAllNodes(n_a):
    """
    # set containing all possible nodes in the map -- not a particularly efficient way of doing this
    :param n_a:
    :return:
    """
    all_nodes.clear()

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
        if nodes_per_axis > n[0] >= 0 and nodes_per_axis > n[1] >= 0:
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

    return final_path[:i], dist, robot_failed


if __name__ == "__main__":
    num_of_robots = 8
    n_a = 8
    square_side_dist = 3.
    fuel_capacity_ratio = 1.5
    rp = 3

    from src.http_server.json_handlers import saveGraphPath

    metadata = {"mode": "h2",
                "v": 0.2,
                "t": 100.,
                "dt": 0.1,
                "lookback_time": 30.
                # "visualize_paths_graph_path": saveGraphPath("yasars-heuristic-main", "all_robot_paths.png"),
                # "visitation_frequency_graph_path": saveGraphPath("yasars-heuristic-main", "visitation_frequency.png")
                }
    start = time.time()
    optimized_node_paths, optimized_world_paths, metadata = generate_robot_paths_redundancy(num_of_robots,
                                                                                            n_a,
                                                                                            square_side_dist,
                                                                                            fuel_capacity_ratio,
                                                                                            rp,
                                                                                            None,
                                                                                            None,
                                                                                            metadata)
    print(f"took {time.time() - start} seconds")

