"""
Author: Samara Holmes
Date: 2/17/2024

This program defines the methods relating to recalculating the paths for robots after a failure occurs
"""
import math
import os
from flask import json

#from src.http_server.heuristic2 import *
#from src.http_server.json_handlers import saveResultsToCache
#from src.http_server.mrpcp import saveGraphPath, convertToWorldPath

# Global variables
all_nodes = set()
nodes_covered = set()  # nodes covered is a set of every node that has been covered so far
alpha = 0.05
def recalculate_paths(job_id,
                      num_of_robots: int,
                      nodes_to_robot_ratio: int,
                      square_side_dist: float,
                      fuel_capacity_ratio: float,
                      failure_rate: int,
                      curr_robots_pos, failed_robot_id):
    """
    This function recalculates the paths based on the current positioFns and where the failed robot starts back at the origin.
    :return: The recalculated node and world paths
    """
    # Define the parameters
    k = num_of_robots  # number of robots
    n_a = k * nodes_to_robot_ratio  # number of targets in an axis
    d = square_side_dist # Chose the length of distance of each side of the square arena
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    MDBF = 100.0  # Mean Distance Between Failures
    alpha = 0.00001*failure_rate
    rpp = alpha * MDBF  # redundancy parameter percentage
    rp = np.ceil(k * rpp) + 1

    # Fuel Capacity Parameters
    max_fuel_cost_to_node = d * np.sqrt(2)  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L_min = max_fuel_cost_to_node * 2  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L = L_min * fuel_capacity_ratio  # Fuel capacity (1 unit of fuel = 1 unit of distance)

    robot_fuel = [L for ki in range(k)] #robot fuel is a list storing each robot's fuel at the present moment

    # Convert the current robot positions back to an integer array
    curr_robots_pos = [int(pos) for pos in curr_robots_pos]

    print("Current robot positions:", curr_robots_pos)

    # initialize all nodes
    initAllNodes(k, nodes_to_robot_ratio)

    # Convert the current (x,y) world positions to node positions. For the failed robot, round down to the nearest node position. For others, just do normal calculation.
    new_robot_paths = generate_robot_paths_redundancy_failure(int(k), int(n_a), L, int(ssd), int(fr), curr_robots_pos,  int(failed_robot_id))
    worldPath = convertToWorldPath(n_a, d, new_robot_paths)

    print("Heuristic recalculation completed...returning paths to server endpoint /solve")
    print("New robot paths:", new_robot_paths)

    print("The optimized paths converted to world path are: ", worldPath)
    print("Returning solution to be sent to a json file...")

    # visualize the new paths and save the graph to the cache
    visualize_recalculated_paths(new_robot_paths, int(k), int(nk), int(ssd), saveGraphPath(job_id, 'recalculated_paths'))
    return new_robot_paths, worldPath   # Return the content of the JSON file

def visualize_recalculated_paths(paths, robots, targets, d, save_path=None):
    k = robots
    # Chose the number of targets in an axis
    n_a = int(targets)
    d = d / 2.  # Distance per side of the square
    targets = np.mgrid[-d:d:n_a * 1j, -d:d:n_a * 1j]  # Size d x d
    targets = targets.reshape(targets.shape + (1,))
    targets = np.concatenate((targets[0], targets[1]), axis=2)
    targets = targets.reshape((n_a * n_a, 2))
    depots = np.array([
        [-1., -1.],
    ]) * d
    depots = np.concatenate((depots, depots))

    depots = np.concatenate((depots, depots))
    nodes = np.concatenate((targets, depots))
    depot_indices = range(len(targets), len(targets) + len(depots))

    nodes = np.concatenate((targets, depots))
    B_k = np.array([depot_indices[0]] * k)

    num_robots = len(paths)
    num_rows = (num_robots + 1) // 2  # Two plots per row
    fig, axs = plt.subplots(num_rows, 2, figsize=(10, 5 * num_rows))  # Adjust the figure size as needed

    # Flatten the axs array for easy iteration if there's more than one row
    if num_robots > 2:
        axs = axs.flatten()

    for index, path in enumerate(paths):
        ax = axs[index]

        # Plot targets and depots
        ax.scatter(targets[:, 0], targets[:, 1], c='blue', s=10, label='Targets')
        ax.scatter(depots[:, 0], depots[:, 1], c='red', s=50, label='Depots')

        # Plot path for this robot
        for i in range(len(path) - 1):
            start_node = path[i]
            end_node = path[i + 1]
            ax.plot([nodes[start_node, 0], nodes[end_node, 0]],
                    [nodes[start_node, 1], nodes[end_node, 1]],
                    color="purple", linewidth=1)
            ax.scatter(nodes[start_node, 0], nodes[start_node, 1], c="purple", s=8)
            ax.text(nodes[start_node, 0], nodes[start_node, 1], str(start_node), fontsize=8, ha='center', va='center')

        # Plot a line returning to the starting depot
        ax.plot([nodes[path[-1], 0], nodes[B_k[0], 0]],
                [nodes[path[-1], 1], nodes[B_k[0], 1]],
                color="purple", linewidth=1, linestyle="--", label='Return to Depot')

        # Plot the starting depot
        ax.text(nodes[B_k[0], 0], nodes[B_k[0], 1], str(B_k[0]), fontsize=8, ha='center', va='center')

        # Set title with cost
        ax.set_title(f"Robot #{index + 1}")
        ax.grid()
        ax.legend()

    # Hide any unused subplots
    for i in range(index + 1, num_rows * 2):
        fig.delaxes(axs[i])

    # plt.tight_layout()
    fig.suptitle(f"Paths for all robots")

    # Save the figure if save_path is provided
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()


def convertToNodePath(world_path):
    node_path = []
    for i in range(len(world_path)):
        if i == 0:
            node_path.append([world_path[i]])
        else:
            node_path.append([world_path[i], world_path[i - 1]])
    return node_path


def convertToNodePosition(world_position):
    return world_position


def getParamsFromJobId(job_id):
    """
    Function to return all the MRPCP and Heuristic parameters based on the job id
    It takes the job id and splits the id into the parameters
    :param job_id:
    :return:
    """
    k, q_k, n_a, rp, l, d, mode = job_id.split('_')
    return k, q_k, n_a, rp, l, d, mode




def generate_robot_paths_redundancy_failure(k, n_a, L, ssd, failure_rate, curr_robots_pos, failed_robot_id):
    """
    This function solves the MRPCP problem using the heuristic approach where the failed robot starts back at the origin.
    :return: The optimized paths and the world path
    """

    world_posns = convertToWorldPath(n_a, d, curr_robots_pos)

    dist_betw_each_node = ssd/(n_a-1)
    robot_paths = [[] for ki in range(k)]

    last_node = [(round((ssd/2 + world_posns[ki][0])/(dist_betw_each_node)), round((ssd/2 + world_posns[ki][1])/(dist_betw_each_node))) for ki in range(k)]

    last_node[failed_robot_id] = (0,0)

    while n_a*n_a - len(nodes_covered) > 0:
        for ki in range(0,k):
            goal = (0,0)
            while goal in nodes_covered and math.dist(goal, (0,0) < robot_fuel[ki]) and len(nodes_covered) < n_a*n_a: #if goal is already covered, find a different one
                nodes_uncovered = [item for item in all_nodes if item not in nodes_covered]

                max_dist = 0
                goal = (0,0)
                for n in nodes_uncovered:
                    if math.dist((0,0),n) > max_dist:
                        max_dist = math.dist((0,0),n)
                        goal = n


            path, distance_travelled, robot_failed = a_star_search(last_node[ki], goal, n_a)

            robot_paths[ki] = robot_paths[ki] + path

            [nodes_seen.append(p) for p in path]

            counted_nodes_seen = Counter(nodes_seen)

            for n in nodes_seen:
                if counted_nodes_seen[n] >= RP:
                    nodes_covered.add(n)

            robot_fuel[ki] = robot_fuel[ki] - distance_travelled

            last_node[ki] = robot_paths[ki][len(robot_paths[ki])-1]

            #managing fuel levels
            if (0,0) == last_node[ki]:
                robot_fuel[ki] = L

    world_path = [[] for ki in range(k)]

    for ki in range(0,k):
        path = []
        for item in robot_paths[ki]:
            path.append([dist_betw_each_node*item[0]-ssd/2, dist_betw_each_node*item[1]-ssd/2])
        world_path[ki] = path

    return robot_paths
