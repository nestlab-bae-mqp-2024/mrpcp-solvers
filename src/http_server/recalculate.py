"""
Author: Samara Holmes
Date: 2/17/2024

This program defines the methods relating to recalculating the paths for robots after a failure occurs
"""

import os
from flask import json

from src.http_server.heuristic2 import *
from src.http_server.json_handlers import saveResultsToCache
from src.http_server.mrpcp import saveGraphPath, convertToWorldPath

# Global variables
all_nodes = set()
nodes_covered = set()  # nodes covered is a set of every node that has been covered so far
alpha = 0.05
robot_failure_percent = 0.1  # percent of robots that will fail as a value from 0 to 1

def recalculate_paths(job_id, curr_robots_pos, failed_robot_id):
    """
    This function recalculates the paths based on the current positions and where the failed robot starts back at the origin.
    """
    # Convert the current robot positions back to an integer array
    curr_robots_pos = [int(pos) for pos in curr_robots_pos]

    # Define the cache folder path relative to the current directory
    cache_folder_path = os.path.join(os.getcwd(), 'cache')

    # Check if the job folder exists
    job_folder_path = os.path.join(cache_folder_path, job_id)
    if not os.path.exists(job_folder_path):
        print(f"Job folder does not exist: {job_folder_path}.")
        return

    # Read the result.json file to retrieve the parameters
    result_file_path = os.path.join(job_folder_path, 'result.json')
    if not os.path.exists(result_file_path):
        print(f"Result file does not exist: {result_file_path}.")
        return

    # Parse the JSON content to extract the parameters
    with open(result_file_path, 'r') as file:
        result_data = json.load(file)
        params = result_data.get('params')
        previous_robot_node_path = result_data.get('robot_node_path')

    # Use the extracted parameters for recalculation
    params_tuple = getParamsFromJobId(job_id)
    k, nk, ssd, fcr, fr, mode = params_tuple

    print("Previous robot paths:", previous_robot_node_path)
    print("Current robot positions:", curr_robots_pos)

    # Convert the current (x,y) world positions to node positions. For the failed robot, round down to the nearest node position. For others, just do normal calculation.
    new_robot_paths = recalcRobotPaths(previous_robot_node_path, curr_robots_pos, int(k), int(nk), int(ssd), float(fcr), int(fr), int(failed_robot_id))
    print("New robot paths:", new_robot_paths)
    # visualize the new paths and save the graph to the cache
    visualize_recalculated_paths(new_robot_paths, int(k), int(nk), int(ssd), saveGraphPath(job_id, 'recalculated_paths'))

    result_data = {'job_id': job_id, 'params': {'k': k, 'nk': nk, 'ssd': ssd, 'fcr': fcr, 'fr': fr, 'mode': 'recalc'},
                   'robot_node_path': new_robot_paths,
                   'robot_world_path': convertToWorldPath(int(nk), ssd, new_robot_paths)}
    saveResultsToCache(job_id, result_data, 'recalculated_paths.json')
    return result_data  # Return the content of the JSON file


def recalcRobotPaths(previous_robot_node_path, curr_robots_pos, num_of_robots: int,
                     nodes_to_robot_ratio: int,
                     square_side_dist: float,
                     fuel_capacity_ratio: float,
                     failure_rate: int):
    """
    This function recalculates the paths based on the current positions and where the failed robot starts back at the origin.
    :param previous_node_path:
    :param current_robot_positions:
    :param failed_robot_id:
    :return: The recalculated node and world paths
    """
    k = num_of_robots  # number of robots
    n_a = k * nodes_to_robot_ratio  # number of targets in an axis
    d = square_side_dist # Chose the length of distance of each side of the square arena
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    MDBF = 100.0  # Mean Distance Between Failures
    alpha = 0.00001*failure_rate
    rpp = alpha * MDBF  # redundancy parameter percentage
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    rp = np.ceil(k * rpp) + 1
    # Fuel Capacity Parameters
    max_fuel_cost_to_node = d * np.sqrt(2)  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L_min = max_fuel_cost_to_node * 2  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L = L_min * fuel_capacity_ratio  # Fuel capacity (1 unit of fuel = 1 unit of distance)

    robot_fuel = [L for ki in range(k)] #robot fuel is a list storing each robot's fuel at the present moment
    robot_paths = previous_robot_node_path  # previous robot paths

    # initialize all nodes
    initAllNodes(k, nodes_to_robot_ratio)

    new_robot_paths = generate_robot_paths_redundancy(n_a, k, rp, robot_paths, robot_fuel, L)

    print("New robot paths:", new_robot_paths)
    # visualize_paths_brute_force(k, n_a, new_robot_paths)

    print("Heuristic recalculation completed...returning paths to server endpoint /solve")
    worldPath = convertToWorldPath(n_a, d, new_robot_paths)
    print("The optimized paths are: ", new_robot_paths)
    print("The optimized paths converted to world path are: ", worldPath)
    print("Returning solution to be sent to a json file...")

    print("Recalculated paths: ", new_robot_paths)

    return new_robot_paths, worldPath


def getIndexOf(path, position):
    try:
        return path.index(position)
    except ValueError:
        return -1

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

# %%
