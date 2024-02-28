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
    k, q_k, n_a, rp, l, d, mode = getParamsFromJobId(job_id)

    # Lets say that the we use 6_0.5_4 for the parameters (this will be the edges)
    # [[16, 17, 10, 14, 9], [16, 17, 0, 1, 3, 2], [16, 17, 4, 8, 12, 13, 5], [16, 17, 7], [16, 17, 15], [16, 17, 6, 11]]

    # Example current robot positions
    ex_robot_positions = [10, 0, 12, 7, 15, 11]  # 1 index based
    ex_failed_robot_id = 1

    # Example new robot paths if the robots just need to finish where they left off
    # [[16, 14, 9], [0, 1, 3, 2], [12, 13, 5], [7, 16], [15, 16], [11, 16]]
    print("Previous robot paths:", previous_robot_node_path)
    print("Current robot positions:", curr_robots_pos)

    # Convert the current (x,y) world positions to node positions. For the failed robot, round down to the nearest node position. For others, just do normal calculation.

    # new_robot_paths = recalcRobotPaths(previous_robot_node_path, curr_robots_pos, int(rp), int(n_a),
    #                                    int(failed_robot_id))
    new_robot_paths = recalcRobotPaths2(previous_robot_node_path, curr_robots_pos, int(rp), int(n_a), int(l), int(d),
                                        int(failed_robot_id))
    print("New robot paths:", new_robot_paths)
    # visualize the new paths and save the graph to the cache
    visualize_recalculated_paths(new_robot_paths, int(k), int(n_a), saveGraphPath(job_id, 'recalculated_paths'))

    result_data = {'job_id': job_id, 'params': {'k': k, 'q_k': q_k, 'n_a': n_a, 'rp': rp, 'l': l, 'd': d, 'mode': 'h'},
                   'robot_node_path': new_robot_paths,
                   'robot_world_path': convertToWorldPath(int(n_a), new_robot_paths)}
    saveResultsToCache(job_id, result_data, 'recalculated_paths.json')
    return result_data  # Return the content of the JSON file


def recalcRobotPaths(previous_node_path, current_robot_positions, rp, n_a, failed_robot_id):
    """
    This function takes in the previous_node_path and the current_robot_positions and recalculates the paths based on the new positions.
    The robots start where they currently are. The failed robot starts back at the depot. All the robots recalculate their paths based on the new positions
    and the failed robot's new position. They need even frequency coverage to match the redundancy parameter.
    """
    new_node_paths = []

    # Determine the depot node
    depot_node = n_a ** 2  # The depot node is the last node in the grid

    # Recalculate paths for all robots
    for robot_id, path in enumerate(previous_node_path):
        if robot_id == failed_robot_id - 1:
            # Failed robot starts from the depot and continues the rest of its path
            current_position = current_robot_positions[robot_id]
            index = getIndexOf(path, current_position)
            new_path = [depot_node] + path[index:] if index != -1 else [depot_node]
        else:
            # Other robots continue from where they left off or start from the beginning
            current_position = current_robot_positions[robot_id]
            index = getIndexOf(path, current_position)
            new_path = path[index:] if index != -1 else [depot_node]
        new_node_paths.append(new_path)

    # Ensure all paths have a length equal to the redundancy parameter
    for i in range(len(new_node_paths)):
        while len(new_node_paths[i]) < rp:
            new_node_paths[i].append(depot_node)  # Repeat the depot node if needed
    print(new_node_paths)
    return new_node_paths


def recalcRobotPaths2(previous_node_path, current_robot_positions, rp, n_a, l, d, failed_robot_id):
    """
    This function recalculates the paths based on the current positions and where the failed robot starts back at the origin.
    :param previous_node_path:
    :param current_robot_positions:
    :param rp:
    :param n_a:
    :param l:
    :param d:
    :param failed_robot_id:
    :return: The recalculated node and world paths
    """
    k = len(current_robot_positions)  # number of robots
    robot_fuel = [l for ki in range(k)]  # fuel capacity of each robot
    robot_paths = previous_node_path  # previous robot paths

    # initialize all nodes
    initAllNodes(n_a)

    new_robot_paths = generate_robot_paths_redundancy(n_a, k, rp, robot_paths, robot_fuel, l)

    # visualize_paths_brute_force(k, n_a, new_robot_paths)

    print("Heuristic recalculation completed...returning paths to server endpoint /solve")
    worldPath = convertToWorldPath(n_a, new_robot_paths)
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


# recalcRobotPaths(
#     [[16, 17, 10, 14, 9], [16, 17, 0, 1, 3, 2], [16, 17, 4, 8, 12, 13, 5], [16, 17, 7], [16, 17, 15], [16, 17, 6, 11]],
#     [10, 0, 12, 7, 15, 11], 1, 1)
# Example new robot paths if the robots just need to finish where they left off
# [[16, 14, 9], [0, 1, 3, 2], [12, 13, 5], [7], [15], [11]]


def visualize_recalculated_paths(paths, robots, targets, save_path=None):
    k = robots
    # Chose the number of targets in an axis
    n_a = int(targets)

    # Create a uniform (n*n, 2) numpy target grid for MAXIMUM SPEED
    targets = np.mgrid[-1:1:n_a * 1j, -1.:1:n_a * 1j]
    targets = targets.reshape(targets.shape + (1,))
    targets = np.concatenate((targets[0], targets[1]), axis=2)
    targets = targets.reshape((n_a * n_a, 2))
    print(f"{targets.shape=}")
    depots = np.array([
        [-1., -1.],
    ])

    depots = np.concatenate((depots, depots))
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
