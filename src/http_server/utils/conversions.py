import numpy as np
from matplotlib import pyplot as plt


def convertToWorldPath(n_a, d, robot_node_path):
    """
    This function takes in the node paths and should return X and Y coordinates for the robots to follow in ArgOS
    Example:
        robot_node_path = [
            [16,15]
        ]
        convertToWorldPath(4, robot_node_path)
        # Returns:
        [
            [[-1, -1], [-0.5, -0.5], [-1, -1]]
        ]
    """
    print("Converting to world path")
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

    # Graphical sanity check
    plt.figure()
    plt.scatter(targets[:, 0], targets[:, 1], c='blue', s=10)
    plt.scatter(depots[:, 0], depots[:, 1], c='red', s=50)
    plt.grid()

    # Label nodes with node IDs and their positions
    for i, node in enumerate(nodes):
        plt.text(node[0], node[1], f'{i}', fontsize=8, ha='center')

    plt.show()

    robot_world_path = []
    for path in robot_node_path:
        world_path = []
        for i, node in enumerate(path):
            x, y = nodes[node]
            world_path.append([float(x), float(y)])  # Convert to floats
        world_path.append(world_path[0])  # Return to starting node
        robot_world_path.append([world_path])
    for i in range(pow(n_a, 2) + 1):
        print(f"{i=}, {nodes[i]}")
    return robot_world_path


def worldToNodePath(n_a, d, world_path):
    """
    Use this function to convert the world path to a node path
    Used for recalculation and for heuristic 2 to convert back to MILP/H1 node path format
    :return:
    """
    d_2 = d / 2.  # Distance per side of the square
    dist_betw_each_node = d / (n_a - 1)
    corner = -d_2
    node_path = []
    for point in world_path:
        # Ensure point coordinates are numeric
        if isinstance(point, (int, float)):
            node_path.append(point)  # Add the point directly if it's a single coordinate
        elif isinstance(point, (list, tuple)) and len(point) == 2:
            # Calculate the node indices based on the point coordinates and node spacing
            node_x = int((point[0] - corner) / dist_betw_each_node)
            node_y = int((point[1] - corner) / dist_betw_each_node)
            node_path.append([node_x, node_y])
        else:
            raise ValueError("Invalid point format in world path")
    return node_path


def convertToNodePaths(world_paths, ssd, n_a):
    dist_betw_each_node = ssd / (n_a - 1)
    num_of_robots = len(world_paths)

    updated_paths = [[] for ki in range(num_of_robots)]
    for ki in range(0, num_of_robots):
        path = []

        if len(world_paths[ki][
                   0]) == 2:  # this is just to account for if there are multiple subtours in what it's given, or not.
            for item in world_paths[ki]:
                path.append((round(((ssd) / 2 + item[0]) / (dist_betw_each_node)),
                             round((ssd / 2 + item[1]) / (dist_betw_each_node))))

        else:
            for w_path in world_paths[ki]:
                for item in w_path:
                    path.append((round((ssd / 2 + item[0]) / (dist_betw_each_node)),
                                 round((ssd / 2 + item[1]) / (dist_betw_each_node))))

        updated_paths[ki] = path

    return updated_paths


def nodePathToIds(node_path, n_a):
    """
    Convert node paths from coordinates to IDs, including subtours when the path returns to node 0.
    """
    node_id_mapping = {}  # Dictionary to store the mapping of coordinates to IDs
    next_node_id = 0  # Counter for assigning IDs

    node_path_ids = []  # List to store the node paths with IDs

    for path in node_path:
        sub_path_list = []  # List to store subpaths for each subtour
        path_ids = []  # List to store the IDs for the current path
        zero_count = 0  # Counter for the number of (0, 0) occurrences
        for node_coord in path:
            x, y = node_coord
            # Calculate the ID based on the node's position
            if x / 2 == 1:
                node_id = ((abs(x)+1)*n_a)-(abs(y)+1)
            else:
                node_id = (abs(x)*n_a)+abs(y)
            # Check if the current node coordinate is already assigned an ID
            if node_coord not in node_id_mapping:
                node_id_mapping[node_coord] = node_id
                next_node_id += 1
            # Append the ID of the current node coordinate to the path IDs list
            path_ids.append(node_id_mapping[node_coord])

            # Check if the current node is (0, 0)
            if node_coord == (0, 0):
                zero_count += 1
                if zero_count == 1:  # Open bracket for the first occurrence of (0, 0)
                    sub_path_list.append(path_ids)
                    path_ids = []
                elif zero_count == 3:  # Close bracket after the third occurrence of (0, 0)
                    zero_count = 0
                    sub_path_list.append(path_ids)
                    node_path_ids.append(sub_path_list)
                    sub_path_list = []
                    path_ids = []

        # Append the path IDs list to the node path IDs list if it's not empty
        if path_ids:
            sub_path_list.append(path_ids)

    return node_path_ids




