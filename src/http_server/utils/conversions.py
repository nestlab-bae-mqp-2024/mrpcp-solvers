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
        robot_world_path.append(world_path)
    for i in range(pow(n_a, 2) + 1):
        print(f"{i=}, {nodes[i]}")
    return robot_world_path


def worldToNodePath(world_path):
    """
    Use this function to convert the world path to a node path
    Used for recalculation and for heuristic 2 to convert back to MILP/H1 node path format
    :return:
    """
    node_path = []
    for i in range(len(world_path)):
        if i == 0:
            node_path.append([world_path[i]])
        else:
            node_path.append([world_path[i], world_path[i - 1]])
    return node_path
