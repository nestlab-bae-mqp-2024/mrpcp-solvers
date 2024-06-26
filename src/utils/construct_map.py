import numpy as np
import matplotlib.pyplot as plt


def construct_map(n, ssd, d, milp=False, gfx_sanity_check=False):
    # nodes = targets + depots
    # Create a uniform (n*n, 2) numpy target grid for MAXIMUM SPEED
    targets = np.mgrid[-1:1:n * 1j, -1.:1:n * 1j] * (d / 2.)
    targets = targets.reshape(targets.shape + (1,))
    targets = np.concatenate((targets[0], targets[1]), axis=2)
    targets = targets.reshape((n * n, 2))
    target_indices = list(range(len(targets)))
    # print(f"{targets.shape=}")

    # Specify depots
    # One depot node in the corner
    if milp:
        depots = np.array([
            [-1., -1.],
            [-1., -1.],
        ]) * (d / 2.)
    else:
        depots = np.array([
            [-1., -1.],
        ]) * (d / 2.)
    # print(f"{depots=}")
    depot_indices = list(range(len(targets), len(targets) + len(depots)))

    nodes = np.concatenate((targets, depots))
    # print(f"{nodes.shape=}")
    node_indices = list(range(len(targets) + len(depots)))

    # Graphical sanity check
    if gfx_sanity_check:
        plt.figure()
        plt.scatter(targets[:, 0], targets[:, 1], c='blue', s=10)
        plt.scatter(depots[:, 0], depots[:, 1], c='red', s=50)
        plt.xlim(-ssd / 2., ssd / 2.)
        plt.ylim(-ssd / 2., ssd / 2.)
        plt.grid()
        plt.show()

    return nodes, node_indices, target_indices, depot_indices
