import datetime
import numpy as np
from matplotlib import pyplot as plt
import itertools
from tqdm import tqdm
import math


def visualize_paths_edges_brute_force(edges, nodes, node_indices, target_indices, depot_indices, cost, visualization_path=None):
    # Only plot the paths for the robots that were assigned a path
    active_robots = []
    for ki in range(len(edges)):
        if (cost * edges[ki]).sum() > 0.01:
            active_robots.append(ki)

    subplot_per_hor_axis = int(np.ceil(np.sqrt(len(active_robots))))
    subplot_per_vert_axis = int(np.ceil(len(active_robots) / subplot_per_hor_axis))
    fig, axs = plt.subplots(subplot_per_hor_axis, subplot_per_vert_axis, figsize=(subplot_per_hor_axis * 4, subplot_per_vert_axis * 4))
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.1, top=0.9, right=0.9, left=0.1, wspace=0.3, hspace=0.3)

    hor_i = 0
    vert_i = 0
    for robot_i, ki in enumerate(active_robots):
        # print(f"Robot #{ki}\n-------")
        # print(f"Staring position: {B_k[ki]} -> {[nodes[B_k[ki, 0], B_k[ki, 1], 0], nodes[B_k[ki, 0], B_k[ki, 1], 1]]}")
        if subplot_per_hor_axis == 1 and subplot_per_vert_axis == 1:
            ax = axs
        elif subplot_per_vert_axis == 1:
            ax = axs[hor_i]
        else:
            ax = axs[hor_i][vert_i]
        ax.set_title(f"Robot #{robot_i +1} (cost={(cost * edges[ki]).sum():.3f})")
        ax.scatter(nodes[target_indices, 0], nodes[target_indices, 1], c='blue', s=10)
        ax.scatter(nodes[depot_indices, 0], nodes[depot_indices, 1], c='red', s=50)
        ax.scatter(nodes[depot_indices[0], 0], nodes[depot_indices[0], 1], c='red', s=100)

        for i, j in itertools.product(node_indices, node_indices):
            if edges[ki][i][j] > 0.5:  # In case there is any floating math errors
                # print(f"Connection from {[i1,j1]} to {[i2,j2]}")
                ax.scatter(nodes[i, 0], nodes[i, 1], c="purple", s=8)
                ax.scatter(nodes[j, 0], nodes[j, 1], c="purple", s=8)
                ax.plot([nodes[i, 0], nodes[j, 0]], [nodes[i, 1], nodes[j, 1]], color="purple", linewidth=1)

        vert_i += 1
        if vert_i >= subplot_per_vert_axis:
            vert_i = 0
            hor_i += 1
        ax.grid()

    for h in range(subplot_per_hor_axis):
        for v in range(subplot_per_vert_axis):
            if subplot_per_hor_axis == 1 and subplot_per_vert_axis == 1:
                ax = axs
            elif subplot_per_vert_axis == 1:
                ax = axs[h]
            else:
                ax = axs[h][v]
            ax.set_box_aspect(1)

    fig.suptitle \
        (f"Paths for all robots (# of active/available robots={len(active_robots)}/{len(edges)}, sum of costs={(cost * edges).sum():.3f})")
    if visualization_path:
        plt.savefig(visualization_path.replace("visualization.png", "visitation_frequency_visualization.png"))
    else:
        plt.show()
    plt.show()


def get_next_nodes(edges, node, node_indices):
    next_nodes = []
    for n_i in node_indices:
        if edges[node, n_i]:
            next_nodes.append(n_i)
    return next_nodes


def visualize_paths_faster(paths, nodes, node_indices, target_indices, depot_indices, cost, visualization_path=None):
    # Only plot the paths for the robots that were assigned a path
    active_robots = []
    for ki in range(len(paths)):
        if len(paths[ki]) > 0:
            active_robots.append(ki)

    subplot_per_hor_axis = int(np.ceil(np.sqrt(len(active_robots))))
    subplot_per_vert_axis = int(np.ceil(len(active_robots) / subplot_per_hor_axis))
    fig, axs = plt.subplots(subplot_per_hor_axis, subplot_per_vert_axis,
                            figsize=(subplot_per_hor_axis * 4, subplot_per_vert_axis * 4))
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.1, top=0.9, right=0.9, left=0.1, wspace=0.3, hspace=0.3)

    total_cost = 0
    hor_i = 0
    vert_i = 0
    for robot_i, ki in tqdm(enumerate(active_robots)):
        # print(f"Robot #{ki}\n-------")
        # print(f"Staring position: {B_k[ki]} -> {[nodes[B_k[ki, 0], B_k[ki, 1], 0], nodes[B_k[ki, 0], B_k[ki, 1], 1]]}")
        if subplot_per_hor_axis == 1 and subplot_per_vert_axis == 1:
            ax = axs
        elif subplot_per_vert_axis == 1:
            ax = axs[hor_i]
        else:
            ax = axs[hor_i][vert_i]
        ax.scatter(nodes[target_indices, 0], nodes[target_indices, 1], c='blue', s=10)
        ax.scatter(nodes[depot_indices, 0], nodes[depot_indices, 1], c='red', s=50)
        ax.scatter(nodes[depot_indices[0], 0], nodes[depot_indices[0], 1], c='red', s=100)

        total_cost_i = 0
        for i in range(len(paths[ki])):
            for j in range(len(paths[ki][i])-1):
                curr_node = paths[ki][i][j]
                next_node = paths[ki][i][j+1]
                total_cost_i += cost[curr_node, next_node]

                # print(f"{curr_node=} {next_node=}")

                ax.scatter(nodes[curr_node, 0], nodes[curr_node, 1], c="purple", s=8)
                ax.plot([nodes[curr_node, 0], nodes[next_node, 0]], [nodes[curr_node, 1], nodes[next_node, 1]], color="purple", linewidth=1)

        ax.set_title(f"Robot #{robot_i + 1} (cost={total_cost_i:.3f})")
        total_cost += total_cost_i

        vert_i += 1
        if vert_i >= subplot_per_vert_axis:
            vert_i = 0
            hor_i += 1
        ax.grid()

    for h in range(subplot_per_hor_axis):
        for v in range(subplot_per_vert_axis):
            if subplot_per_hor_axis == 1 and subplot_per_vert_axis == 1:
                ax = axs
            elif subplot_per_vert_axis == 1:
                ax = axs[h]
            else:
                ax = axs[h][v]
            ax.set_box_aspect(1)

    plt.grid()
    fig.suptitle \
        (f"Paths for all robots (# of active/available robots={len(active_robots)}/{len(paths)}, sum of costs={total_cost:.3f})")
    if visualization_path:
        plt.savefig(visualization_path.replace("visualization.png", "all_robots_visualization.png"))
    else:
        plt.show()
    plt.show()


def visualize_subtours(subtours, nodes, node_indices, target_indices, depot_indices, cost, mode="brute-force", visualization_path=None):
    if mode == "brute-force":
        path_edges = np.zeros((len(subtours), len(node_indices), len(node_indices)), dtype=np.uint8)

        for ki, subtour in enumerate(subtours):
            for curr_node_i in range(len(subtour) - 1):
                next_node_i = curr_node_i + 1
                curr_node = subtour[curr_node_i]
                next_node = subtour[next_node_i]
                path_edges[ki][curr_node][next_node] = 1

        visualize_paths_edges_brute_force(path_edges, nodes, node_indices, target_indices, depot_indices, cost, visualization_path)
    elif mode == "faster":
        visualize_paths_faster([[subtour] for subtour in subtours], nodes, node_indices, target_indices, depot_indices, cost, visualization_path)


def visualize_paths(paths, nodes, node_indices, target_indices, depot_indices, cost, mode="brute-force", visualization_path=None):
    if mode == "brute-force":
        path_edges = np.zeros((len(paths), len(node_indices), len(node_indices)), dtype=np.uint8)

        for ki in range(len(paths)):
            for path_i, path in enumerate(paths[ki]):
                for curr_node_i in range(len(path) - 1):
                    next_node_i = curr_node_i + 1
                    curr_node = path[curr_node_i]
                    next_node = path[next_node_i]
                    path_edges[ki][curr_node][next_node] = 1
        visualize_paths_edges_brute_force(path_edges, nodes, node_indices, target_indices, depot_indices, cost, visualization_path)
    elif mode == "faster":
        visualize_paths_faster(paths, nodes, node_indices, target_indices, depot_indices, cost, visualization_path)


def visualize_visitation_frequency(paths, nodes, visualization_path=None):
    all_nodes = []

    for ki in range(len(paths)):
        for path in paths[ki]:
            for curr_node_i in range(1, len(path) - 2):
                next_node_i = curr_node_i + 1
                curr_node_pos = nodes[path[curr_node_i]]
                next_node_pos = nodes[path[next_node_i]]

                step_size = 0.1
                dist = ((curr_node_pos[0]-next_node_pos[0]) ** 2 + (curr_node_pos[1]-next_node_pos[1]) ** 2) ** 0.5
                num_of_strides = np.ceil(dist / step_size).astype(int)
                # print(f"{curr_node_pos[0]=} {next_node_pos[0]=} {num_of_strides=}")

                for intermediate_node_pos in np.concatenate((np.linspace(curr_node_pos[0], next_node_pos[0], num_of_strides+1).reshape(-1, 1), np.linspace(curr_node_pos[1], next_node_pos[1], num_of_strides+1).reshape(-1, 1)), axis=1):
                    # Skip for locations close to depot for better visualization.
                    if np.isclose(intermediate_node_pos, nodes[path[0]], atol=step_size).all():
                        continue
                    all_nodes.append(intermediate_node_pos)

    all_nodes = np.array(all_nodes)
    # print(f"{all_nodes=}")
    # node_visitation_freq = 1. / nodes_counter
    # print(f"{nodes_counter=}")

    heatmap, xedges, yedges = np.histogram2d(all_nodes[:, 0], all_nodes[:, 1], bins=30)
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]

    plt.imshow(heatmap.T, extent=extent, origin='lower')
    if visualization_path:
        plt.savefig(visualization_path.replace("visualization.png", "visitation_frequency_visualization.png"))
    else:
        plt.show()



