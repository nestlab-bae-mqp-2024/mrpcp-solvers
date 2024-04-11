import numpy as np
from matplotlib import pyplot as plt
from tqdm import tqdm
from src.utils.construct_map import construct_map


def get_next_nodes(edges, node, node_indices):
    next_nodes = []
    for n_i in node_indices:
        if edges[node, n_i]:
            next_nodes.append(n_i)
    return next_nodes


def visualize_paths(paths, metadata):
    print("Visualizing paths for heuristic 1 or MILP")
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

        nodes, _, _, _ = construct_map(metadata["n_a"], metadata["ssd"])

        ax.scatter(nodes[:-1, 0], nodes[:-1, 1], c='blue', s=10)
        ax.scatter(nodes[-1, 0], nodes[-1, 1], c='red', s=100)  # Last node is depot

        total_cost_i = 0
        for i in range(len(paths[ki])):
            for j in range(len(paths[ki][i]) - 1):
                curr_node = paths[ki][i][j]
                next_node = paths[ki][i][j + 1]
                total_cost_i += np.linalg.norm(nodes[curr_node] - nodes[next_node])

                ax.scatter(nodes[curr_node, 0], nodes[curr_node, 1], c="purple", s=8)
                ax.plot([nodes[curr_node, 0], nodes[next_node, 0]], [nodes[curr_node, 1], nodes[next_node, 1]],
                        color="purple", linewidth=1)

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

    fig.suptitle(
        f"Paths for all robots (# of active/available robots={len(active_robots)}/{len(paths)}, sum of costs={total_cost:.3f})")
    if "visualize_paths_graph_path" in metadata:
        plt.savefig(metadata["visualize_paths_graph_path"])
    #plt.show()

    return metadata


def visualize_subtours(subtours, metadata):
    return visualize_paths([[subtour] for subtour in subtours], metadata)
