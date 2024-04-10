import numpy as np
from matplotlib import colors, cm, pyplot as plt, pyplot
import itertools

from src.http_server.utils.conversions import convertToNodePaths


def visualize_paths_edges_brute_force(edges, nodes, node_indices, target_indices, depot_indices, cost, save_path=None):
    # Only plot the paths for the robots that were assigned a path
    active_robots = []
    for ki in range(len(edges)):
        if (cost * edges[ki]).sum() > 0.01:
            active_robots.append(ki)

    subplot_per_hor_axis = int(np.ceil(np.sqrt(len(active_robots))))
    subplot_per_vert_axis = int(np.ceil(len(active_robots) / subplot_per_hor_axis))
    fig, axs = plt.subplots(subplot_per_hor_axis, subplot_per_vert_axis,
                            figsize=(subplot_per_hor_axis * 4, subplot_per_vert_axis * 4))
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.1, top=0.9, right=0.9, left=0.1, wspace=0.3, hspace=0.3)

    hor_i = 0
    vert_i = 0
    for robot_i, ki in enumerate(active_robots):
        if subplot_per_hor_axis == 1 and subplot_per_vert_axis == 1:
            ax = axs
        elif subplot_per_vert_axis == 1:
            ax = axs[hor_i]
        else:
            ax = axs[hor_i][vert_i]
        ax.set_title(f"Robot #{robot_i + 1} (cost={(cost * edges[ki]).sum():.3f})")
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
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()


def get_next_nodes(edges, node, node_indices):
    next_nodes = []
    for n_i in node_indices:
        if edges[node, n_i]:
            next_nodes.append(n_i)
    return next_nodes


def visualize_paths_faster(paths, nodes, node_indices, target_indices, depot_indices, cost, save_path=None):
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
    for robot_i, ki in enumerate(active_robots):
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
            for j in range(len(paths[ki][i]) - 1):
                curr_node = paths[ki][i][j]
                next_node = paths[ki][i][j + 1]
                total_cost_i += cost[curr_node, next_node]

                # print(f"{curr_node=} {next_node=}")

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

    plt.grid()
    fig.suptitle \
        (f"Paths for all robots (# of active/available robots={len(active_robots)}/{len(paths)}, sum of costs={total_cost:.3f})")
    # Save the figure if save_path is provided
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()


def visualize_subtours(subtours, nodes, node_indices, target_indices, depot_indices, cost, mode="brute-force"):
    if mode == "brute-force":
        path_edges = np.zeros((len(subtours), len(node_indices), len(node_indices)), dtype=np.uint8)

        for ki, subtour in enumerate(subtours):
            for curr_node_i in range(len(subtour) - 1):
                next_node_i = curr_node_i + 1
                curr_node = subtour[curr_node_i]
                next_node = subtour[next_node_i]
                path_edges[ki][curr_node][next_node] = 1

        visualize_paths_edges_brute_force(path_edges, nodes, node_indices, target_indices, depot_indices, cost)
    elif mode == "faster":
        visualize_paths_faster([[subtour] for subtour in subtours], nodes, node_indices, target_indices, depot_indices,
                               cost)


def visualize_paths(paths, nodes, node_indices, target_indices, depot_indices, cost, mode="brute-force",
                    save_path=None):
    if mode == "brute-force":
        path_edges = np.zeros((len(paths), len(node_indices), len(node_indices)), dtype=np.uint8)

        for ki in range(len(paths)):
            for path_i, path in enumerate(paths[ki]):
                for curr_node_i in range(len(path) - 1):
                    next_node_i = curr_node_i + 1
                    curr_node = path[curr_node_i]
                    next_node = path[next_node_i]
                    path_edges[ki][curr_node][next_node] = 1
        visualize_paths_edges_brute_force(path_edges, nodes, node_indices, target_indices, depot_indices, cost,
                                          save_path)
    elif mode == "faster":
        visualize_paths_faster(paths, nodes, node_indices, target_indices, depot_indices, cost, save_path)


# visualizes percent coverage over time
def visualize_coverage(step_requirement, robot_paths, world_paths, metadata):
    print("Visualizing coverage over time")
    timestep = 2
    ssd = metadata["ssd"]
    n_a = metadata["n_a"]
    coverage_figure = plt.figure(figsize=(5, 5))
    coverage_ax = plt.subplot()
    plt.xlabel('number of steps')
    plt.ylabel('% coverage')

    num_of_robots = 0
    # code to convert world_paths to node_path format if necessary
    if world_paths is not None:
        num_of_robots = len(world_paths)
        updated_paths = convertToNodePaths(world_paths, ssd, n_a)

    elif robot_paths is not None:  # if given robot_paths, assuming already in node_path format
        num_of_robots = len(robot_paths)
        updated_paths = robot_paths

    number_of_steps = max(len(updated_paths[ki]) for ki in range(num_of_robots))

    comparison = 100 * step_requirement / (n_a * n_a / num_of_robots)

    coverage_list = []

    binary_heatmap = np.zeros((n_a, n_a))
    path_counter = [0 for ki in range(num_of_robots)]

    for step_counter in range(0, number_of_steps):
        for a in range(0, n_a):
            for b in range(0, n_a):
                binary_heatmap[a][b] = max(0, binary_heatmap[a][b] - 1)

        for ki in range(num_of_robots):
            if path_counter[ki] >= len(updated_paths[ki]) - 1:
                path_counter[ki] = 0
            else:
                path_counter[ki] += 1

            (x, y) = updated_paths[ki][path_counter[ki]]
            binary_heatmap[x][y] = step_requirement

        coverage = round(100 * len(binary_heatmap[binary_heatmap > 0]) / (n_a * n_a), 2)
        coverage_list.append(coverage)

    r = [*range(0, len(coverage_list))]
    coverage_ax.plot(r, coverage_list[0:number_of_steps])
    #coverage_ax.plot(r, [comparison] * number_of_steps, '--')

    average_coverage = sum(coverage_list[timestep:]) / len(coverage_list[timestep:])
    # print(average_coverage)
    if "average_coverage" in metadata:
        metadata["average_coverage"] = average_coverage

    plt.suptitle("Percent Coverage Over Time")
    if "percent_coverage_visualization" in metadata:
        plt.savefig(metadata["percent_coverage_visualization"])
    else:
        plt.show()
    # print(metadata)
    return metadata


# visualizes percent coverage over time
def visualize_coverage_stepwise(step_requirement_time, world_paths, metadata, t=10, dt=0.1):
    #time :: seconds

    print("Visualizing coverage over time")
    timestep = 2
    ssd = metadata["ssd"]
    n_a = metadata["n_a"]
    coverage_figure = plt.figure(figsize=(5, 5))
    coverage_ax = plt.subplot()
    plt.xlabel('time (s)')
    plt.ylabel('% coverage')


    coverage_list = []
    for curr_time in np.arange(0., t + dt, dt):
        number_of_covered_nodes = 0

        for robot_path in world_paths:
            for path in robot_path:
                if path[2] <= curr_time and path[2] >= curr_time-step_requirement_time:
                    number_of_covered_nodes += 1

        coverage = round(100 * number_of_covered_nodes / (n_a * n_a), 2)

        coverage_list.append(coverage)

    r = np.arange(0., t + dt, dt)
    coverage_ax.plot(r, coverage_list)

    #coverage_ax.plot(r, [comparison] * number_of_steps, '--')

    average_coverage = sum(coverage_list) / len(coverage_list)
    print(average_coverage)
    if "average_coverage" in metadata:
        metadata["average_coverage"] = average_coverage

    plt.suptitle("Percent Coverage Over Time")
    if "percent_coverage_visualization" in metadata:
        plt.savefig(metadata["percent_coverage_visualization"])
    else:
        plt.show()
    # print(metadata)
    return metadata


def visualize_node_visitations(step_requirement, robot_paths, world_paths,
                               metadata):
    print("Visualizing heatmap for coverage over time")
    ssd = metadata["ssd"]
    n_a = metadata["n_a"]
    dist_betw_each_node = ssd / (n_a - 1)

    num_of_robots = 0
    # code to convert world_paths to node_path format if necessary
    if world_paths is not None:

        num_of_robots = len(world_paths)
        no_time_paths = [[] for ki in range(num_of_robots)]

        for ki in range(num_of_robots):
            for z in world_paths[ki]:
                print(z)
                no_time_paths[ki].append([z[0], z[1]])

        print(no_time_paths)
        updated_paths = convertToNodePaths(no_time_paths, ssd, n_a)

    elif robot_paths is not None:  # if given robot_paths, assuming already in node_path format
        num_of_robots = len(robot_paths)
        updated_paths = robot_paths

    number_of_steps = max(len(updated_paths[ki]) for ki in range(num_of_robots))

    heatmap = np.zeros((n_a, n_a))
    path_counter = [0 for ki in range(num_of_robots)]
    for step_counter in range(0, number_of_steps):
        for ki in range(num_of_robots):
            if path_counter[ki] >= len(updated_paths[ki]) - 1:
                path_counter[ki] = 0
            else:
                path_counter[ki] += 1

            (x, y) = updated_paths[ki][path_counter[ki]]
            heatmap[x][y] = heatmap[x][y] + 1

    # Create a new figure
    plt.figure()

    fig, ax = plt.subplots()
    im = ax.imshow(heatmap[:, :], norm=colors.Normalize(0, heatmap[0][0]), origin='lower')

    bar = fig.colorbar(cm.ScalarMappable(norm=colors.Normalize(0, heatmap[0][0])), ax=ax)

    bar.set_ticks([0, heatmap[0][0]])

    xtick = [str(i) for i in np.arange(-ssd/2, ssd/2+0.5, 0.5)]
    ax.set_xticks((n_a/(2*ssd))*np.arange(len(xtick))-0.5, labels = xtick)
    ax.set_yticks((n_a/(2*ssd))*np.arange(len(xtick))-0.5, labels = xtick)

    for (j, i), label in np.ndenumerate(heatmap):
        text = ax.text(i, j, int(label),
                       ha="center", va="center", color="w")

    fig.suptitle(f"Number of Visitations per Node: Simulation run for {number_of_steps} steps")
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    if "node_visitation_heatmap" in metadata:
        plt.savefig(metadata["node_visitation_heatmap"])
    else:
        plt.show()

    return metadata


def visualize_paths_heuristic2(robot_paths, metadata):
    print("Visualizing paths for heuristic 2")
    n_a = metadata["n_a"]
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

        ax.set_title(f"Robot #{ki+1}")
        ax.grid()
        ax.legend()

    # if visualization_path:
    #     pyplot.savefig(visualization_path.replace("visualization.png", "h2_visualization.png"))
    # else:
    #     pyplot.show()
    fig.suptitle(f"Paths for all robots (# of robots={len(robot_paths)}")
    if "visualize_paths_graph_path" in metadata:
        plt.savefig(metadata["visualize_paths_graph_path"])
    plt.show()
    return metadata


def visualize_individual_paths(paths, nodes, targets, depots, b_k, costs, metadata):
    """
    Visualization used in MRPCP
    """
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
        ax.plot([nodes[path[-1], 0], nodes[b_k[0], 0]],
                [nodes[path[-1], 1], nodes[b_k[0], 1]],
                color="purple", linewidth=1, linestyle="--", label='Return to Depot')

        # Plot the starting depot
        ax.text(nodes[b_k[0], 0], nodes[b_k[0], 1], str(b_k[0]), fontsize=8, ha='center', va='center')

        # Set title with cost
        ax.set_title(f"Robot #{index + 1} (Cost: {costs[index]:.2f})")
        ax.grid()
        ax.legend()

    # Hide any unused subplots
    for i in range(index + 1, num_rows * 2):
        fig.delaxes(axs[i])

    # plt.tight_layout()
    fig.suptitle(f"Paths for all robots (sum of costs={sum(costs):.3f})")

    if "visualize_paths_graph_path" in metadata:
        plt.savefig(metadata["visualize_paths_graph_path"])
    plt.show()
    return metadata
