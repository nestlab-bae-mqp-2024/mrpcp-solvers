import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors

def visualize_mean_time_between_revisitation(all_robot_world_points, metadata=None):
    """
    Visualize the mean time between revisitation of each node in the world
    Revisitation is defined as any robot visitng a node that has been visited before (time = steps)
    :param all_robot_world_points:
    :param metadata:
    :return:
    """
    print("Visualizing Mean Time Between Revisitation")

    visit_times = {}

    # Iterate over all robot world points to record visit times
    for robot_world_points in all_robot_world_points:
        visited_nodes = set()
        for point in robot_world_points:
            node = tuple(point)
            if node not in visited_nodes:
                # If the node hasn't been visited before, record its visit time
                if node not in visit_times:
                    visit_times[node] = []
                visit_times[node].append(len(visit_times[node]))
            visited_nodes.add(node)

    # Create the heatmap
    if visit_times:
        # Extract nodes and corresponding visit times
        nodes, times = zip(*visit_times.items())
        mean_times = [np.mean(times) for times in times] # Calculate mean time between revisitation for each node
        heatmap, xedges, yedges = np.histogram2d(
            [node[0] for node in nodes], [node[1] for node in nodes], bins=9, weights=mean_times
        )
        extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]  # Define extent for the heatmap
    else:
        print('No revisitation')
        heatmap = np.zeros((9, 9))  # Create an empty heatmap if no revisitation time
        extent = [0, 1, 0, 1]  # Default
        mean_times = []

    # Plotting
    plt.figure()
    fig, ax = plt.subplots()
    cmap = cm.viridis
    norm = colors.Normalize(vmin=np.min(mean_times), vmax=np.max(mean_times))  # Normalize colormap
    im = ax.imshow(heatmap.T, extent=extent, origin='lower', cmap=cmap, norm=norm)
    cbar = fig.colorbar(im)
    cbar.set_label('Mean Time Between Revisitation (steps)')
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Mean Time Between Revisitation (Heatmap)')

    # Save or display the plot based on metadata
    if metadata and "mean_time_between_revisitation" in metadata:
        plt.savefig(metadata["mean_time_between_revisitation"])
    else:
        plt.show()

    return metadata
