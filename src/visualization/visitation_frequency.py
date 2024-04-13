import numpy as np
from matplotlib import pyplot as plt
from matplotlib import ticker


def visualize_visitation_frequency(all_robot_world_points, metadata=None):
    all_world_points = []
    for robot_world_points in all_robot_world_points:
        # print(f"{robot_world_points=}")
        all_world_points.extend(robot_world_points)
    all_world_points = np.array(all_world_points)
    heatmap, xedges, yedges = np.histogram2d(all_world_points[:, 0], all_world_points[:, 1], bins=min(metadata["n_a"], 10))
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]

    heatmap_normalized = heatmap / heatmap.sum() * 100.

    fig = plt.figure()
    pos = plt.imshow(heatmap_normalized.T, extent=extent, origin='lower', vmin=0, vmax=np.round(10., decimals=2))
    cb = fig.colorbar(pos)
    cb.locator = ticker.MaxNLocator(nbins=10)
    cb.formatter = ticker.FormatStrFormatter("%.1f%%")
    xcenters = (xedges[:-1] + xedges[1:]) / 2
    ycenters = (yedges[:-1] + yedges[1:]) / 2
    for (i, j), label in np.ndenumerate(heatmap_normalized):
        # print(f"{i=} {j=}")
        plt.text(xcenters[i], ycenters[j], f"{label:.1f}%", ha="center", va="center", color="w")

    cb.update_ticks()

    fig.suptitle("Normalized World Coordinate Visitation Frequency (Heatmap)")
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    if "visitation_frequency_graph_path" in metadata:
        fig.savefig(metadata["visitation_frequency_graph_path"])
    else:
        plt.show()

    return metadata

