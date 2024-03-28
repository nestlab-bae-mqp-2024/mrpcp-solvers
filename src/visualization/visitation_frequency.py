import numpy as np
from matplotlib import pyplot as plt
from matplotlib import ticker


def visualize_visitation_frequency(all_robot_world_points, visualization_path=None):
    all_world_points = []
    for robot_world_points in all_robot_world_points:
        print(f"{robot_world_points=}")
        all_world_points.extend(robot_world_points)
    all_world_points = np.array(all_world_points)
    heatmap, xedges, yedges = np.histogram2d(all_world_points[:, 0], all_world_points[:, 1], bins=9)
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]

    heatmap_normalized = heatmap / heatmap.sum()

    fig = plt.figure()
    pos = plt.imshow(heatmap_normalized.T, extent=extent, origin='lower', vmin=0, vmax=np.round(heatmap_normalized.max(), decimals=2))
    cb = fig.colorbar(pos)
    cb.locator = ticker.MaxNLocator(nbins=10)
    cb.formatter = ticker.FormatStrFormatter("%.3f")
    cb.update_ticks()

    fig.suptitle("Normalized Visitation Frequency (Heatmap)")
    if visualization_path:
        fig.savefig(visualization_path.replace("visualization.png", "visitation_frequency_visualization.png"))
    else:
        plt.show()

