from src.visualization.pseudo_simulate import pseudo_simulate
from src.visualization.paths_and_subtours import visualize_paths, visualize_subtours
from src.visualization.visitation_frequency import visualize_visitation_frequency
from src.http_server.utils.visualize import visualize_coverage, visualize_heatmap, visualize_paths_heuristic2


def run_visualization_pipeline(robot_node_path, robot_world_path, metadata, visualization_path=None):
    if metadata["mode"] == "h2":
        # 1. Visualize the paths assigned to each robot
        metadata = visualize_paths_heuristic2(robot_node_path, metadata)
        # 2. Pseudo-Simulate and create the 2d histogram of the visitation frequency
        # all_world_points = pseudo_simulate(robot_world_path, t=30, ds=0.1)
        # metadata = visualize_visitation_frequency(all_world_points, metadata)
    else:
        metadata = visualize_paths(robot_node_path, metadata)
        all_world_points = pseudo_simulate(robot_world_path, t=30, ds=0.1)
        metadata = visualize_visitation_frequency(all_world_points, metadata)

    # TODO: 3. % coverage over time
    visualize_coverage(20, 1000, robot_node_path, robot_world_path, metadata)

    visualize_heatmap(20, 1000, robot_node_path, robot_world_path, metadata)

    # TODO: 4. mean time between revisitation heatmap

    return metadata

