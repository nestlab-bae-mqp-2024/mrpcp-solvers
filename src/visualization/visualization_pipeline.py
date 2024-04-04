from src.visualization.pseudo_simulate import pseudo_simulate
from src.visualization.paths_and_subtours import visualize_paths, visualize_subtours
from src.visualization.visitation_frequency import visualize_visitation_frequency
from src.http_server.utils.visualize import visualize_coverage, visualize_heatmap, visualize_paths_heuristic2
from src.visualization.discretization import discretize_world_points

def run_visualization_pipeline(robot_node_path, robot_world_path, metadata):
    # 1. Visualize the paths assigned to each robot
    # TODO: convert world to node and then use visualize_paths
    if metadata["mode"] == "h2":
        metadata = visualize_paths_heuristic2(robot_node_path, metadata)
    else:
        metadata = visualize_paths(robot_node_path, metadata)

    # 2. Pseudo-Simulate and create the 2d histogram of the normalized visitation frequency
    all_world_points = pseudo_simulate(robot_world_path, t=30, ds=0.1)
    metadata = visualize_visitation_frequency(all_world_points, metadata)

    discretized = discretize_world_points(all_world_points, metadata)
    # 3. percent coverage over time
    metadata = visualize_coverage(20, None, discretized, metadata)

    # 4. node visitation over time
    metadata = visualize_heatmap(20, None, discretized, metadata)

    # TODO: 5. mean time/distance between revisitation heatmap

    return metadata
