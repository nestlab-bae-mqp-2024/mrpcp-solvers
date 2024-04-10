from src.visualization.pseudo_simulate import pseudo_simulate
from src.visualization.paths_and_subtours import visualize_paths
from src.http_server.utils.visualize import visualize_paths_heuristic2, \
    visualize_coverage_stepwise
from src.visualization.discretization import discretize_world_points


def run_visualization_pipeline(robot_node_path, robot_world_path, metadata):
    # 1. Visualize the paths assigned to each robot
    # TODO: convert world to node and then use visualize_paths
    print("Running visualization pipeline...")
    print("Robot node path: ", robot_node_path)
    print("Robot world path: ", robot_world_path)
    if metadata["mode"] == "h2":
        metadata = visualize_paths_heuristic2(robot_node_path, metadata)
        # print("Converted to node path: ", convertToNodePaths(robot_world_path, metadata['ssd'], metadata['n_a']))
        # node_ids = nodePathToIds(convertToNodePaths(robot_world_path, metadata['ssd'], metadata['n_a']), metadata['n_a'])
        # print("Node IDs: ", node_ids)
        # metadata = visualize_paths(node_ids, metadata)
    else:
        metadata = visualize_paths(robot_node_path, metadata)

    # 2. Pseudo-Simulate and create the 2d histogram of the normalized visitation frequency
    all_world_points = pseudo_simulate(robot_world_path, metadata)

    # metadata = visualize_visitation_frequency(all_world_points, metadata)

    discretized = discretize_world_points(all_world_points, metadata)

    # print(discretized)
    # 3. percent coverage over time
    #metadata = visualize_coverage(20, None, discretized, metadata)
    metadata = visualize_coverage_stepwise(discretized, metadata)

    # 4. node visitation over time
    # metadata = visualize_node_visitations(20, None, discretized, metadata)

    # 5. mean time/distance between revisitation heatmap
    #metadata = visualize_mean_time_between_revisitation(all_world_points, metadata)
    print("Visualizations complete! Returning metadata...")
    return metadata
