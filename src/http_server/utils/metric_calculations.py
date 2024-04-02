import math


def calculate_mean_distance_per_path(robot_world_path):
    mean_distances = []
    for robot_path in robot_world_path:
        mean_distance = 0
        num_segments = len(robot_path)
        total_distance = 0
        for segment in robot_path:
            segment_distance = 0
            for i in range(len(segment) - 1):
                node1 = segment[i]
                node2 = segment[i + 1]
                segment_distance += calculate_distance(node1, node2)
            total_distance += segment_distance
        if num_segments > 0:
            mean_distance = total_distance / num_segments
        mean_distances.append(mean_distance)
    return mean_distances


def calculate_distance(node1, node2):
    """
    Function to calculate the Euclidean distance between two points in 2D space.
    """
    return math.sqrt((node2[0] - node1[0])**2 + (node2[1] - node1[1])**2)
