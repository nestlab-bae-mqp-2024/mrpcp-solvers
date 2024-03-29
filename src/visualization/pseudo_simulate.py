import numpy as np


def get_next_node(robot_world_path, si, pi):
    if pi < len(robot_world_path[si]) - 1:
        nsi = si
        npi = pi + 1
    elif si < len(robot_world_path) - 1:
        nsi = si + 1
        npi = 0
    else:
        nsi = 0
        npi = 0

    return nsi, npi


def pseudo_simulate(robot_world_paths, t=10, ds=0.1):
    all_robot_world_points = []
    for ki, robot_world_path in enumerate(robot_world_paths):
        robot_world_points = []

        si = 0
        pi = 0
        robot_world_point = np.array(robot_world_path[si][pi])
        # print(f"{robot_world_point=}")
        robot_world_points.append(robot_world_point.copy())
        for dist in np.arange(0., t + ds, ds):
            dist_travelled = 0
            while dist_travelled < ds:
                nsi, npi = get_next_node(robot_world_path, si, pi)
                new_robot_world_point = np.array(robot_world_path[nsi][npi])
                delta_new_goal = new_robot_world_point - robot_world_point
                dist_to_new_goal = (delta_new_goal[0] ** 2 + delta_new_goal[1] ** 2) ** 0.5
                if dist_to_new_goal - dist_travelled >= ds:
                    robot_world_point += delta_new_goal / dist_to_new_goal * (ds - dist_travelled)
                    dist_travelled = ds
                else:
                    robot_world_point = new_robot_world_point
                    dist_travelled += dist_to_new_goal
                    si = nsi
                    pi = npi

            # print(f"dist={dist:.2f} {robot_world_point} (from Subtour {si} Point {pi}: {robot_world_path[si][pi]})")
            robot_world_points.append(robot_world_point.copy())
        all_robot_world_points.append(robot_world_points)

    return all_robot_world_points
