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


def pseudo_simulate(robot_world_paths, metadata):
    # print("Pseudo-simulating robots...")

    v = metadata["v"]
    dt = metadata["dt"]
    t = metadata["t"]
    ds = v * dt
    all_robot_world_points = []
    for ki, robot_world_path in enumerate(robot_world_paths):
        robot_world_points = []

        si = 0
        pi = 0
        robot_world_point = np.array(robot_world_path[si][pi])
        # print(f"{robot_world_point=}")
        datum = robot_world_point.copy().tolist()
        datum.append(0.)
        robot_world_points.append(datum)
        for curr_time in np.arange(0., t + dt, dt):
            dist_travelled = 0

            while len(robot_world_paths[ki]) > 1 and dist_travelled < ds:
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
            datum = robot_world_point.copy().tolist()
            datum.append(curr_time)
            robot_world_points.append(datum)
        all_robot_world_points.append(np.array(robot_world_points))

    return np.array(all_robot_world_points)
