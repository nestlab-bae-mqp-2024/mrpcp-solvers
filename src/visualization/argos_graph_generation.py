import numpy as np
import matplotlib.pyplot as plt
from discretization import discretize_world_points

positions = open('current_pos.txt', 'r')

world_path = []
for line in positions:
    for xy in line.split(";"):
        if xy.find("\n"):
            val = xy.split(",")
            world_path.append((float(val[0]), float(val[1]), float(val[2])))

def discretize_world_points(all_world_points, ssd, n_a):
    dist_betw_each_node = ssd/(n_a-1)
    one_side = np.arange(-ssd/2, ssd/2 + ssd/(n_a-1), dist_betw_each_node)

    all_nodes = [(x,y) for x in one_side for y in one_side]

    #print(all_nodes)
    discretized_points = []
    last_posn = (0,0,0)

    for position in all_world_points:
        for node in all_nodes:
            #print(node)
            if((abs(position[0] - node[0]) < 0.05) and abs(position[1] - node[1]) < 0.05):
                last_posn = position
                discretized_points.append((node[0], node[1], position[2]*0.1))


    print(discretized_points)
    return discretized_points


def visualize_coverage_stepwise(step_requirement_time, world_paths, ssd, n_a, t=10, dt=0.1):
    #time :: seconds

    print("Visualizing coverage over time")
    timestep = 2
    coverage_figure = plt.figure(figsize=(5, 5))
    coverage_ax = plt.subplot()
    plt.xlabel('time (s)')
    plt.ylabel('% coverage')


    coverage_list = []
    for curr_time in np.arange(0., t + dt, dt):
        covered_nodes = set()

        for path in world_paths:
            if path[2] <= curr_time and path[2] >= curr_time-step_requirement_time:
                covered_nodes.add((path[0], path[1]))


        coverage = min(round(100 * len(covered_nodes) / (n_a * n_a), 2), 100)

        coverage_list.append(coverage)

    r = np.arange(0., t + dt, dt)
    coverage_ax.plot(r, coverage_list)

    average_coverage = sum(coverage_list) / len(coverage_list)
    print(average_coverage)

    plt.show()
    #plt.savefig("percent_coverage_visualization.png")


discretized_paths = discretize_world_points(world_path, 3, 8)
visualize_coverage_stepwise(1, discretized_paths, 3, 8, t = 1008, dt = 0.1)
