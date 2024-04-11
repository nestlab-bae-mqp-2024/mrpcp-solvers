import numpy as np
import itertools

def discretize_world_points(all_world_points, metadata):
    ssd = metadata["ssd"]
    n_a = metadata["n_a"]
    k = metadata["k"]
    dist_betw_each_node = ssd/(n_a-1)
    one_side = np.arange(-ssd/2, ssd/2 + ssd/(n_a-1), dist_betw_each_node)

    all_nodes = [(x,y) for x in one_side for y in one_side]

    discretized_points = [[] for ki in range(0,k)]
    for ki in range(0,k):
        for position in all_world_points[ki]:
            last_posn = (0,0)
            for node in all_nodes:
                if(abs(last_posn[0]-position[0]) > 0.05 and abs(last_posn[1]-position[1]) > 0.05 and (abs(position[0] - node[0]) < 0.05 and abs(position[1] - node[1]) < 0.05)):
                    last_posn = position
                    discretized_points[ki].append((node[0], node[1], position[2]))

    # print(discretized_points)
    return discretized_points
    #print(all_nodes)
