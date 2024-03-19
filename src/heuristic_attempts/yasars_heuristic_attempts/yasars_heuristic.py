from matplotlib import pyplot as plt
from scipy.spatial import distance
import numpy as np
from src.heuristic_attempts.yasars_heuristic_attempts.utils.tsp_solver import k_opt
from src.heuristic_attempts.yasars_heuristic_attempts.utils.visualize import visualize_paths, visualize_subtours, visualize_visitation_frequency
from concurrent.futures import ProcessPoolExecutor
import time
import os


def yasars_heuristic(num_of_robots: int,
                     nodes_to_robot_ratio: int,
                     square_side_dist: float,
                     fuel_capacity_ratio: float,
                     failure_rate: int,
                     visualization_path: str = None):
    # Chose number of robots
    k = num_of_robots
    # Chose the number of targets in an axis
    n = k * nodes_to_robot_ratio
    # Chose the length of distance of each side of the square arena
    d = square_side_dist
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    MDBF = 100.0  # Mean Distance Between Failures
    alpha = 0.00001*failure_rate
    rpp = alpha * MDBF  # redundancy parameter percentage
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    rp = np.ceil(k * rpp) + 1
    # Fuel Capacity Parameters
    max_fuel_cost_to_node = d * np.sqrt(2)  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L_min = max_fuel_cost_to_node * 2  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L = L_min * fuel_capacity_ratio  # Fuel capacity (1 unit of fuel = 1 unit of distance)
    print(f"{L_min=} {L=}")
    print(f"{k=} {n=} {d=} {rp=}")

    # 1. Create map and get node indices
    nodes, node_indices, target_indices, depot_indices = construct_map(n, d)

    # 2. Calculate cost between each node
    cost = distance.cdist(nodes, nodes, 'euclidean')

    # 3. Solve for given parameters
    # Step 1: sort the node indices for creating subtours
    start = time.time()
    heading_values = []
    for n_i in target_indices:
        v1 = nodes[n_i]
        v2 = nodes[depot_indices[0]]
        heading = np.arctan2(*(v1 - v2)) % (2 * np.pi)
        heading_values.append(heading)
    heading_values = np.array(heading_values)
    # print(f"{heading_values.shape=}")
    nodes_costs = cost[:depot_indices[0], depot_indices[0]]
    # print(f"{nodes_costs.shape=}")
    sorted_indices = np.lexsort((nodes_costs, heading_values))
    # for i, ti in enumerate(sorted_indices):
    #     print(f"{i=} {ti=} {heading_values[ti]=} {nodes_costs[ti]=}")
    print(f"Step 1 took {time.time() - start} seconds.")

    # Step 2: Divide nodes to subtours s.t. cost <= L
    start = time.time()
    # https://takeuforward.org/arrays/split-array-largest-sum/
    def countSubtourPartitions(maxSum, maxp):
        # n = len(a)  # size of array
        partitions = 1
        partitions_array = [[depot_indices[0], depot_indices[0]]]
        partitions_cost = []
        max_partitions_cost = 0
        for i, n_i in enumerate(sorted_indices):
            node_addition_index = 1
            node_addition_cost = np.inf
            subarraySum = 0
            for j, n_j in enumerate(partitions_array[partitions - 1][:-1]):
                n_k = partitions_array[partitions - 1][j + 1]
                edge_cost = cost[n_j, n_k]
                subarraySum += edge_cost
                new_edge_cost = cost[n_j, n_i] + cost[n_i, n_k]
                if new_edge_cost - edge_cost < node_addition_cost:
                    node_addition_cost = new_edge_cost - edge_cost
                    node_addition_index = j + 1

            if subarraySum + node_addition_cost >= maxSum:
                # if not, insert element to next subarray
                partitions += 1
                if partitions > maxp:
                    # print(f"\tearly stopping...")
                    return np.inf, None, None, np.inf

                partitions_cost.append(subarraySum)
                max_partitions_cost = max(max_partitions_cost, subarraySum)
                node_addition_index = 1
                partitions_array.append([depot_indices[0], depot_indices[0]])
            partitions_array[partitions - 1].insert(node_addition_index, n_i)
            # print(f"{partitions_array[partitions-1]}")

        if len(partitions_array) != len(partitions_cost):
            subarraySum = 0
            for j, n_j in enumerate(partitions_array[partitions - 1][:-1]):
                n_k = partitions_array[partitions - 1][j + 1]
                subarraySum += cost[n_j, n_k]
            partitions_cost.append(subarraySum)
            max_partitions_cost = max(max_partitions_cost, subarraySum)

        return partitions, partitions_array, partitions_cost, max_partitions_cost

    tsp_subtours, tsp_costs, maxSum = divideArrayByP(k, countSubtourPartitions, low=L_min, high=L)
    print(f"Step 2: Found {len(tsp_subtours)} subtours.")
    # print(f"{L=}")
    # print(f"{len(tsp_subtours)=} {maxSum=}")
    # print(f"{distributed_nodes_indices=}")
    # print(f"{tsp_upper_bound=}")
    # visualize_subtours(tsp_subtours, nodes, node_indices, target_indices, depot_indices, cost, mode="faster")
    print(f"Step 2 took {time.time() - start} seconds.")

    # exit()

    # Step 3: Further optimize the subtours by running tsp on them
    start = time.time()
    tsp_subtours_prev, tsp_costs_prev = tsp_subtours, tsp_costs
    tsp_subtours = []
    tsp_costs = []
    with ProcessPoolExecutor(max_workers=os.cpu_count() - 1) as executor:
        futures = []
        for heur_subtour in tsp_subtours_prev:
            futures.append(executor.submit(k_opt, *[heur_subtour, cost, 2]))

        uncompleted_jobs = list(range(len(futures)))
        while len(uncompleted_jobs) != 0:
            new_uncompleted_jobs = uncompleted_jobs
            for future_i in uncompleted_jobs:
                if futures[future_i].done():
                    best_subtour, c = futures[future_i].result()
                    print(
                        f"\t[subtour {future_i + 1}/{len(futures)}] took {time.time() - start:.3f} seconds and improved {(1 - c / tsp_costs_prev[future_i]) * 100:.3f}%.")
                    tsp_subtours.append(best_subtour)
                    tsp_costs.append(c)
                    new_uncompleted_jobs.remove(future_i)
                else:
                    time.sleep(0.1)
            uncompleted_jobs = new_uncompleted_jobs
    tsp_indices = np.array(tsp_costs).argsort().tolist()

    # visualize_subtours(tsp_subtours, nodes, node_indices, target_indices, depot_indices, cost, mode="faster")
    print(f"Step 3 took {time.time() - start} seconds.")

    # Step 4: Ensure rp
    start = time.time()
    num_of_subtours = len(tsp_subtours)
    rp = max(int(np.ceil(k / num_of_subtours)), rp)
    for i in range(num_of_subtours, num_of_subtours * rp):
        tsp_subtours.append(tsp_subtours[i % num_of_subtours])
        tsp_costs.append(tsp_costs[i % num_of_subtours])
        tsp_indices.append(i)
    print(f"Step 4 took {time.time() - start} seconds.")

    # Step 5: Divide subtours between robots
    start = time.time()

    def countRobotPartitions(maxSum, maxp):
        # n = len(a)  # size of array
        partitions = 1
        subarraySum = 0
        partitions_array = [[]]
        partitions_cost = []
        max_partitions_cost = 0
        for i in tsp_indices:
            if subarraySum + tsp_costs[i] <= maxSum:
                # insert element to current subarray
                subarraySum += tsp_costs[i]
            else:
                # if not, insert element to next subarray
                partitions += 1
                if partitions > maxp:
                    # print(f"\tearly stopping...")
                    return np.inf, None, None, np.inf

                partitions_cost.append(subarraySum)
                max_partitions_cost = max(max_partitions_cost, subarraySum)
                subarraySum = tsp_costs[i]
                partitions_array.append([])
            partitions_array[partitions - 1].append(tsp_subtours[i])

        if len(partitions_array) != len(partitions_cost):
            partitions_cost.append(subarraySum)
            max_partitions_cost = max(max_partitions_cost, subarraySum)

        return partitions, partitions_array, partitions_cost, max_partitions_cost

    # TODO: A better estimate for high could be given to speed up binary search
    opt_node_paths, opt_node_path_costs, maxSum = divideArrayByP(k, countRobotPartitions, low=max(tsp_costs), high=sum(tsp_costs), force_p_equals=True)
    # for i, optimized_node_path in enumerate(opt_node_paths):
    #     print(f"[{i}] {len(optimized_node_path)=} cost=({optimized_node_path_costs[i]})")
    print(f"{sum(opt_node_path_costs)=} {max(opt_node_path_costs)=}")
    opt_world_paths = []
    for ki in range(k):
        robot_world_path = []
        for i, subtour in enumerate(opt_node_paths[ki]):
            robot_world_path.append(nodes[subtour].tolist())
        opt_world_paths.append(robot_world_path)
    print(f"Step 5 took {time.time() - start} seconds.")
    visualize_paths(opt_node_paths, nodes, node_indices, target_indices, depot_indices, cost, mode="faster", visualization_path=visualization_path)

    visualize_visitation_frequency(opt_node_paths, nodes, visualization_path)

    return opt_node_paths, opt_world_paths


def construct_map(n, d):
    # nodes = targets + depots
    # Create a uniform (n*n, 2) numpy target grid for MAXIMUM SPEED
    targets = np.mgrid[-1:1:n * 1j, -1.:1:n * 1j] * (d / 2.)
    targets = targets.reshape(targets.shape + (1,))
    targets = np.concatenate((targets[0], targets[1]), axis=2)
    targets = targets.reshape((n * n, 2))
    target_indices = list(range(len(targets)))
    # print(f"{targets.shape=}")

    # Specify depots
    # One depot node in the corner
    depots = np.array([
        [-1., -1.],
    ]) * (d / 2.)
    # print(f"{depots=}")
    depot_indices = list(range(len(targets), len(targets) + len(depots)))

    nodes = np.concatenate((targets, depots))
    # print(f"{nodes.shape=}")
    node_indices = list(range(len(targets) + len(depots)))

    # Graphical sanity check
    plt.figure()
    plt.scatter(targets[:, 0], targets[:, 1], c='blue', s=10)
    plt.scatter(depots[:, 0], depots[:, 1], c='red', s=50)
    plt.grid()
    plt.show()

    return nodes, node_indices, target_indices, depot_indices


def divideArrayByP(maxp, countf, low, high, force_p_equals=False):
    # condition = True
    maxSum = low
    maxSum_max = high
    maxSum_min = low
    err_thresh = 0.01
    best_p, best_p_a, best_p_c, best_max_p_c = np.inf, None, [np.inf], np.inf
    # print(f"{best_p=}")
    while True:
        # print(f"{maxSum_max=} {delta=} {maxSum=}")
        p, p_a, p_c, max_p_c = countf(maxSum, maxp)

        if p > maxp:
            maxSum_min = maxSum
        elif p <= maxp:
            maxSum_max = min(max_p_c, maxSum_max)
            maxSum = max_p_c

        if (force_p_equals and maxp >= p and max_p_c <= best_max_p_c) or (not force_p_equals and p <= best_p):
            best_p = p
            best_p_a = p_a
            best_p_c = p_c
            best_max_p_c = max_p_c

        # print(f"{p=} {best_p=} {maxSum=} {maxSum_min=} {maxSum_max=} {maxSum_max - maxSum_min=} {max_p_c=} {p_c=}")
        # print(f"{p=} {p_a=} {p_c=}")
        # time.sleep(0.5)

        if best_p <= maxp and maxSum_max - maxSum_min < err_thresh:
            return best_p_a, best_p_c, maxSum

        if p > maxp and maxSum <= high:
            delta = maxSum_max - maxSum
            maxSum = maxSum + delta / 2.
        else:
            delta = maxSum - maxSum_min
            maxSum = maxSum - delta / 2.


if __name__ == "__main__":
    num_of_robots = 30
    num_of_targets_per_side = 20
    dist_per_side = 3.
    redundancy_parameter = 20
    fuel_capacity_ratio = 1.5

    optimized_node_paths, optimized_world_paths = yasars_heuristic(num_of_robots,
                                                                   num_of_targets_per_side,
                                                                   dist_per_side,
                                                                   redundancy_parameter,
                                                                   fuel_capacity_ratio)

    # for ki in range(num_of_robots):
    #     print(f"--- {ki=} ---")
    #     for i in range(len(optimized_node_paths[ki])):
    #         print(f"\t{i=} {optimized_node_paths[ki][i]=}")
    #         print(f"\t\t{i=} {optimized_world_paths[ki][i]=}")
