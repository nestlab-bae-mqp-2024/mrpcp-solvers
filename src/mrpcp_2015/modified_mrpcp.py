"""
Author: N/A
Date: 2/17/2024

This program defines the methods relating to solving the MRPCP (Multi-Robot Path Coverage Problem) using MILP (Mixed-Integer Linear Programming)
and TSP (Travelling Salesman Problem) using 2-OPT and k-OPT algorithms.
"""
import time

from src.utils.construct_map import construct_map
from typing import Dict
from matplotlib import pyplot as plt
import numpy as np
import itertools
import gurobipy as gp
from gurobipy import GRB
from scipy.spatial import distance
import os
from src.http_server.utils.conversions import convertToWorldPath
from src.http_server.utils.tsp_solver import k_opt
from src.http_server.utils.visualize import visualize_individual_paths
from src.visualization.visualization_pipeline import run_visualization_pipeline
import matplotlib
from src.http_server.json_handlers import saveGraphPath
from src.http_server.json_handlers import *


def solve_milp_with_optimizations(num_of_robots: int,
                                  nodes_per_axis: int,
                                  square_side_dist: float,
                                  fuel_capacity_ratio: float,
                                  rp: int,
                                  metadata: Dict = None):
    """
    This function solves the MRPCP using MILP (Mixed-Integer Linear Programming) and TSP (Travelling Salesman Problem) using 2-OPT and k-OPT algorithms.
    :return: The optimized paths with 2-OPT and the world path
    """
    if metadata is None:
        metadata = {}
    print("Initializing MILP solution...")
    # Chose number of robots
    k = num_of_robots
    # Chose the number of targets in an axis
    n_a = nodes_per_axis
    # Chose the length of distance of each side of the square arena
    d = square_side_dist
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    rp = min(rp, k)
    # Fuel Capacity Parameters
    max_fuel_cost_to_node = d * np.sqrt(2)  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L_min = max_fuel_cost_to_node * 2  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L = L_min * fuel_capacity_ratio  # Fuel capacity (1 unit of fuel = 1 unit of distance)
    M = L + max_fuel_cost_to_node
    # print(f"{L_min=} {L=}")
    # print(f"{k=} {n=} {d=} {rp=}")
    # meta data given params
    metadata["k"] = k
    metadata["n_a"] = nodes_per_axis
    metadata["ssd"] = square_side_dist
    metadata["fcr"] = fuel_capacity_ratio
    metadata["rp"] = rp
    # metadata derived params
    metadata["L_min"] = L
    metadata["mode"] = "m"

    # 1. Create map and get node indices
    nodes, node_indices, target_indices, depot_indices = construct_map(n_a, d, milp=True)
    B_k = np.array([depot_indices[0]] * k)

    # 2. Calculate cost between each node
    cost = distance.cdist(nodes, nodes, 'euclidean')

    m = gp.Model()

    # A. Integer Constraints (4), (5)
    # Note: All edges are now binary
    x = m.addMVar((k, len(node_indices), len(node_indices)), name='x', vtype=GRB.BINARY)

    # B. Degree Constraints (6), (7), (8), (9), (10)
    # (6) and (7) Only one robot arrives to and leaves from a target (B_k is a depot, so we don't need to remove it from targets)
    _ = m.addConstrs(x[:, i, :].sum() == rp for i in target_indices)
    _ = m.addConstrs(x[:, :, i].sum() == rp for i in target_indices)

    for ki in range(k):
        if rp > 1:
            _ = m.addConstrs(x[ki, i, :].sum() <= 1 for i in target_indices)
            _ = m.addConstrs(x[ki, :, i].sum() <= 1 for i in target_indices)

        # (8) and (9) Begin and end at same position B_k
        # _ = m.addConstr(x[ki,B_k[ki,0],B_k[ki,1],:,:].sum() >= 1)
        _ = m.addConstr(x[ki, B_k[ki], :].sum() <= 1)
        # _ = m.addConstr(x[ki,:,:,B_k[ki,0],B_k[ki,1]].sum() >= 1)
        _ = m.addConstr(x[ki, :, B_k[ki]].sum() <= 1)

        # (10) Every robot that visits a target leaves the target
        _ = m.addConstrs((x[ki, :, i] - x[ki, i, :]).sum() == 0 for i in node_indices)

        # Additional constraint: no loopholes!
        _ = m.addConstrs(x[ki, i, i] == 0 for i in node_indices)

    # C. Capacity and Flow Constraints (11), (12), (13), (14)
    p = m.addMVar((k, len(node_indices), len(node_indices)), name='p', vtype=GRB.INTEGER, lb=0, ub=len(target_indices))

    for ki in range(k):
        # (11) and (12) flow constraints
        right_side = 0
        for i, j in itertools.product(target_indices, node_indices):
            right_side += x[ki, i, j]
        _ = m.addConstr((p[ki, B_k[ki], :] - p[ki, :, B_k[ki]]).sum() == right_side)

        for i in target_indices:
            _ = m.addConstr((p[ki, :, i] - p[ki, i, :]).sum() == x[ki, i, :].sum())

        # (13) Make sure target capacity doesn't change when passing through a depot
        # Note: Disable for now because we only have one depot which is the starting point
        for i in depot_indices:
            if i == B_k[ki]: continue  # Why? See: https://github.com/NESTLab/mrpcp#linear-constraints
            left_side = 0
            for j in node_indices:
                if i == j: continue
                left_side += p[ki, j, i] - p[ki, i, j]
            _ = m.addConstr(left_side == 0)

        # (14) Ensure that target capacity for each robot doesn't exceed |T|
        _ = m.addConstrs(p[ki, i, j] <= len(target_indices) * x[ki, i, j] for i in node_indices for j in node_indices)

    # # D. Fuel Constraints (15), (16), (17), (18), (19), (20)
    r = m.addMVar((k, len(node_indices), len(node_indices)), name='r', vtype=GRB.CONTINUOUS, lb=0, ub=L)  # (20)

    for ki in range(k):
        # (15) and (16)
        for i, j in itertools.product(target_indices, target_indices):
            left_side = r[ki, j, i] - r[ki, i, j] + cost[i, j]
            right_side = M * (1 - x[ki, i, j])
            _ = m.addConstr(left_side <= right_side)
            _ = m.addConstr(left_side >= -right_side)

        # (17) and (18)
        for i, j in itertools.product(depot_indices, target_indices):
            left_side = r[ki, j, i] - L + cost[i, j]
            right_side = M * (1 - x[ki, i, j])
            _ = m.addConstr(left_side >= -right_side)
            _ = m.addConstr(left_side <= right_side)

            # (19)
            _ = m.addConstr(r[ki, j, i] - cost[j, i] >= -M * (1 - x[ki, j, i]))

    # Set objective function (3)
    p_max = m.addVar(vtype=GRB.CONTINUOUS, name="p_max")
    _ = m.addConstrs((cost * x[ki]).sum() <= p_max for ki in range(k))
    m.setObjective(p_max)

    def extract_and_calculate_milp_costs(x, start_nodes, num_robots, num_nodes, cost_matrix):
        print("Extracting Costs")
        milp_costs = []
        milp_paths = []

        for ki in range(num_robots):
            current_node = start_nodes[ki]  # Start at the robot's starting node
            path = [current_node]  # Initialize path with start node
            visited = {current_node}  # Set to keep track of visited nodes

            while len(visited) < num_nodes:
                next_node = np.argmax(x[ki, current_node, :])
                if next_node in visited:
                    break  # Avoid revisiting nodes
                path.append(next_node)
                visited.add(next_node)
                current_node = next_node

            # Filter out the largest node if it exists in the path
            path = [node for node in path if node != max(path)]

            milp_paths.append(path)
            milp_costs.append(calculate_path_cost(path, cost_matrix))

        return milp_paths, milp_costs

    def calculate_path_cost(path, cost_matrix):
        total_cost = 0
        for i in range(len(path) - 1):
            total_cost += cost_matrix[path[i], path[i + 1]]
        # Add cost of returning to the starting depot
        total_cost += cost_matrix[path[-1], path[0]]
        return total_cost

    class MILPSolver:
        min_cost_edges = None
        min_cost = np.inf
        selected_nodes = None
        opt_node_paths = None
        opt_world_paths = None
        start_time = None
        sol_counter = 0

        def __init__(self, model, num_threads=1):
            self.model = model
            MILPSolver.selected_nodes = []
            self.num_threads = num_threads
            MILPSolver.start_time = time.time()

        @staticmethod
        def cb(what, where):
            if where == GRB.Callback.MIPSOL and what.cbGet(GRB.Callback.MIPSOL_OBJ) < MILPSolver.min_cost:
                MILPSolver.min_cost = what.cbGet(GRB.Callback.MIPSOL_OBJ)
                print(f"Found a new solution with lower cost({MILPSolver.min_cost:.3f})!")
                MILPSolver.min_cost_edges = what.cbGetSolution(what._x)

                # Convert edges to the node/world paths
                MILPSolver.opt_node_paths = convertEdgesToPaths(MILPSolver.min_cost_edges, nodes, depot_indices)
                MILPSolver.opt_world_paths = []
                for ki in range(len(MILPSolver.opt_node_paths)):
                    robot_world_path = []
                    for i, subtour in enumerate(MILPSolver.opt_node_paths[ki]):
                        robot_world_path.append(nodes[subtour].tolist())
                    MILPSolver.opt_world_paths.append(robot_world_path)

                metadata["k"] = len(MILPSolver.opt_node_paths)
                # Generate the visualizations
                if "visualize_paths_graph_path" in metadata:
                    metadata_new_sol = metadata.copy()
                    metadata_new_sol["milp_visualize_subfolder"] = os.path.join(metadata_new_sol["milp_visualize_subfolder"], f"{MILPSolver.sol_counter}")
                    os.makedirs(metadata_new_sol["milp_visualize_subfolder"], exist_ok=True)
                    metadata_new_sol["visualize_paths_graph_path"] = os.path.join(metadata_new_sol["milp_visualize_subfolder"], metadata["visualize_paths_graph_path"].split("/")[-1])
                    metadata_new_sol["visitation_frequency_graph_path"] = os.path.join(metadata_new_sol["milp_visualize_subfolder"], metadata["visitation_frequency_graph_path"].split("/")[-1])
                    metadata_new_sol["percent_coverage_visualization"] = os.path.join(metadata_new_sol["milp_visualize_subfolder"], metadata["percent_coverage_visualization"].split("/")[-1])
                    metadata_new_sol["node_visitation_heatmap"] = os.path.join(metadata_new_sol["milp_visualize_subfolder"], metadata["node_visitation_heatmap"].split("/")[-1])
                    metadata_new_sol["mean_time_between_revisitation"] = os.path.join(metadata_new_sol["milp_visualize_subfolder"], metadata["mean_time_between_revisitation"].split("/")[-1])
                    run_visualization_pipeline(MILPSolver.opt_node_paths, MILPSolver.opt_world_paths, metadata_new_sol)
                    plt.close()

                    # Save result in a JSON file within the cache folder
                    result_data = {'job_id': metadata["job_id"],
                                   'params': {'k': k, 'n_a': n_a, 'ssd': square_side_dist, 'fcr': fuel_capacity_ratio, 'rp': rp, 'mode': 'm'},
                                   'robot_node_path': MILPSolver.opt_node_paths, 'robot_world_path': MILPSolver.opt_world_paths,
                                   'status': 'in-progress'}
                    stats_data = {'job_id': metadata["job_id"], 'runtime': time.time() - MILPSolver.start_time,
                                  'average_coverage': metadata_new_sol['average_coverage']}
                    metadata["saveResultsToCache"](metadata["job_id"], result_data, os.path.join(metadata_new_sol["milp_visualize_subfolder"], f"result.json"))  # Save the results to the cache
                    metadata["saveResultsToCache"](metadata["job_id"], stats_data, os.path.join(metadata_new_sol["milp_visualize_subfolder"], f"stats.json"))
                    MILPSolver.sol_counter += 1

                # If this solution's maximum costing tour ~= the cost of tour that only travels between depot and the furthest node,
                # then, this is guaranteed to be optimal.
                if (MILPSolver.min_cost - L_min) < 0.10:
                    print("!This is guaranteed to be the optimal solution!")
                    what.terminate()
                    # visualize_paths_brute_force(MILPSolver.min_cost_edges)

        def solve(self):
            self.model.optimize(MILPSolver.cb)

    num_threads_available = os.cpu_count()  # Get the number of available CPU threads

    # Print the number of available CPU threads
    print(f"Number of available CPU threads: {num_threads_available}")

    solver = MILPSolver(m, num_threads_available - 1)  # Create an instance of your MILP solver with multi-threading

    # Set the number of threads for Gurobi
    # grb.setParam('Threads', num_threads_available-1)
    m._x = x
    # solver = MILPSolver(m)
    solver.solve()  # Optimize until the first optimal solution is found

    # milp_solution_x = np.array([x[ki].x for ki in range(k)]).reshape(k, len(node_indices), len(node_indices))
    # milp_paths, milp_costs = extract_and_calculate_milp_costs(milp_solution_x, B_k, k, len(node_indices), cost)

    # # Apply 2-opt/3-opt algorithm to each path -> iteratively remove two/three edges and reconnect the two paths in a
    # # different way that reduces the total distance.
    # optimized_paths_2opt = []  # Initialize an empty list to store optimized paths
    # optimized_paths_kopt = []
    # for path in milp_paths:
    #     opt_path2, opt_dist2 = k_opt(path, cost, 2)
    #     optimized_paths_2opt.append(opt_path2)
    #
    #     # Apply 3-opt algorithm to each path
    #     # opt_pathk, opt_distk = k_opt(path, cost, 3)
    #     # optimized_paths_kopt.append(opt_pathk)
    #
    # # Calculate costs for each robot
    # optimized_costs_2opt = [calculate_path_cost(path, cost) for path in optimized_paths_2opt]  # two opt
    #
    # # optimized_costs_kopt = [calculate_path_cost(path, cost) for path in
    # #                         optimized_paths_kopt]  # Calculate costs for each robot with 3-opt
    #
    # # Call the updated visualization function with costs
    # # visualize_individual_paths(optimized_paths_2opt, nodes, targets, depots, B_k, optimized_costs_2opt, metadata)
    # # visualize_individual_paths(optimized_paths_kopt, nodes, targets, depots, B_k, optimized_costs_kopt)  # three opt
    #
    # # Calculate cost reduction for each robot
    # for index, (milp_cost, opt_cost) in enumerate(zip(milp_costs, optimized_costs_2opt)):
    #     cost_reduction = milp_cost - opt_cost
    #     print(f"Cost reduction for Robot (two-opt) {index + 1}: {cost_reduction:.2f}")
    #
    # # Calculate cost reduction for each robot
    # # for index, (milp_cost, opt_cost) in enumerate(zip(milp_costs, optimized_costs_kopt)):
    # #     cost_reduction = milp_cost - opt_cost
    # #     print(f"Cost reduction for Robot (k-opt) {index + 1}: {cost_reduction:.2f}")
    #
    # print("MILP solution completed...returning paths to server endpoint /solve")
    # # worldPath = convertToWorldPath(n_a, d, optimized_paths_2opt)
    # # print(paths)
    # print(milp_paths)

    # print("The optimized paths with 2-OPT are: ", optimized_paths_2opt)
    # print("The paths are: ", paths)
    # print("The world paths are: ", worldPath)
    # print("Returning MILP solution to be sent to a json file...")
    return solver.opt_node_paths, solver.opt_world_paths, metadata


def convertEdgesToPaths(edges, nodes, depot_indices):
    edges = np.copy(edges)
    paths = []
    for ki in range(len(edges)):
        subtour_idx = 0
        list_of_subtours = [[]]
        curr_node = depot_indices[0]
        list_of_subtours[subtour_idx].append(curr_node)
        while True:
            if edges[ki][curr_node].sum() < 0.5:
                break
            next_node = np.argmax(edges[ki][curr_node])
            list_of_subtours[subtour_idx].append(int(depot_indices[0]) if next_node == depot_indices[1] else int(next_node))
            edges[ki][curr_node][next_node] = 0
            curr_node = next_node
            if curr_node == depot_indices[1]:
                subtour_idx += 1
                list_of_subtours.append([int(depot_indices[0])])

        filtered_list_of_subtours = []
        for i, si in enumerate(list_of_subtours):
            all_depot = True
            for ni in si:
                if not np.isclose(nodes[ni], nodes[depot_indices[0]]).all():
                    all_depot = False
            if not all_depot:
                filtered_list_of_subtours.append(si)
        if len(filtered_list_of_subtours):
            paths.append(filtered_list_of_subtours)
    return paths


if __name__ == "__main__":
    # Make sure matplotlib gui is turned off
    matplotlib.use('Agg')

    num_of_robots = 3
    n_a = 4
    square_side_dist = 3.
    fuel_capacity_ratio = 1.5
    rp = 2

    job_id = f"mrpcp-standalone/{num_of_robots}_{n_a}_{square_side_dist}_{fuel_capacity_ratio}_{rp}_m"

    metadata = {"mode": "h1",
                "v": 0.2,
                "t": 300.,
                "dt": 0.1,
                "lookback_time": 30.,
                "visualize_paths_graph_path": saveGraphPath(job_id, "all_robot_paths.png"),
                "visitation_frequency_graph_path": saveGraphPath(job_id, "visitation_frequency.png"),
                "percent_coverage_visualization": saveGraphPath(job_id, "percent_coverage_visualization.png"),
                "node_visitation_heatmap": saveGraphPath(job_id, "node_visitation_heatmap.png"),
                "mean_time_between_revisitation": saveGraphPath(job_id, "mean_time_between_revisitation.png"),
                "milp_visualize_subfolder": saveGraphPath(job_id, "intermediate_solutions/"),
                "job_id": job_id,
                "saveResultsToCache": saveResultsToCache,
                }
    start_time = time.time()
    optimized_node_paths, optimized_world_paths, metadata = solve_milp_with_optimizations(num_of_robots,
                                                                             n_a,
                                                                             square_side_dist,
                                                                             fuel_capacity_ratio,
                                                                             rp,
                                                                             metadata)

    metadata = run_visualization_pipeline(optimized_node_paths, optimized_world_paths, metadata)

    # Save result in a JSON file within the cache folder
    result_data = {'job_id': metadata["job_id"],
                   'params': {'k': num_of_robots, 'n_a': n_a, 'ssd': square_side_dist, 'fcr': fuel_capacity_ratio, 'rp': rp, 'mode': 'm'},
                   'robot_node_path': optimized_node_paths, 'robot_world_path': optimized_world_paths,
                   'status': 'completed'}
    stats_data = {'job_id': metadata["job_id"], 'runtime': time.time() - start_time,
                  'average_coverage': metadata['average_coverage']}
    metadata["saveResultsToCache"](metadata["job_id"], result_data, "result.json")  # Save the results to the cache
    metadata["saveResultsToCache"](metadata["job_id"], stats_data, "stats.json")

