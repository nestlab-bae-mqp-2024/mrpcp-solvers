import copy
import itertools
import time
from typing import Dict

import gurobipy as gp
import matplotlib.pyplot as plt
import numpy as np
from gurobipy import GRB
from scipy.spatial import distance

from src.http_server.json_handlers import *
from src.http_server.utils.tsp_solver import k_opt
from src.utils.construct_map import construct_map
from src.visualization.visualization_pipeline import run_visualization_pipeline, visualize_paths


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


def solve_milp_with_optimizations(num_of_robots: int,
                                  nodes_per_axis: int,
                                  square_side_dist: float,
                                  fuel_capacity_ratio: float,
                                  rp: int,
                                  metadata: Dict = None):
    if metadata is None:
        metadata = {}

    # 0. Set params
    # Chose number of robots
    k = num_of_robots
    # Chose the number of targets in an axis
    n = nodes_per_axis
    # Chose the length of distance of each side of the square arena
    ssd = square_side_dist
    # Chose the length of distance between the outermost nodes of each axis
    sr = ssd / (np.sqrt(2.) * n)
    d = ssd - np.sqrt(2.) * sr
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    rp = min(rp, k)
    # Fuel Capacity Parameters
    max_fuel_cost_to_node = d * np.sqrt(2)  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L_min = max_fuel_cost_to_node * 2  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L = L_min * fuel_capacity_ratio  # Fuel capacity (1 unit of fuel = 1 unit of distance)
    M = L + max_fuel_cost_to_node

    # meta data given params
    metadata["k"] = k
    metadata["n_a"] = n
    metadata["ssd"] = ssd
    metadata["ssd_discrete"] = d
    metadata["fcr"] = fuel_capacity_ratio
    metadata["rp"] = rp
    # metadata derived params
    metadata["L_min"] = L
    metadata["mode"] = "m"

    # 1. Create map and get node indices
    start = time.time()
    nodes, node_indices, target_indices, depot_indices = construct_map(n, ssd, d, milp=True, gfx_sanity_check=True)
    B_k = np.array([depot_indices[0]] * k)
    print(f"Constructing the map took {time.time() - start} seconds.")

    # 2. Calculate cost between each node
    start = time.time()
    cost = distance.cdist(nodes, nodes, 'euclidean')
    print(f"Constructing the cost matrix took {time.time() - start} seconds.")

    m = gp.Model()

    # Note: All equation numbers are from the MRPCP paper
    # A. Integer Constraints (4), (5)
    start = time.time()
    x = m.addMVar((k, len(node_indices), len(node_indices)), name='x', vtype=GRB.BINARY)
    print(f"Setting up (A.) integer constraints took {time.time() - start} seconds.")

    # B. Degree Constraints (6), (7), (8), (9), (10)
    start = time.time()
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
    print(f"Setting up (B.) degree constraints took {time.time() - start} seconds.")

    # C. Capacity and Flow Constraints (11), (12), (13), (14)
    start = time.time()
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
    print(f"Setting up (C.) capacity and flow constraints took {time.time() - start} seconds.")

    # D. Fuel Constraints (15), (16), (17), (18), (19), (20)
    start = time.time()
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
    print(f"Setting up (D.) fuel constraints took {time.time() - start} seconds.")

    # Set objective function (3)
    p_max = m.addVar(vtype=GRB.CONTINUOUS, name="p_max")
    _ = m.addConstrs((cost * x[ki]).sum() <= p_max for ki in range(k))
    m.setObjective(p_max)

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
                print(f"Found a new solution with lower cost({MILPSolver.min_cost:.3f})!", flush=True)
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

                # Save the milp found solution for visualization
                milp_node_paths = copy.deepcopy(MILPSolver.opt_node_paths)

                # Run TSP on the MILP discovered paths
                for kii in range(len(MILPSolver.opt_node_paths)):
                    for sii in range(len(MILPSolver.opt_node_paths[kii])):
                        MILPSolver.opt_node_paths[kii][sii], _ = k_opt(MILPSolver.opt_node_paths[kii][sii], cost, 2)
                        # print(f"{kii=} {sii=} {MILPSolver.opt_node_paths[kii][sii]=}")

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

                    metadata_nnew_sol = metadata_new_sol.copy()
                    metadata_nnew_sol["visualize_paths_graph_path"] = os.path.join(metadata_new_sol["milp_visualize_subfolder"], "milp.png")
                    print("Visualizing MILP produced result")
                    visualize_paths(milp_node_paths, metadata_nnew_sol)
                    metadata_nnew_sol["visualize_paths_graph_path"] = os.path.join(metadata_new_sol["milp_visualize_subfolder"], "tsp.png")
                    print("Visualizing post-processed result")
                    visualize_paths(MILPSolver.opt_node_paths, metadata_nnew_sol)

                    print("Running the visualization pipeline on the post-processed result")
                    run_visualization_pipeline(MILPSolver.opt_node_paths, MILPSolver.opt_world_paths, metadata_new_sol)
                    plt.close("all")

                    # Save result in a JSON file within the cache folder
                    result_data = {'job_id': metadata["job_id"],
                                   'params': {'k': k, 'n_a': n, 'ssd': square_side_dist, 'fcr': fuel_capacity_ratio, 'rp': rp, 'mode': 'm'},
                                   'robot_node_path': MILPSolver.opt_node_paths, 'robot_world_path': MILPSolver.opt_world_paths,
                                   'status': 'in-progress'}
                    stats_data = {'job_id': metadata["job_id"], 'runtime': time.time() - MILPSolver.start_time,
                                  'average_coverage': metadata_new_sol['average_coverage']}
                    metadata["saveResultsToCache"](metadata["job_id"], result_data, os.path.join(metadata_new_sol["milp_visualize_subfolder"], f"result.json"))  # Save the results to the cache
                    metadata["saveResultsToCache"](metadata["job_id"], stats_data, os.path.join(metadata_new_sol["milp_visualize_subfolder"], f"stats.json"))
                    MILPSolver.sol_counter += 1

                # If this solution's maximum costing tour ~= the cost of tour that only travels between depot and the furthest node,
                # then, this is guaranteed to be optimal.
                if (MILPSolver.min_cost - L_min) < 0.01:
                    print("!This is guaranteed to be the optimal solution!")
                    what.terminate()
                    # visualize_paths_brute_force(MILPSolver.min_cost_edges)

        def solve(self):
            self.model.optimize(MILPSolver.cb)

    os.makedirs(metadata["milp_visualize_subfolder"], exist_ok=True)
    m.setParam("LogFile", os.path.join(metadata["milp_visualize_subfolder"], "gurobi.log"))

    if "TimeLimit" in metadata:
        m.setParam("TimeLimit", metadata["TimeLimit"])
        print(f"Set a time limit of {metadata['TimeLimit']} seconds")

    if "num_threads_available" in metadata and metadata["num_threads_available"] > 0:
        num_threads_available = metadata["num_threads_available"]
    else:
        num_threads_available = os.cpu_count() - 1  # Get the number of available CPU threads

    # Print the number of available CPU threads
    print(f"Number of available CPU threads: {num_threads_available}")

    solver = MILPSolver(m, num_threads_available)  # Create an instance of your MILP solver with multi-threading

    # Set the number of threads for Gurobi
    # grb.setParam('Threads', num_threads_available-1)
    m._x = x
    # solver = MILPSolver(m)
    solver.solve()  # Optimize until the first optimal solution is found or time limit is reached
    return solver.opt_node_paths, solver.opt_world_paths, metadata


if __name__ == "__main__":
    # Make sure matplotlib gui is turned off
    # matplotlib.use('Agg')

    num_of_robots = 8
    n_a = 8
    square_side_dist = 3.
    fuel_capacity_ratio = 1.5
    rp = 3

    job_id = f"mrpcp-standalone/{num_of_robots}_{n_a}_{square_side_dist}_{fuel_capacity_ratio}_{rp}_m"

    metadata = {"mode": "m",
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
                "TimeLimit": 100,
                "num_threads_available": 10
                }
    start_time = time.time()
    optimized_node_paths, optimized_world_paths, metadata = solve_milp_with_optimizations(num_of_robots,
                                                                             n_a,
                                                                             square_side_dist,
                                                                             fuel_capacity_ratio,
                                                                             rp,
                                                                             metadata)

    print("Finished running MILP, running the visualization pipeline on the best result...")
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

