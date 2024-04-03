"""
Author: N/A
Date: 2/17/2024

This program defines the methods relating to solving the MRPCP (Multi-Robot Path Coverage Problem) using MILP (Mixed-Integer Linear Programming)
and TSP (Travelling Salesman Problem) using 2-OPT and k-OPT algorithms.
"""
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


def solve_milp_with_optimizations(num_of_robots: int,
                                  nodes_to_robot_ratio: int,
                                  square_side_dist: float,
                                  fuel_capacity_ratio: float,
                                  failure_rate: int,
                                  job_id: str = None,
                                  metadata: Dict = None):
    """
    This function solves the MRPCP using MILP (Mixed-Integer Linear Programming) and TSP (Travelling Salesman Problem) using 2-OPT and k-OPT algorithms.
    :return: The optimized paths with 2-OPT and the world path
    """
    if metadata is None:
        metadata = {}
    k = num_of_robots  # Chose the number of robots
    n_a = k * nodes_to_robot_ratio  # Chose the number of targets in an axis
    MDBF = 100.0  # Mean Distance Between Failures
    alpha = 0.00001*failure_rate
    rpp = alpha * MDBF  # redundancy parameter percentage
    # Choose the redundancy parameter (have each target be visited by exactly that many robots)
    rp = np.ceil(k * rpp) + 1
    d = square_side_dist / 2.  # Distance per side of the square
    max_fuel_cost_to_node = square_side_dist * np.sqrt(
        2)  # √8 is the max possible distance between our nodes (-1, -1) and (1, 1)
    L = fuel_capacity_ratio * max_fuel_cost_to_node * 2  # Fuel capacity (1 unit of fuel = 1 unit of distance)
    M = L + max_fuel_cost_to_node

    # meta data given params
    metadata["k"] = k
    metadata["nk"] = nodes_to_robot_ratio
    metadata["ssd"] = square_side_dist
    metadata["fcr"] = fuel_capacity_ratio
    metadata["fr"] = failure_rate
    # metadata derived params
    metadata["n"] = n_a
    metadata["rp"] = rp
    metadata["L_min"] = L
    metadata["mode"] = "m"

    # Create a uniform (n*n, 2) numpy target grid for MAXIMUM SPEED
    targets = np.mgrid[-d:d:n_a * 1j, -d:d:n_a * 1j]  # Size d x d
    targets = targets.reshape(targets.shape + (1,))
    targets = np.concatenate((targets[0], targets[1]), axis=2)
    targets = targets.reshape((n_a * n_a, 2))
    target_indices = range(len(targets))
    depots = np.array([
        [-1., -1.],
    ]) * d
    depots = np.concatenate((depots, depots))
    depot_indices = range(len(targets), len(targets) + len(depots))
    nodes = np.concatenate((targets, depots))
    node_indices = range(len(targets) + len(depots))
    B_k = np.array([depot_indices[0]] * k)

    # Graphical sanity check
    plt.figure()
    plt.scatter(targets[:, 0], targets[:, 1], c='blue', s=10)
    plt.scatter(depots[:, 0], depots[:, 1], c='red', s=50)
    plt.grid()

    # Label nodes with node IDs and their positions
    for i, node in enumerate(nodes):
        plt.text(node[0], node[1], f'{i}', fontsize=8, ha='center')
    plt.show()

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
        # (8) and (9) Begin and end at same position B_k
        _ = m.addConstr(x[ki, B_k[ki], :].sum() <= 1)
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
    r = m.addMVar((len(node_indices)), name='r', vtype=GRB.CONTINUOUS, lb=0, ub=L)  # (20)

    for ki in range(k):
        # (15) and (16)
        for i, j in itertools.product(target_indices, target_indices):
            left_side = r[j] - r[i] + cost[i, j]
            right_side = M * (1 - x[ki, i, j])
            _ = m.addConstr(left_side <= right_side)
            _ = m.addConstr(left_side >= -right_side)

        # (17) and (18)
        for i, j in itertools.product(depot_indices, target_indices):
            left_side = r[j] - M + cost[i, j]
            right_side = M * (1 - x[ki, i, j])
            _ = m.addConstr(left_side >= -right_side)
            _ = m.addConstr(left_side <= right_side)

            # (19)
            _ = m.addConstr(r[j] - cost[j, i] >= -M * (1 - x[ki, j, i]))

    # Set objective function (3)
    p_max = m.addVar(vtype=GRB.CONTINUOUS, name="p_max")
    _ = m.addConstrs((cost * x[ki]).sum() <= p_max for ki in range(k))
    m.setObjective(p_max)

    def visualize_paths_brute_force(edges):
        # Only plot the paths for the robots that were assigned a path
        active_robots = []
        for ki in range(k):
            if (cost * edges[ki]).sum() > 0.01:
                active_robots.append(ki)

        subplot_per_hor_axis = int(np.ceil(np.sqrt(len(active_robots))))
        subplot_per_vert_axis = int(np.ceil(len(active_robots) / subplot_per_hor_axis))
        fig, axs = plt.subplots(subplot_per_hor_axis, subplot_per_vert_axis,
                                figsize=(subplot_per_hor_axis * 4, subplot_per_vert_axis * 4))
        fig.tight_layout()
        fig.subplots_adjust(bottom=0.1, top=0.9, right=0.9, left=0.1, wspace=0.3, hspace=0.3)

        hor_i = 0
        vert_i = 0
        for robot_i, ki in enumerate(active_robots):
            if subplot_per_hor_axis == 1 and subplot_per_vert_axis == 1:
                ax = axs
            elif subplot_per_vert_axis == 1:
                ax = axs[hor_i]
            else:
                ax = axs[hor_i][vert_i]
            ax.set_title(f"Robot #{robot_i + 1} (cost={(cost * edges[ki]).sum():.3f})")
            ax.scatter(targets[:, 0], targets[:, 1], c='blue', s=10)
            ax.scatter(depots[:, 0], depots[:, 1], c='red', s=50)
            ax.scatter(nodes[B_k[ki], 0], nodes[B_k[ki], 1], c='red', s=100)

            for i, j in itertools.product(node_indices, node_indices):
                if edges[ki, i, j] > 0.5:  # In case there is any floating math errors
                    # print(f"Connection from {[i1,j1]} to {[i2,j2]}")
                    ax.scatter(nodes[i, 0], nodes[i, 1], c="purple", s=8)
                    ax.scatter(nodes[j, 0], nodes[j, 1], c="purple", s=8)
                    ax.plot([nodes[i, 0], nodes[j, 0]], [nodes[i, 1], nodes[j, 1]], color="purple", linewidth=1)

            vert_i += 1
            if vert_i >= subplot_per_vert_axis:
                vert_i = 0
                hor_i += 1
            ax.grid()

        for h in range(subplot_per_hor_axis):
            for v in range(subplot_per_vert_axis):
                if subplot_per_hor_axis == 1 and subplot_per_vert_axis == 1:
                    ax = axs
                elif subplot_per_vert_axis == 1:
                    ax = axs[h]
                else:
                    ax = axs[h][v]
                ax.set_box_aspect(1)

        fig.suptitle(
            f"Paths for all robots (# of active/available robots={len(active_robots)}/{k}, sum of costs={(cost * edges).sum():.3f})")
        plt.show()

    def extract_and_calculate_milp_costs(x, start_nodes, num_robots, num_nodes, cost_matrix):
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

        def __init__(self, model, num_threads=1):
            self.model = model
            MILPSolver.selected_nodes = []
            self.num_threads = num_threads

        @staticmethod
        def cb(what, where):
            if where == GRB.Callback.MIPSOL and what.cbGet(GRB.Callback.MIPSOL_OBJ) < MILPSolver.min_cost:
                MILPSolver.min_cost = what.cbGet(GRB.Callback.MIPSOL_OBJ)
                print(f"Found a new solution with lower cost({MILPSolver.min_cost:.3f})!")
                MILPSolver.min_cost_edges = what.cbGetSolution(what._x)
                visualize_paths_brute_force(MILPSolver.min_cost_edges)

                # If this solution's maximum costing tour ~= the cost of the tour that only travels between depot and the furthest node,
                # then, this is guaranteed to be optimal.
                if (MILPSolver.min_cost - max_fuel_cost_to_node * 2) < 0.01:
                    print("!This is guaranteed to be the optimal solution!")
                    what.terminate()

        def solve(self):
            self.model.optimize(MILPSolver.cb)

    num_threads_available = os.cpu_count()  # Get the number of available CPU threads

    # Print the number of available CPU threads
    print(f"Number of available CPU threads: {num_threads_available}")

    solver = MILPSolver(m, num_threads_available - 1)  # Create an instance of your MILP solver with multi-threading

    # import gurobipy as grb

    # Set the number of threads for Gurobi
    # grb.setParam('Threads', num_threads_available-1)
    m._x = x
    # solver = MILPSolver(m)
    solver.solve()  # Optimize until the first optimal solution is found

    milp_solution_x = np.array([x[ki].x for ki in range(k)]).reshape(k, len(node_indices), len(node_indices))
    milp_paths, milp_costs = extract_and_calculate_milp_costs(milp_solution_x, B_k, k, len(node_indices), cost)

    # Apply 2-opt/3-opt algorithm to each path -> iteratively remove two/three edges and reconnect the two paths in a
    # different way that reduces the total distance.
    optimized_paths_2opt = []  # Initialize an empty list to store optimized paths
    optimized_paths_kopt = []
    for path in milp_paths:
        opt_path2, opt_dist2 = k_opt(path, cost, 2)
        optimized_paths_2opt.append(opt_path2)

        # Apply 3-opt algorithm to each path
        opt_pathk, opt_distk = k_opt(path, cost, 3)
        optimized_paths_kopt.append(opt_pathk)

    # Calculate costs for each robot
    optimized_costs_2opt = [calculate_path_cost(path, cost) for path in optimized_paths_2opt]  # two opt

    optimized_costs_kopt = [calculate_path_cost(path, cost) for path in
                            optimized_paths_kopt]  # Calculate costs for each robot with 3-opt

    # Call the updated visualization function with costs
    # visualize_individual_paths(optimized_paths_2opt, nodes, targets, depots, B_k, optimized_costs_2opt, )
    # visualize_individual_paths(optimized_paths_kopt, nodes, targets, depots, B_k, optimized_costs_kopt)  # three opt

    # Calculate cost reduction for each robot
    for index, (milp_cost, opt_cost) in enumerate(zip(milp_costs, optimized_costs_2opt)):
        cost_reduction = milp_cost - opt_cost
        print(f"Cost reduction for Robot (two-opt) {index + 1}: {cost_reduction:.2f}")

    # Calculate cost reduction for each robot
    for index, (milp_cost, opt_cost) in enumerate(zip(milp_costs, optimized_costs_kopt)):
        cost_reduction = milp_cost - opt_cost
        print(f"Cost reduction for Robot (k-opt) {index + 1}: {cost_reduction:.2f}")

    print("MILP solution completed...returning paths to server endpoint /solve")
    worldPath = convertToWorldPath(n_a, d, optimized_paths_2opt)


    print("The optimized paths with 2-OPT are: ", optimized_paths_2opt)
    print("The optimized paths converted to world path are: ", worldPath)
    print("Returning solution to be sent to a json file...")
    return optimized_paths_2opt, worldPath, metadata



