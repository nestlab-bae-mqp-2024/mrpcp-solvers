import numpy as np


def calculate_total_distance(path, cost_matrix):
    total_cost = 0
    for i in range(len(path) - 1):
        total_cost += cost_matrix[path[i], path[i + 1]]
    total_cost += cost_matrix[path[-1], path[0]]  # Return to start
    return total_cost


def k_opt(route, cost_matrix, ki):
    best_distance = calculate_total_distance(route, cost_matrix)
    best_route = route

    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 1):
            for j in range(i + 1, len(route)):
                if j - i < ki - 1:
                    continue  # Ensure the segment size is at least k

                new_route = route.copy()
                new_route[i:j] = reversed(route[i:j])  # Reverse the segment between i and j

                new_distance = calculate_total_distance(new_route, cost_matrix)

                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = new_route
                    improved = True

        route = best_route

    best_route.append(best_route[0])
    return best_route, calculate_total_distance(best_route, cost_matrix)
