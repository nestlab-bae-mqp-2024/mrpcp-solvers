from matplotlib import pyplot as plt
from scipy.spatial import distance
import numpy as np
from concurrent.futures import ProcessPoolExecutor
import time
import os

from src.http_server.json_handlers import saveGraphPath
from src.http_server.utils.tsp_solver import k_opt
from src.http_server.utils.visualize import visualize_paths
from src.heuristic_attempts.yasars_heuristic_attempts.yasars_heuristic import yasars_heuristic

def solve_mrpcp_heuristic(num_of_robots, interval, num_of_targets_per_side, redundancy_parameter, fuel_capacity_ratio, dist_per_side, visualization_path):
    return yasars_heuristic(num_of_robots, num_of_targets_per_side, dist_per_side, redundancy_parameter, fuel_capacity_ratio, visualization_path)

