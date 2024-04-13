"""
Author: Samara Holmes
Date: 2/6/2024

This program defines a Flask application that provides an endpoint '/solve' for solving MILP (Mixed-Integer Linear Programming) problems.
The program takes parameters 'k', 'q_k', and 'n' via a POST request to the '/solve' endpoint.
It then checks if a result for the given parameters exists in the cache folder.
If the result exists, it returns the result from the cache.
If not, it runs the MILP solver function 'solve_milp_with_optimizations' with the provided parameters on a separate thread.
Once the solver completes, it saves the result to a JSON file in the cache folder and returns the result to the client.
"""
import json
import threading
import time

from flask import Flask, request, jsonify
from src.http_server.json_handlers import *
from src.heuristic_attempts.yasars_heuristic_attempts.yasars_heuristic import yasars_heuristic
from src.http_server import heuristic2

from src.mrpcp_2015.modified_mrpcp import solve_milp_with_optimizations
import os
from src.http_server.utils.metric_calculations import calculate_mean_distance_per_path
from src.visualization.visualization_pipeline import run_visualization_pipeline

from tqdm import tqdm

app = Flask(__name__)
analysis_file = "runtime_analysis.txt"  # File path for storing runtime analysis

@app.route('/solve', methods=['POST'])
def solve_endpoint():
    """
    This function defines the '/solve' endpoint for solving MILP or Heuristic problems.
    """
    print("Solution request received.")
    k = request.args.get('k')
    n_a = request.args.get('n_a')
    ssd = request.args.get('ssd')
    fcr = request.args.get('fcr')
    rp = request.args.get('rp')
    mode = request.args.get('mode')

    # Generate job ID based on parameters
    job_id = f"{k}_{n_a}_{ssd}_{fcr}_{rp}_{mode}"

    # Get the current working directory
    current_dir = os.getcwd()

    # Define the cache folder path relative to the current directory
    cache_folder_path = os.path.join(current_dir, 'cache')

    # Check if folder with job ID exists in the cache folder
    job_folder_path = os.path.join(cache_folder_path, job_id)

    if os.path.exists(os.path.join(job_folder_path, 'result.json')):
        #print(f"Job folder exists: {job_folder_path}. Returning the content of the JSON file...")
        # If folder exists, read JSON file and return its content
        with open(os.path.join(job_folder_path, 'result.json'), 'r') as file:
            result = json.load(file)
        #print(f"Contents of the JSON file: {result}")  # Print contents of JSON file
        #export_world_paths(result)
        adding_to_json_3()
        return jsonify(result), 200

    print(
        f"Job folder does not exist: {job_folder_path}. Starting solver function with parameters k={k}, n_a={n_a}, ssd={ssd}, fcr={fcr}, rp={rp}, job_id={job_id}, mode={mode}...")

    # Check if there's an ongoing solver thread with the same parameters
    for thread in threading.enumerate():
        if thread.name == job_id:
            print(f"Solver thread with the same parameters is already running. Waiting for completion...")
            thread.join()  # Wait for the thread to complete
            # Once the thread completes, read the result from the JSON file
            with open(os.path.join(job_folder_path, 'result.json'), 'r') as file:
                result = json.load(file)
            return jsonify(result), 200

    # Run MILP solver function with parameters on a separate thread
    solve_thread = threading.Thread(name=job_id, target=run_solver, args=(k, n_a, ssd, fcr, rp, mode, job_id))
    solve_thread.start()
    result_data = {'job_id': job_id, 'params': {'k': k, 'n_a': n_a, 'ssd': ssd, 'fcr': fcr, 'fr': rp, 'mode': mode},
                   'robot_node_path': None, 'robot_world_path': None, 'status': 'in progress'}
    return jsonify(result_data), 200


def run_solver(k, n_a, ssd, fcr, rp, mode, job_id, skip_vis=False):
    """
    This function runs the MILP solver function with the provided parameters.
    Once the solver completes, it saves the result to a JSON file in the cache folder.
    Params:
        num_of_robots: int,
        nodes_per_axis: int,
        square_side_dist: int,
        fuel_capacity_ratio: float,
        rp: int,
        job_id: str
    """
    try:
        if mode == 'm':
            # Run MILP solver function with parameters
            start_time = time.time()
            metadata = {"visualize_paths_graph_path": saveGraphPath(job_id, "all_robot_paths.png"),
                        "visitation_frequency_graph_path": saveGraphPath(job_id, "visitation_frequency.png"),
                        "percent_coverage_visualization": saveGraphPath(job_id, "percent_coverage_visualization.png"),
                        "node_visitation_heatmap": saveGraphPath(job_id, "node_visitation_heatmap.png"),
                        "mean_time_between_revisitation": saveGraphPath(job_id, "mean_time_between_revisitation.png"),
                        "milp_visualize_subfolder": saveGraphPath(job_id, "intermediate_solutions/"),
                        "average_coverage": None, "v": 0.2,
                        "t": 3600.,
                        "dt": 0.1,
                        "lookback_time": 5.,
                        "job_id": job_id,
                        "saveResultsToCache": saveResultsToCache}
            print(
                f"Running MILP solver function with parameters: k={k}, n_a={n_a}, ssd={ssd}, fcr={fcr}, rp={rp}, job_id={job_id}, mode=m...")
            robot_node_path_w_subtours, robot_world_path, metadata = solve_milp_with_optimizations(int(k), int(n_a), float(ssd), float(fcr), int(rp), metadata)
            if not skip_vis:
                metadata = run_visualization_pipeline(robot_node_path_w_subtours, robot_world_path, metadata)
            runtime = time.time() - start_time
            log_runtime("MILP", {"k": k, "n_a": n_a, "ssd": ssd, "fcr": fcr, "rp": rp, "mode": mode}, runtime)
            print(f"MILP solver function completed with parameters: k={k}, n_a={n_a}, ssd={ssd}, fcr={fcr}, rp={rp}, mode=m.")
            # Save result in a JSON file within the cache folder
            result_data = {'job_id': job_id,
                           'params': {'k': k, 'n_a': n_a, 'ssd': ssd, 'fcr': fcr, 'rp': rp, 'mode': 'm'},
                           'robot_node_path': robot_node_path_w_subtours, 'robot_world_path': robot_world_path,
                           'status': 'completed'}
            stats_data = {'job_id': job_id, 'runtime': runtime,
                          'mean_distance_per_path': calculate_mean_distance_per_path(robot_world_path),
                          'average_coverage': metadata['average_coverage']}
            saveResultsToCache(job_id, result_data, 'result.json')  # Save the results to the cache
            saveResultsToCache(job_id, stats_data, 'stats.json')
        elif mode == 'h1':
            # Run Heuristic solver function with parameters
            start_time = time.time()
            print(
                f"Running Heuristic solver function with parameters: k={k}, n_a={n_a}, ssd={ssd}, fcr={fcr}, rp={rp}, job_id={job_id}  mode=h1...")
            metadata = {"visualize_paths_graph_path": saveGraphPath(job_id, "all_robot_paths.png"),
                        "visitation_frequency_graph_path": saveGraphPath(job_id, "visitation_frequency.png"),
                        "percent_coverage_visualization": saveGraphPath(job_id, "percent_coverage_visualization.png"),
                        "node_visitation_heatmap": saveGraphPath(job_id, "node_visitation_heatmap.png"),
                        "mean_time_between_revisitation": saveGraphPath(job_id, "mean_time_between_revisitation.png"),
                        "average_coverage": None, "v": 0.2,
                        "t": 3600.,
                        "dt": 0.1,
                        "lookback_time": 5.}
            robot_node_path_w_subtours, robot_world_path, metadata = yasars_heuristic(int(k), int(n_a), float(ssd),
                                                                                      float(fcr), int(rp), metadata, False)
            if not skip_vis:
                metadata = run_visualization_pipeline(robot_node_path_w_subtours, robot_world_path, metadata)
            runtime = time.time() - start_time
            log_runtime("h1 heuristic", {"k": k, "n_a": n_a, "ssd": ssd, "fcr": fcr, "rp": rp, "mode": mode}, runtime)
            print(
                f"Heuristic solver function completed with parameters: k={k}, n_a={n_a}, ssd={ssd}, fcr={fcr}, rp={rp}, job_id={job_id}, mode=h1")
            robot_node_path = []
            for subtours in robot_node_path_w_subtours:
                robot_path = []
                for subtour in subtours:
                    for node in subtour:
                        robot_path.append(int(node))
                robot_node_path.append(robot_path)
            # Save result in a JSON file within the cache folder
            result_data = {'job_id': job_id,
                           'params': {'k': k, 'n_a': n_a, 'ssd': ssd, 'fcr': fcr, 'rp': rp, 'mode': 'h1'},
                           'robot_node_path': robot_node_path, 'robot_world_path': robot_world_path,
                           'status': 'completed'}
            stats_data = {'job_id': job_id, 'runtime': runtime,
                          'mean_distance_per_path': calculate_mean_distance_per_path(robot_world_path),
                          'average_coverage': metadata['average_coverage']}
            saveResultsToCache(job_id, result_data, 'result.json')
            saveResultsToCache(job_id, stats_data, 'stats.json')
            return result_data  # Return the content of the JSON file

        elif mode == 'h2':
            # Run Heuristic solver function with parameters
            start_time = time.time()
            print(
                f"Running Heuristic solver function with parameters: k={k}, n_a={n_a}, ssd={ssd}, fcr={fcr}, rp={rp}, job_id={job_id}  mode=h2...")
            metadata = {"visualize_paths_graph_path": saveGraphPath(job_id, "all_robot_paths.png"),
                        "visitation_frequency_graph_path": saveGraphPath(job_id, "visitation_frequency.png"),
                        "percent_coverage_visualization": saveGraphPath(job_id, "percent_coverage_visualization.png"),
                        "node_visitation_heatmap": saveGraphPath(job_id, "node_visitation_heatmap.png"),
                        "mean_time_between_revisitation": saveGraphPath(job_id, "mean_time_between_revisitation.png"),
                        "average_coverage": None,
                        "v": 0.2,
                        "t": 3600.,
                        "dt": 0.1,
                        "lookback_time": 5.}
            edges, robot_world_path, metadata = heuristic2.generate_robot_paths_redundancy(int(k), int(n_a), int(ssd),
                                                                                           float(fcr), int(rp), None,
                                                                                           None, None,
                                                                                           metadata)  # Run the other heuristic solver
            if not skip_vis:
                metadata = run_visualization_pipeline(edges, robot_world_path, metadata)
            runtime = time.time() - start_time
            log_runtime("h2 heuristic", {"k": k, "n_a": n_a, "ssd": ssd, "fcr": fcr, "rp": rp, "mode": mode}, runtime)
            print(
                f"Heuristic solver function completed with parameters: k={k}, n_a={n_a}, ssd={ssd}, fcr={fcr}, rp={rp}, mode=h2.")
            # Save result in a JSON file within the cache folder
            result_data = {'job_id': job_id,
                           'params': {'k': k, 'n_a': n_a, 'ssd': ssd, 'fcr': fcr, 'rp': rp, 'mode': 'h2'},
                           'robot_node_path': edges, 'robot_world_path': robot_world_path,
                           'status': 'completed'}
            stats_data = {'job_id': job_id, 'runtime': runtime,
                          'mean_distance_per_path': calculate_mean_distance_per_path(robot_world_path),
                          'average_coverage': metadata['average_coverage']}
            saveResultsToCache(job_id, result_data, 'result.json')
            saveResultsToCache(job_id, stats_data, 'stats.json')
            return result_data  # Return the content of the JSON file

    except Exception as e:
        print(f"Error occurred during solving: {e}")


@app.route('/recalculate', methods=['POST'])
def recalc_endpoint():
    """
    This function defines the '/recalculate' endpoint for recalculating the paths
    """
    print("Recalculation request received.")
    job_id = request.args.get('job_id')
    curr_robots_pos = request.args.get('curr_robots_pos')
    curr_fuel_levels = request.args.get('curr_fuel_levels')
    curr_robots_pos = json.loads(curr_robots_pos)
    failed_robot_id = request.args.get('failed_robot_id')
    k, n_a, ssd, fcr, rp, mode = getParamsFromJobId(job_id)
    new_job_id = f"{k}_{n_a}_{ssd}_{fcr}_{rp}_recalc"
    start_time = time.time()
    metadata = {"visualize_paths_graph_path": saveGraphPath(job_id, "all_robot_paths.png"),
                "visitation_frequency_graph_path": saveGraphPath(job_id, "visitation_frequency.png"),
                "percent_coverage_visualization": saveGraphPath(job_id, "percent_coverage_visualization.png"),
                "node_visitation_heatmap": saveGraphPath(job_id, "node_visitation_heatmap.png"),
                "mean_time_between_revisitation": saveGraphPath(job_id, "mean_time_between_revisitation.png"),
                "average_coverage": None,
                "v": 0.2,
                "t": 3600.,
                "dt": 0.1,
                "lookback_time": 5.}
    # TODO: add into account that when you recalculate, you need to update the number of robots you are calculating for
    robot_node_path, robot_world_path, metadata = heuristic2.generate_robot_paths_redundancy(
        int(k - len(curr_robots_pos)), int(n_a), int(ssd), float(fcr), int(rp), failed_robot_id, curr_robots_pos,
        curr_fuel_levels, metadata)  # Run the other heuristic solver
    metadata = run_visualization_pipeline(robot_node_path, robot_world_path, metadata)
    runtime = time.time() - start_time
    log_runtime("solve_endpoint", {"k": k, "n_a": n_a, "ssd": ssd, "fcr": fcr, "rp": rp, "mode": 'recalc'}, runtime)
    result_data = {'job_id': job_id,
                   'params': {'k': k, 'n_a': n_a, 'ssd': ssd, 'fcr': fcr, 'rp': rp, 'mode': 'recalc'},
                   'robot_node_path': robot_node_path, 'robot_world_path': robot_world_path,
                   'status': 'completed'}
    stats_data = {'job_id': new_job_id, 'runtime': runtime, 'average_coverage': metadata['average_coverage']}
    saveResultsToCache(new_job_id, result_data, 'recalculation_result.json')  # Save the results to the cache
    saveResultsToCache(new_job_id, stats_data, 'stats.json')
    # Run the function to recalculate the paths based on the input parameters
    return jsonify(result_data), 200  # Return the json result of the recalculation


def getParamsFromJobId(job_id):
    """
    Function to return all the MRPCP and Heuristic parameters based on the job id
    It takes the job id and splits the id into the parameters
    :param job_id:
    :return:
    """
    params = job_id.split('_')
    return tuple(params)


def log_runtime(func_name, params, runtime):
    """
    Logs the runtime of a function along with its parameters to a text file.
    """
    # Convert runtime to minutes and seconds
    minutes = int(runtime // 60)
    seconds = runtime % 60

    with open(analysis_file, "a") as f:
        f.write(f"Function: {func_name}\n")
        f.write(f"Parameters: {params}\n")
        f.write(f"Runtime: {minutes} minutes {seconds:.6f} seconds\n")
        f.write("\n")


import os

from flask import json, jsonify

from src.visualization.pseudo_simulate import pseudo_simulate
from src.http_server.utils.visualize import visualize_coverage_stepwise_no_plotting

from src.http_server.server import run_solver
from src.visualization.discretization import discretize_world_points

if __name__ == '__main__':
    # Create or overwrite the analysis file
    with open(analysis_file, "w") as f:
        f.write("Runtime Analysis\n")
        f.write("----------------\n\n")
    print("Waiting for a request...")  # Added waiting message
    app.run(host='127.0.0.1', port=5000, debug=False, use_reloader=False)

#%%
