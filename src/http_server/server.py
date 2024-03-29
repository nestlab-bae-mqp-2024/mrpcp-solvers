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
import threading
import time

from flask import Flask, request, jsonify

from src.heuristic_attempts.yasars_heuristic_attempts.yasars_heuristic import yasars_heuristic
from src.http_server import heuristic2
from src.http_server.recalculate import *
from src.http_server.mrpcp import *
import os
import src.http_server.json_handlers as json_handlers
from src.visualization.visualization_pipeline import run_visualization_pipeline

app = Flask(__name__)
analysis_file = "runtime_analysis.txt" # File path for storing runtime analysis

@app.route('/solve', methods=['POST'])
def solve_endpoint():
    """
    This function defines the '/solve' endpoint for solving MILP or Heuristic problems.
    """
    print("Solution request received.")
    k = request.args.get('k')
    nk = request.args.get('nk')
    ssd = request.args.get('ssd')
    fcr = request.args.get('fcr')
    fr = request.args.get('fr')
    mode = request.args.get('mode')

    # Generate job ID based on parameters
    job_id = f"{k}_{nk}_{ssd}_{fcr}_{fr}_{mode}"

    # Get the current working directory
    current_dir = os.getcwd()

    # Define the cache folder path relative to the current directory
    cache_folder_path = os.path.join(current_dir, 'cache')

    # Check if folder with job ID exists in the cache folder
    job_folder_path = os.path.join(cache_folder_path, job_id)

    if os.path.exists(os.path.join(job_folder_path, 'result.json')):
        print(f"Job folder exists: {job_folder_path}. Returning the content of the JSON file...")
        # If folder exists, read JSON file and return its content
        with open(os.path.join(job_folder_path, 'result.json'), 'r') as file:
            result = json.load(file)
        print(f"Contents of the JSON file: {result}")  # Print contents of JSON file
        return jsonify(result), 200

    print(
        f"Job folder does not exist: {job_folder_path}. Starting solver function with parameters k={k}, nk={nk}, ssd={ssd}, fcr={fcr}, fr={fr}, job_id={job_id}, mode={mode}...")

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
    solve_thread = threading.Thread(name=job_id, target=run_solver, args=(k, nk, ssd, fcr, fr, mode, job_id))
    solve_thread.start()
    result_data = {'job_id': job_id, 'params': {'k': k, 'nk': nk, 'ssd': ssd, 'fcr': fcr, 'fr': fr, 'mode': mode},
                   'robot_node_path': None, 'robot_world_path': None, 'status': 'in progress'}
    return jsonify(result_data), 200


def run_solver(k, nk, ssd, fcr, fr, mode, job_id):
    """
    This function runs the MILP solver function with the provided parameters.
    Once the solver completes, it saves the result to a JSON file in the cache folder.
    Params:
        num_of_robots: int,
        nodes_to_robot_ratio: int,
        square_side_dist: int,
        fuel_capacity_ratio: float,
        failure_rate: int,
        job_id: str
    """
    try:
        if mode == 'm':
            # Run MILP solver function with parameters
            start_time = time.time()
            metadata = {"visualize_paths_graph_path": saveGraphPath(job_id, "all_robot_paths.png"),
                        "visitation_frequency_graph_path": saveGraphPath(job_id, "visitation_frequency.png")}
            print(
                f"Running MILP solver function with parameters: k={k}, nk={nk}, ssd={ssd}, fcr={fcr}, fr={fr}, job_id={job_id}, mode=m...")
            edges, robot_world_path, metadata = solve_milp_with_optimizations(int(k), int(nk), float(ssd), float(fcr), int(fr),
                                                         job_id, metadata)
            metadata = run_visualization_pipeline(edges, robot_world_path, metadata)
            runtime = time.time() - start_time
            log_runtime("MILP", {"k": k, "nk": nk, "ssd": ssd, "fcr": fcr, "fr": fr, "mode": mode}, runtime)
            print(
                f"MILP solver function completed with parameters: k={k}, nk={nk}, ssd={ssd}, fcr={fcr}, fr={fr}, mode=m.")
            # Convert edges to a node format
            robot_node_path = [[int(edge) for edge in path] for path in edges]
            print("Robot node path", robot_node_path)
            print("Robot world path", robot_world_path)
            # Save result in a JSON file within the cache folder
            result_data = {'job_id': job_id,
                           'params': {'k': k, 'nk': nk, 'ssd': ssd, 'fcr': fcr, 'fr': fr, 'mode': 'm'},
                           'robot_node_path': robot_node_path, 'robot_world_path': robot_world_path,
                           'status': 'completed'}
            stats_data = {'job_id': job_id, 'runtime': runtime}
            json_handlers.saveResultsToCache(job_id, result_data, 'result.json')  # Save the results to the cache
            json_handlers.saveResultsToCache(job_id, stats_data, 'stats.json')
        elif mode == 'h1':
            # Run Heuristic solver function with parameters
            start_time = time.time()
            print(
                f"Running Heuristic solver function with parameters: k={k}, nk={nk}, ssd={ssd}, fcr={fcr}, fr={fr}, job_id={job_id}  mode=h1...")
            metadata = {"visualize_paths_graph_path": saveGraphPath(job_id, "all_robot_paths.png"),
                        "visitation_frequency_graph_path": saveGraphPath(job_id, "visitation_frequency.png")}
            robot_node_path_w_subtours, robot_world_path, metadata = yasars_heuristic(int(k), int(nk), float(ssd), float(fcr), int(fr), metadata)
            metadata = run_visualization_pipeline(robot_node_path_w_subtours, robot_world_path, metadata)

            runtime = time.time() - start_time
            log_runtime("h1 heuristic", {"k": k, "nk": nk, "ssd": ssd, "fcr": fcr, "fr": fr, "mode": mode}, runtime)
            print(
                f"Heuristic solver function completed with parameters: k={k}, nk={nk}, ssd={ssd}, fcr={fcr}, fr={fr}, job_id={job_id}, mode=h1")
            robot_node_path = []
            for subtours in robot_node_path_w_subtours:
                robot_path = []
                for subtour in subtours:
                    for node in subtour:
                        robot_path.append(int(node))
                robot_node_path.append(robot_path)
            print("Robot node path", robot_node_path)
            print("Robot world path", robot_world_path)
            # Save result in a JSON file within the cache folder
            result_data = {'job_id': job_id,
                           'params': {'k': k, 'nk': nk, 'ssd': ssd, 'fcr': fcr, 'fr': fr, 'mode': 'h1'},
                           'robot_node_path': robot_node_path, 'robot_world_path': robot_world_path,
                           'status': 'completed'}
            stats_data = {'job_id': job_id, 'runtime': runtime}
            json_handlers.saveResultsToCache(job_id, result_data, 'result.json')
            json_handlers.saveResultsToCache(job_id, stats_data, 'stats.json')
            return result_data  # Return the content of the JSON file

        elif mode == 'h2':
            # Run Heuristic solver function with parameters
            start_time = time.time()
            print(
                f"Running Heuristic solver function with parameters: k={k}, nk={nk}, ssd={ssd}, fcr={fcr}, fr={fr}, job_id={job_id}  mode=h2...")
            metadata = {"visualize_paths_graph_path": saveGraphPath(job_id, "all_robot_paths.png"),
                        "visitation_frequency_graph_path": saveGraphPath(job_id, "visitation_frequency.png")}
            edges, robot_world_path, metadata = heuristic2.generate_robot_paths_redundancy(int(k), int(nk), int(ssd), float(fcr), int(fr), None, None, None, saveGraphPath(job_id, "visualization.png"), metadata)  # Run the other heuristic solver
            # metadata = run_visualization_pipeline(edges, robot_world_path, metadata)
            runtime = time.time() - start_time
            log_runtime("h2 heuristic", {"k": k, "nk": nk, "ssd": ssd, "fcr": fcr, "fr": fr, "mode": mode}, runtime)
            print(
                f"Heuristic solver function completed with parameters: k={k}, nk={nk}, ssd={ssd}, fcr={fcr}, fr={fr}, mode=h2.")
            print("Robot node path", edges)
            print("Robot world path", robot_world_path)
            # Save result in a JSON file within the cache folder
            result_data = {'job_id': job_id,
                           'params': {'k': k, 'nk': nk, 'ssd': ssd, 'fcr': fcr, 'fr': fr, 'mode': 'h2'},
                           'robot_node_path': edges, 'robot_world_path': robot_world_path,
                           'status': 'completed'}
            stats_data = {'job_id': job_id, 'runtime': runtime}
            json_handlers.saveResultsToCache(job_id, result_data, 'result.json')
            json_handlers.saveResultsToCache(job_id, stats_data, 'stats.json')
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
    k, nk, ssd, fcr, fr, mode = getParamsFromJobId(job_id)
    start_time = time.time()
    metadata = {"visualize_paths_graph_path": saveGraphPath(job_id, "all_robot_paths.png"),
                "visitation_frequency_graph_path": saveGraphPath(job_id, "visitation_frequency.png")}
    robot_node_path, robot_world_path, metadata = heuristic2.generate_robot_paths_redundancy(int(k), int(nk), int(ssd), float(fcr), int(fr), failed_robot_id, curr_robots_pos, curr_fuel_levels, saveGraphPath(job_id, "visualization.png"), metadata)  # Run the other heuristic solver
    metadata = run_visualization_pipeline(robot_node_path, robot_world_path, metadata)
    runtime = time.time() - start_time
    log_runtime("solve_endpoint", {"k": k, "nk": nk, "ssd": ssd, "fcr": fcr, "fr": fr, "mode": mode}, runtime)
    result_data = {'job_id': job_id,
                   'params': {'k': k, 'nk': nk, 'ssd': ssd, 'fcr': fcr, 'fr': fr, 'mode': 'recalc'},
                   'robot_node_path': robot_node_path, 'robot_world_path': robot_world_path,
                   'status': 'completed'}
    stats_data = {'job_id': job_id, 'runtime': runtime}
    json_handlers.saveResultsToCache(job_id, result_data, 'recalculation_result.json')  # Save the results to the cache
    json_handlers.saveResultsToCache(job_id, stats_data, 'stats.json')
    # Run the function to recalculate the paths based on the input parameters
    return jsonify(result_data), 200 # Return the json result of the recalculation


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


if __name__ == '__main__':
    # Create or overwrite the analysis file
    with open(analysis_file, "w") as f:
        f.write("Runtime Analysis\n")
        f.write("----------------\n\n")
    print("Waiting for a request...")  # Added waiting message
    app.run(host='127.0.0.1', port=5000, debug=False, use_reloader=False)
