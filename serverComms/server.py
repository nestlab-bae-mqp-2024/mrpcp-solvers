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
from flask import Flask, request, jsonify
from serverComms.mrpcp import *
from serverComms.heuristic import *
import uuid
import requests
import os
import json

app = Flask(__name__)

# Get the current working directory
current_dir = os.getcwd()

# Define the cache folder path relative to the current directory
cache_folder_path = os.path.join(current_dir, 'cache')

"""
This function defines the '/solve' endpoint for solving MILP or Heuristic problems.
"""
@app.route('/solve', methods=['POST'])
def solve_endpoint():
    print("Solution request received.")
    k = request.args.get('k')
    q_k = request.args.get('q_k')
    n_a = request.args.get('n_a')
    rp = request.args.get('rp')
    l = request.args.get('l')
    d = request.args.get('d')
    mode = request.args.get('mode')

    # Generate job ID based on parameters
    job_id = f"{k}_{q_k}_{n_a}_{rp}_{l}_{d}_{mode}"

    # Check if folder with job ID exists in the cache folder
    job_folder_path = os.path.join(cache_folder_path, job_id)
    if os.path.exists(job_folder_path):
        print(f"Job folder exists: {job_folder_path}. Returning the content of the JSON file...")
        # If folder exists, read JSON file and return its content
        with open(os.path.join(job_folder_path, 'result.json'), 'r') as file:
            result = json.load(file)
        print(f"Contents of the JSON file: {result}")  # Print contents of JSON file
        return jsonify(result), 200
    elif mode == 'm':
        print(f"Job folder does not exist: {job_folder_path}. Starting MILP solver function with parameters k={k}, q_k={q_k}, n={n_a}, rp={rp}, l={l}, d={d}, mode={mode}...")
        # Run MILP solver function with parameters on a separate thread
        milp_thread = threading.Thread(target=run_milp_solver, args=(k, q_k, n_a, rp, l, d, job_id, cache_folder_path))
        milp_thread.start()
        return 'MILP solver function started.', 200
    elif mode == 'h':
        print(f"Job folder does not exist: {job_folder_path}. Starting heuristic solver function with parameters k={k}, q_k={q_k}, n={n_a}, rp={rp}, l={l}, d={d}, mode={mode}...")
        # Run heuristic solver function with parameters on a separate thread
        heuristic_thread = threading.Thread(target=run_heuristic_solver, args=(k, q_k, n_a, rp, l, d, job_id, cache_folder_path))
        heuristic_thread.start()
        return 'Heuristic solver function started.', 200
    else:
        return jsonify({'error': 'Invalid mode specified'}), 400



def convertToWorldPath(serializable_edges):
    pass

"""
This function runs the MILP solver function with the provided parameters.
Once the solver completes, it saves the result to a JSON file in the cache folder.
Params:
    k (int): Number of paths
    q_k (float): Maximum number of paths
    n_a (int): Number of nodes per axis
    rp (int): redundancy parameter
    l (float): 
    d (float): distance between nodes
    job_id (str): Job ID
    cache_folder_path (str): Cache folder path
"""
def run_milp_solver(k, q_k, n_a, rp, l, d, job_id, cache_folder_path):
    try:
        # Run MILP solver function with parameters
        print(f"Running MILP solver function with parameters: k={k}, q_k={q_k}, n={n_a}, rp={rp}, l={l}, d={d}, mode=m...")
        edges = solve_milp_with_optimizations(int(k), float(q_k), int(n_a), int(rp), float(l), float(d), job_id)
        print(f"MILP solver function completed with parameters: k={k}, q_k={q_k}, n={n_a}, rp={rp}, l={l}, d={d}, mode=m.")

        if edges:
            # Convert edges to a serializable format
            serializable_edges = [[int(edge) for edge in path] for path in edges]

            # Create job folder
            job_folder_path = os.path.join(cache_folder_path, job_id)
            if not os.path.exists(job_folder_path):
                os.makedirs(job_folder_path)

            # Save result in a JSON file within the cache folder
            result_data = {'job_id': job_id, 'params': {'k': k, 'q_k': q_k, 'n_a': n_a, 'rp': rp, 'l': l, 'd': d, 'mode': 'm'}, 'robot_node_path': serializable_edges, 'robot_world_path': convertToWorldPath(serializable_edges)}
            with open(os.path.join(job_folder_path, 'result.json'), 'w') as file:
                json.dump(result_data, file)
                print("Result and parameters saved to result.json file.")
            return result_data  # Return the content of the JSON file

    except Exception as e:
        print(f"Error occurred during MILP solving: {e}")

"""
This function runs the Heuristic solver function with the provided parameters.
Once the solver completes, it saves the result to a JSON file in the cache folder.
Params:
    k (int): Number of paths
    q_k (float): Maximum number of paths
    n_a (int): Number of nodes per axis
    rp (int): redundancy parameter
    l (float): 
    d (float): distance between nodes
    job_id (str): Job ID
    cache_folder_path (str): Cache folder path
"""
def run_heuristic_solver(k, q_k, n_a, rp, l, d, job_id, cache_folder_path):
    try:
        # Run Heuristic solver function with parameters
        print(f"Running Heuristic solver function with parameters: k={k}, q_k={q_k}, n={n_a}, rp={rp}, l={l}, d={d}, mode=h...")
        edges = solve_mrpcp_heuristic(int(k), float(q_k), int(n_a), int(rp), float(l), float(d), job_id)
        print(f"Heuristic solver function completed with parameters: k={k}, q_k={q_k}, n={n_a}, rp={rp}, l={l}, d={d}, mode=h.")

        if edges:
            # Convert edges to a serializable format
            serializable_edges = [[int(edge) for edge in path] for path in edges]

            # Create job folder
            job_folder_path = os.path.join(cache_folder_path, job_id)
            if not os.path.exists(job_folder_path):
                os.makedirs(job_folder_path)

            # Save result in a JSON file within the cache folder
            result_data = {'job_id': job_id, 'params': {'k': k, 'q_k': q_k, 'n_a': n_a, 'rp': rp, 'l': l, 'd': d, 'mode': 'h'}, 'robot_node_path': serializable_edges, 'robot_world_path': convertToWorldPath(serializable_edges)}
            with open(os.path.join(job_folder_path, 'result.json'), 'w') as file:
                json.dump(result_data, file)
                print("Result and parameters saved to result.json file.")
            return result_data  # Return the content of the JSON file

    except Exception as e:
        print(f"Error occurred during Heuristic solving: {e}")

"""
This function defines the '/recalculate' endpoint for recalculating the paths
"""
@app.route('/recalculate', methods=['POST'])
def recalc_endpoint():
    print("Recalculation request received.")
    job_id = request.args.get('job_id')
    curr_robots_pos = request.args.get('curr_robots_pos')
    failed_robot_id = request.args.get('failed_robot_id')

    # Run the function to recalculate the paths based on the input parameters
    return recalculate_paths(job_id, curr_robots_pos, failed_robot_id) # Return the json result of the recalculation

"""
This function recalculates the paths based on the current positions and where the failed robot starts back at the origin.
"""
def recalculate_paths(job_id, curr_robots_pos, failed_robot):
    # Define the cache folder path relative to the current directory
    cache_folder_path = os.path.join(os.getcwd(), 'cache')

    # Check if the job folder exists
    job_folder_path = os.path.join(cache_folder_path, job_id)
    if not os.path.exists(job_folder_path):
        print(f"Job folder does not exist: {job_folder_path}.")
        return

    # Read the result.json file to retrieve the parameters
    result_file_path = os.path.join(job_folder_path, 'result.json')
    if not os.path.exists(result_file_path):
        print(f"Result file does not exist: {result_file_path}.")
        return

    # Parse the JSON content to extract the parameters
    with open(result_file_path, 'r') as file:
        result_data = json.load(file)
        params = result_data.get('params')
        previous_robot_node_path = result_data.get('robot_node_path')

    # Use the extracted parameters for recalculation
    k = params.get('k')
    q_k = params.get('q_k')
    n_a = params.get('n_a')
    rp = params.get('rp')
    l = params.get('l')
    d = params.get('d')


    # Lets say that the we use 6_0.5_4 for the parameters (this will be the edges)
    #[[16, 17, 10, 14, 9], [16, 17, 0, 1, 3, 2], [16, 17, 4, 8, 12, 13, 5], [16, 17, 7], [16, 17, 15], [16, 17, 6, 11]]

    # Example current robot positions
    ex_robot_positions = [10, 16, 16, 16, 16, 16] # 1 index based
    ex_failed_robot_id = 1

    new_robot_paths = recalcRobotPaths(previous_robot_node_path, ex_robot_positions, rp, ex_failed_robot_id)

    # visualize the new paths and save the graph to the cache
    visualize_recalculated_paths(new_robot_paths, int(k), int(n_a), saveGraphPath(job_id, 'recalculated_paths'))

    result_data = {'job_id': job_id, 'params': {'k': k, 'q_k': q_k, 'n_a': n_a, 'rp': rp, 'l': l, 'd': d, 'mode': 'h'}, 'robot_node_path': new_robot_paths, 'robot_world_path': convertToWorldPath(new_robot_paths)}
    with open(os.path.join(job_folder_path, 'recalculated_paths.json'), 'w') as file:
        json.dump(result_data, file)
        print("Result and parameters saved to recalculated_paths.json file.")
    return result_data  # Return the content of the JSON file


"""
This function takes in the previous_node_path and the current_robot_positions and recalculates the paths based on the new positions.
The robots start where they currently are. The failed robot starts back at the depot. All the robots recalculate their paths based on the new positions
and the failed robot's new position. They need even frequency coverage to match the redundancy parameter.
"""
def recalcRobotPaths(previous_node_path, current_robot_positions, rp, failed_robot_id):
    new_node_paths = [[16, 17, 10, 14, 9], [16, 17, 0, 1, 3, 2], [16, 17, 4, 8, 12, 13, 5], [16, 17, 7], [16, 17, 15], [16, 17, 6, 11]]
    # Calculate visit counts for each node
    node_visit_counts = calculate_visit_counts(current_robot_positions, previous_node_path)

    # Start the failed robot back at the depot
    # new_node_paths[failed_robot_id] = [[0]]

    # Recalculate paths for all robots (making all considerations but prioritizing matching the redundancy parameter) TODO: Implement this


    # prioritize fuel capacity

    return new_node_paths

"""
This function calculates the visit counts for each node based on the current robot positions and the previous paths.
"""
def calculate_visit_counts(current_robot_positions, robot_previous_paths):
    # Find the largest node number
    max_node = max(max(path) for path in robot_previous_paths)

    # Count visits for each node
    node_visit_counts = {}
    for robot_position, previous_path in zip(current_robot_positions, robot_previous_paths):
        node_visit_counts[robot_position] = node_visit_counts.get(robot_position, 0) + 1
        for node in previous_path:
            node_visit_counts[node] = node_visit_counts.get(node, 0) + 1

    # Ensure that the largest node has the same number of visits as the largest node - 1
    max_visits = node_visit_counts.get(max_node, 0)
    max_minus_1_visits = node_visit_counts.get(max_node - 1, 0)
    node_visit_counts[max_node] = node_visit_counts[max_node - 1] = max(max_visits, max_minus_1_visits)

    print("Node visit counts:", node_visit_counts)
    return node_visit_counts


if __name__ == '__main__':
    print("Waiting for a request...")  # Added waiting message
    app.run(host='127.0.0.1', port=5000, debug=False, use_reloader=False)
