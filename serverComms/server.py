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
from serverComms.mrpcp import solve_milp_with_optimizations
import uuid
import requests
import os
import json

app = Flask(__name__)

# Get the current working directory
current_dir = os.getcwd()

# Define the cache folder path relative to the current directory
cache_folder_path = os.path.join(current_dir, 'cache')

@app.route('/solve', methods=['POST'])
def solve_endpoint():
    k = request.args.get('k')
    q_k = request.args.get('q_k')
    n = request.args.get('n')

    # Generate job ID based on parameters
    job_id = f"{k}_{q_k}_{n}"

    # Check if folder with job ID exists in the cache folder
    job_folder_path = os.path.join(cache_folder_path, job_id)
    if os.path.exists(job_folder_path):
        print(f"Job folder exists: {job_folder_path}. Returning the content of the JSON file...")
        # If folder exists, read JSON file and return its content
        with open(os.path.join(job_folder_path, 'result.json'), 'r') as file:
            result = json.load(file)
        print(f"Contents of the JSON file: {result}")  # Print contents of JSON file
        return jsonify(result), 200
    else:
        print(f"Job folder does not exist: {job_folder_path}. Starting MILP solver function with parameters k={k}, q_k={q_k}, n={n}...")
        # Run MILP solver function with parameters on a separate thread
        milp_thread = threading.Thread(target=run_milp_solver, args=(k, q_k, n, job_id, cache_folder_path))
        milp_thread.start()
        return 'MILP solver function started.', 200

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
from serverComms.mrpcp import solve_milp_with_optimizations
import uuid
import requests
import os
import json

app = Flask(__name__)

# Get the current working directory
current_dir = os.getcwd()

# Define the cache folder path relative to the current directory
cache_folder_path = os.path.join(current_dir, 'cache')

@app.route('/solve', methods=['POST'])
def solve_endpoint():
    k = request.args.get('k')
    q_k = request.args.get('q_k')
    n = request.args.get('n')

    # Generate job ID based on parameters
    job_id = f"{k}_{q_k}_{n}"

    # Check if folder with job ID exists in the cache folder
    job_folder_path = os.path.join(cache_folder_path, job_id)
    if os.path.exists(job_folder_path):
        print(f"Job folder exists: {job_folder_path}. Returning the content of the JSON file...")
        # If folder exists, read JSON file and return its content
        with open(os.path.join(job_folder_path, 'result.json'), 'r') as file:
            result = json.load(file)
        print(f"Contents of the JSON file: {result}")  # Print contents of JSON file
        return jsonify(result), 200
    else:
        print(f"Job folder does not exist: {job_folder_path}. Starting MILP solver function with parameters k={k}, q_k={q_k}, n={n}...")
        # Run MILP solver function with parameters on a separate thread
        milp_thread = threading.Thread(target=run_milp_solver, args=(k, q_k, n, job_id, cache_folder_path))
        milp_thread.start()
        return 'MILP solver function started.', 200

def run_milp_solver(k, q_k, n, job_id, cache_folder_path):
    try:
        # Run MILP solver function with parameters
        print(f"Running MILP solver function with parameters: k={k}, q_k={q_k}, n={n}")
        edges = solve_milp_with_optimizations(int(k), float(q_k), int(n))
        print(f"MILP solver function completed with parameters: k={k}, q_k={q_k}, n={n}")

        if edges:
            # Convert edges to a serializable format
            serializable_edges = [[int(edge) for edge in path] for path in edges]

            # Create job folder
            job_folder_path = os.path.join(cache_folder_path, job_id)
            if not os.path.exists(job_folder_path):
                os.makedirs(job_folder_path)

            # Save result in a JSON file within the cache folder
            with open(os.path.join(job_folder_path, 'result.json'), 'w') as file:
                json.dump({'edges': serializable_edges}, file)
                print("Edges saved to result.json file.")

    except Exception as e:
        print(f"Error occurred during MILP solving: {e}")

if __name__ == '__main__':
    print("Waiting for a request...")  # Added waiting message
    app.run(host='127.0.0.1', port=5000, debug=False, use_reloader=False)
