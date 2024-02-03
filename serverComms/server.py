from flask import Flask, request, jsonify
from serverComms.mrpcp import solve_milp_with_optimizations
import uuid
import requests

app = Flask(__name__)

available_jobs = {}

def milp_solver_function(k, q_k, n):
    print(f"Running MILP solver function with parameters: k={k}, q_k={q_k}, n={n}")
    k = int(k) # Convert k to an integer
    job_id = str(uuid.uuid4()) # Generate a random jobID

    # Run MILP solver function with parameters
    solve_milp_with_optimizations(k, q_k, n)
    print("MILP solver function completed.")

    # Add the jobID to the available jobs dictionary
    available_jobs[job_id] = {'status': 'available'}
    print(f"JobID {job_id} added to available jobs.")

def send_job_to_available_jobs(job_id):
    available_jobs_url = 'http://127.0.0.1:5000/available_jobs'
    data = {'job_id': job_id}
    response = requests.post(available_jobs_url, json=data)

    if response.status_code == 200:
        print('Job sent to /available_jobs successfully.')
    else:
        print(f'Failed to send job to /available_jobs. Status code: {response.status_code}')

def send_edges_to_solution(edges):
    get_solution_url = 'http://127.0.0.1:5000/get_solution'

    data = {'edges': edges}
    response = requests.post(get_solution_url, json=data)

    if response.status_code == 200:
        print('Edges sent successfully to /get_solution endpoint.')
    else:
        print(f'Sending edges failed with status code: {response.status_code}')

@app.route('/solve', methods=['POST'])
def solve_endpoint():
    content = request.get_json()
    if content and all(key in content for key in ['k', 'q_k', 'n']):
        # Extract parameters from the JSON payload
        k = content['k']
        q_k = content['q_k']
        n = content['n']

        # Run MILP solver function with parameters
        milp_solver_function(k, q_k, n)
        return 'MILP solver function requested.', 200
    else:
        return 'Invalid request or missing parameters.', 400

@app.route('/available_jobs', methods=['GET'])
def get_available_jobs():
    return jsonify({'jobs': available_jobs})

@app.route('/available_jobs', methods=['POST'])
def post_available_jobs():
    content = request.get_json()
    if 'jobs' in content:
        global available_jobs
        available_jobs = content['jobs']
        return 'Available jobs updated successfully.', 200
    else:
        return 'Invalid request or missing jobs.', 400

@app.route('/get_solution', methods=['POST'])
def get_solution_endpoint():
    content = request.get_json()
    if 'edges' in content:
        edges = content['edges']
        # Process the received edges as needed
        print(f'Received edges: {edges}')
        return 'Edges received successfully.', 200
    else:
        return 'Invalid request or missing edges.', 400

if __name__ == '__main__':
    print("Waiting for a request...")  # Added waiting message
    app.run(host='127.0.0.1', port=5000, debug=False, use_reloader=False)