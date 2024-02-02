from flask import Flask, request
from serverComms.mrpcp import *
app = Flask(__name__)

def milp_solver_function(k, q_k, n):
    print(f"Running MILP solver function with parameters: k={k}, q_k={q_k}, n={n}")
    solve_milp_with_optimizations(k, q_k, n)
    print("MILP solver function completed.")

@app.route('/solve', methods=['POST'])
def solve_endpoint():
    content = request.get_json()
    if content and all(key in content for key in ['k', 'q_k', 'n']):
        # Extract parameters from the JSON payload
        k = content['k']
        q_k = content['q_k']
        n = content['n']

        # Run your MILP solver function with parameters
        milp_solver_function(k, q_k, n)
        return 'MILP solver function requested.', 200
    else:
        return 'Invalid request or missing parameters.', 400

if __name__ == '__main__':
    print("Waiting for a request...")  # Added waiting message
    app.run(host='127.0.0.1', port=5000)
