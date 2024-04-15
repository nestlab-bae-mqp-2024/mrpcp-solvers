import os

from flask import json, jsonify
import numbers
from src.visualization.pseudo_simulate import pseudo_simulate
from src.http_server.utils.visualize import visualize_coverage_stepwise_no_plotting

from src.http_server.server import run_solver
from src.visualization.discretization import discretize_world_points
from tqdm import tqdm


def adding_to_json_4():
    current_dir = os.getcwd()

    with open("src/visualization/world_paths_graph4.json", 'r') as file:
        formatted_results = json.load(file)

    tests_to_be_run = formatted_results.get('data')

    n_a = 10
    rp = 2
    ssd = 3

    for test in tqdm(tests_to_be_run):
        k = test.get('num_robots')
        fcr = test.get('L_min')
        mode = test.get('mode')

        # Generate job ID based on parameters
        job_id = f"{k}_{n_a}_{ssd}_{fcr}_{rp}_{mode}"

        print(job_id)
        # Check if folder with job ID exists in the cache folder
        job_folder_path = os.path.join("cache", job_id)

        if not os.path.exists(os.path.join(job_folder_path, 'result.json')):
            print("Test:"+job_id+ " does not exist! Attempting to create ...")

            run_solver(k, n_a, ssd, fcr, rp, mode, job_id, skip_vis=True)

        with open(os.path.join(job_folder_path, 'result.json'), 'r') as file:
            single_run = json.load(file)

        metadata = {"k": k,
                    "n_a": n_a,
                    "ssd": ssd,
                    "fcr": fcr,
                    "rp": rp,
                    "mode": mode,
                    "v": 0.2,
                    "t": 300.,
                    "dt": 0.1,
                    "lookback_time": 30.}

        if int(test.get('percent_coverage')) == 0:
            all_world_points = pseudo_simulate(single_run.get('robot_world_path'), metadata)

            discretized = discretize_world_points(all_world_points, metadata)

            avg_covg = visualize_coverage_stepwise_no_plotting(discretized, metadata)

            test['percent_coverage'] = avg_covg

            print("%coverage for: ", job_id, "is:" , avg_covg)

            with open("src/visualization/world_paths_graph4.json", 'w') as file:
                json.dump(formatted_results, file, indent=4)


def adding_to_json_3():
    with open("src/visualization/world_paths_graph3.json", 'r') as file:
        formatted_results = json.load(file)

    tests_to_be_run = formatted_results.get('data')

    fcr = 1.5
    rp = 2
    ssd = 3

    for test in tqdm(tests_to_be_run):
        k = test.get('num_robots')
        fov = test.get('radius')
        if fov == 0.01:
            n_a = 50
        elif fov == 0.05:
            n_a = 20
        elif fov == 0.1:
            n_a = 10
        else:
            n_a = None

        mode = test.get('mode')

        # Generate job ID based on parameters
        job_id = f"{k}_{n_a}_{ssd}_{fcr}_{rp}_{mode}"

        print(job_id)
        # Check if folder with job ID exists in the cache folder
        job_folder_path = os.path.join("cache", job_id)

        if not os.path.exists(os.path.join(job_folder_path, 'result.json')):
            print("Test:"+job_id+ " does not exist! Attempting to create ...")

            run_solver(k, n_a, ssd, fcr, rp, mode, job_id, skip_vis=True)

        with open(os.path.join(job_folder_path, 'result.json'), 'r') as file:
            single_run = json.load(file)

        metadata = {"k": k,
                    "n_a": n_a,
                    "ssd": ssd,
                    "fcr": fcr,
                    "rp": rp,
                    "mode": mode,
                    "v": 0.2,
                    "t": 300.,
                    "dt": 0.1,
                    "lookback_time": 30.}


        if int(test.get('percent_coverage')) == 0:
            print("pseudo simulating ...")
            all_world_points = pseudo_simulate(single_run.get('robot_world_path'), metadata)

            print("dicretizing ...")
            discretized = discretize_world_points(all_world_points, metadata)

            print("calculating percent coverage ...")
            avg_covg = visualize_coverage_stepwise_no_plotting(discretized, metadata)

            test['percent_coverage'] = avg_covg

            print("%coverage for: ", job_id, "is:" , avg_covg)

            with open("src/visualization/world_paths_graph3.json", 'w') as file:
                json.dump(formatted_results, file, indent=4)

        else:
            with open("src/visualization/world_paths_graph3.json", 'w') as file:
                json.dump(formatted_results, file, indent=4)
if __name__ == "__main__":
    adding_to_json_4()
