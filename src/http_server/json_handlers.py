"""
Author: Samara Holmes
Date: 2/17/2024

This program defines the methods relating to returning json data to the client and saving json data to the cache folder
"""
import os

from flask import json


def saveResultsToCache(job_id, result_data, filename):
    # Get the current working directory
    current_dir = os.getcwd()

    # Define the cache folder path relative to the current directory
    cache_folder_path = os.path.join(current_dir, 'cache')

    # Create job folder
    job_folder_path = os.path.join(cache_folder_path, job_id)
    if not os.path.exists(job_folder_path):
        os.makedirs(job_folder_path)

    with open(os.path.join(job_folder_path, filename), 'w') as file:
        json.dump(result_data, file)
        print("Result and parameters saved to ", filename)


def saveGraphPath(job_id, title):
    # Get the current working directory
    current_dir = os.getcwd()
    # Define the cache folder path relative to the current directory
    cache_folder_path = os.path.join(current_dir, 'cache')
    # Check if folder with job ID exists in the cache folder
    job_folder_path = os.path.join(cache_folder_path, job_id)
    # Define the folder name for saving visualizations
    visualization_folder = 'graphs'

    # Construct the folder path for visualizations
    visualization_folder_path = os.path.join(job_folder_path, visualization_folder)

    # Create the directory if it doesn't exist
    os.makedirs(visualization_folder_path, exist_ok=True)

    # Construct the save path for the visualization under the 'graphs' directory
    return os.path.join(visualization_folder_path, title)


def export_world_paths(result):
    # Extract world paths from the result
    num_robots = result.get('params').get('k')
    n_a = result.get('params').get('n_a')
    fcr = result.get('params').get('fcr')
    mode = result.get('params').get('mode')
    if n_a == 100:
        radius = 0.01
    elif n_a == 20:
        radius = 0.05
    elif n_a == 10:
        radius = 0.1
    else:
        radius = None

    robot_world_path = result.get('robot_world_path')
    # Check if robot_world_path is not None and is a list
    if robot_world_path is not None and isinstance(robot_world_path, list):
        for i, world_path in enumerate(robot_world_path):
            # Define the file path based on the index i
            file_path = f"world_paths_graph_{i+1}.json"
            # Write world path to JSON file
            with open(file_path, 'w') as json_file:
                json.dump(world_path, json_file)
            print(f"World path {i+1} exported to {file_path}")
    else:
        print("No world paths to export")