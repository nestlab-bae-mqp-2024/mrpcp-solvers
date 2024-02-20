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

