"""
Author: Samara Holmes
Date: 2/17/2024

This program defines the methods that are used to visualize the overall graphs needed for BAE Systems
"""
import json

from matplotlib import pyplot as plt

from src.http_server.utils.visualize import visualize_coverage_stepwise
from src.visualization.discretization import discretize_world_points
from src.visualization.pseudo_simulate import pseudo_simulate
from src.visualization.visitation_frequency import visualize_visitation_frequency

metadata = {
    "v": 5.,
    "t": 100.,
    "dt": 0.1,
    "lookback_time": 5.
}

# TODO: Plot 1 is percent coverage vs Surveillance Radius for H1, H2, and MILP
# For this, the number of nodes per axis will be the same (nodes = ceil(1/sqrt(2)*r)^2
def plot_coverage_vs_radius():
    # Load data from JSON
    with open('world_paths_graph1.json', 'r') as json_file:
        data = json.load(json_file)

    # Access parameters and data
    speeds = data["parameters"]["speeds"]
    radius = data["parameters"]["radius"]
    modes = data["parameters"]["mode"]

    # Initialize lists to store coverage for each combination
    coverage_data = {mode: [] for mode in modes}

    # Run simulations for each combination of speed, radius, and mode
    for mode in modes:
        for entry in data["data"]:
            if entry["mode"] == mode:
                world_coords = pseudo_simulate(entry["world_coords"], metadata)
                metadata = visualize_visitation_frequency(world_coords)
                discretized = discretize_world_points(world_coords, metadata)
                metadata = visualize_coverage_stepwise(discretized, metadata)
                coverage_data[mode].append((entry["speed"], entry["radius"], metadata["average_coverage"]))

    # Plot percent coverage vs. surveillance radius for each mode
    plt.figure()
    for mode, data in coverage_data.items():
        speeds = [entry[0] for entry in data]
        radius = [entry[1] for entry in data]
        coverage = [entry[2] for entry in data]
        plt.scatter(radius, coverage, label=mode)

    plt.xlabel('Surveillance Radius')
    plt.ylabel('Percent Coverage')
    plt.title('Percent Coverage vs. Surveillance Radius')
    plt.legend()
    plt.grid(True)
    plt.show()

# TODO: Plot 2 is percent coverage vs robot speed for H1, H2
# Keep constant: speed = 0.2 m/s, time = 30s, time step = 0.1s, vary number of robots and speed -> three lines for different number of robots (8, 64, 1024)
# speed is X axis
def plot_coverage_vs_speed():
    # Load data from JSON
    with open('world_paths_graph2.json', 'r') as json_file:
        data = json.load(json_file)

    # Access parameters and data
    speeds = data["parameters"]["speeds"]
    num_robots = data["parameters"]["num_robots"]
    modes = data["parameters"]["mode"]
    simulation_data = data["data"]

    # Initialize lists to store coverage for each combination
    coverage_data = {mode: [] for mode in modes}

    # Run simulations for each combination of speed, radius, and mode
    for mode in modes:
        for entry in data["data"]:
            if entry["mode"] == mode:
                world_coords = pseudo_simulate(entry["world_coords"], metadata)
                metadata = visualize_visitation_frequency(world_coords)
                discretized = discretize_world_points(world_coords, metadata)
                metadata = visualize_coverage_stepwise(discretized, metadata)
                coverage_data[mode].append((entry["speed"], entry["num_robots"], metadata["average_coverage"]))

    # Plot percent coverage vs. surveillance radius for each mode
    plt.figure()
    for mode, data in coverage_data.items():
        speeds = [entry[0] for entry in data]
        num_robots = [entry[1] for entry in data]
        coverage = [entry[2] for entry in data]
        plt.scatter(num_robots, coverage, label=mode)

    plt.xlabel('Number of Robots')
    plt.ylabel('Percent Coverage')
    plt.title('Percent Coverage vs. Number of Robots (Different Robot Speeds)')
    plt.legend()
    plt.grid(True)
    plt.show()

# TODO: Plot 3 is percent coverage vs number of robots for H1, H2, varing surveillance
# For this, speed will be constant 0.5 m/s -> three lines for different surveillance radii
#  (0.01, 0.05, 0.1) (3 values in total) -> Corresponds to nodes/axis of 100, 20, 10
# X axis (8, 16, 32, 64, 128, 256, 512, 1024) robots
def plot_coverage_vs_num_robots():
    with open('world_paths_graph3.json', 'r') as json_file:
        data = json.load(json_file)

    # Access parameters and data
    num_robots_list = data["parameters"]["num_robots"]
    radius_list = data["parameters"]["radius"]
    modes = data["parameters"]["mode"]
    simulation_data = data["data"]

    # Initialize dictionary to store coverage data for each mode and radius
    coverage_data = {mode: {radius: [] for radius in radius_list} for mode in modes}

    # Run simulations for each combination of mode, radius, and num_robots
    for entry in data["data"]:
        mode = entry["mode"]
        radius = entry["radius"]
        num_robots = entry["num_robots"]
        coverage = entry["percent_coverage"]
        coverage_data[mode][radius].append((num_robots, coverage))

    # Plot percent coverage vs. number of robots for each mode and surveillance radius
    plt.figure()
    for mode, mode_data in coverage_data.items():
        for radius, data in mode_data.items():
            num_robots = [entry[0] for entry in data]
            coverage = [entry[1] for entry in data]
            plt.plot(num_robots, coverage, label=f'{mode.upper()}, Surveillance Radius: {radius}')

    # Set x-axis limit to start at 8 robots
    plt.xlim(left=8)

    plt.xlabel('Number of Robots')
    plt.ylabel('Percent Coverage')
    plt.title('Percent Coverage vs. Number of Robots')
    plt.legend()
    plt.grid(True)
    plt.show()


# TODO: Plot 4 is percent coverage vs number of robots = 1.5, 5, 10 for H1, H2 varing L_min
# For this, speed will be constant 0.5 m/s, number of robots = 64 -> three lines for different L_min ratios (1.5, 3, 4.5)
# X axis is number of robots (8, 16, 32, 64, 128, 256, 512, 1024) robots
def plot_coverage_vs_fuel_capacity():
    with open('world_paths_graph4.json', 'r') as json_file:
        data = json.load(json_file)

    parameters = data["parameters"]
    modes = parameters["mode"]

    plt.figure()

    # Initialize dictionary to store coverage data for each mode
    coverage_data = {mode: {} for mode in modes}

    for entry in data["data"]:
        mode = entry["mode"]
        L_min = entry["L_min"]
        num_robots = entry["num_robots"]
        coverage = entry["percent_coverage"]

        # Check if L_min key exists in the dictionary, if not, create it
        if L_min not in coverage_data[mode]:
            coverage_data[mode][L_min] = []

        # Append coverage data for each L_min value
        coverage_data[mode][L_min].append((num_robots, coverage))

    # Plot percent coverage vs. fuel capacity for each mode
    for mode, fuel_data in coverage_data.items():
        for L_min, data_points in fuel_data.items():
            num_robots = [entry[0] for entry in data_points]
            coverage = [entry[1] for entry in data_points]
            plt.plot(num_robots, coverage, label=f'L_min={L_min}, {mode.upper()}')


    # Set x-axis limit to start at 8 robots
    plt.xlim(left=8)

    plt.xlabel('Number of Robots')
    plt.ylabel('Percent Coverage')
    plt.title('Percent Coverage vs. Number of Robots (Different Fuel Capacities)')
    plt.legend()
    plt.grid(True)
    plt.show()

# plot_coverage_vs_num_robots()
plot_coverage_vs_fuel_capacity()