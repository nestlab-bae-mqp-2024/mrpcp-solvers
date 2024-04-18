"""
Author: Samara Holmes
Date: 2/17/2024

This program defines the methods that are used to visualize the overall graphs needed for BAE Systems
"""
import json
from math import sqrt

from matplotlib import pyplot as plt
from matplotlib.ticker import ScalarFormatter


def plot_coverage_vs_radius():
    # Load data from JSON
    with open('world_paths_graph1.json', 'r') as json_file:
        data = json.load(json_file)

    radius_list = data["parameters"]["speed"]
    modes = data["parameters"]["mode"]

    coverage_data = {mode: {radius: [] for radius in radius_list} for mode in modes}

    # Run simulations for each combination of mode, speed, and num_robots
    for entry in data["data"]:
        mode = entry["mode"]
        speed = entry["speed"]
        radius_f = entry["radius"]
        n_a = 1/radius_f
        radius_m = 3000 / (n_a * sqrt(2))
        coverage = entry["percent_coverage"]
        coverage_data[mode][speed].append((radius_m, coverage))

    # Plot percent coverage vs. speed for each mode and number of robots
    plt.figure()
    for mode, mode_data in coverage_data.items():
        for speed, data in mode_data.items():
            radius = [entry[0] for entry in data]
            coverage = [entry[1] for entry in data]
            plt.plot(radius, coverage, marker='o', label=f'{mode.upper()}, Speed: {speed*100} m/s')

    plt.ylim(0, 100)
    plt.text(0.5, 0.85, 'Constants: k=8, fcr=1.5, rp=3, ssd=3km', fontsize=8, transform=plt.gcf().transFigure)
    plt.xlabel('Surveillance Radius (m)')
    plt.ylabel('Field Coverage (%)')
    plt.title('Field Coverage vs. Surveillance Radius (Varying Speed)')
    plt.legend()
    plt.grid(True)
    plt.savefig('coverage_vs_radius.png')
    plt.show()


# TODO: Plot 2 is percent coverage vs robot speed for H1, H2
# Keep constant: speed = 0.2 m/s, time = 30s, time step = 0.1s, vary number of robots and speed -> three lines for different number of robots (8, 64, 1024)
# speed is X axis
def plot_coverage_vs_speed():
    # Load data from JSON
    with open('world_paths_graph2.json', 'r') as json_file:
        data = json.load(json_file)

    # Access parameters and data
    num_robots_list = data["parameters"]["num_robots"]
    speed_list = data["parameters"]["speed"]
    modes = data["parameters"]["mode"]

    # Initialize dictionary to store coverage data for each mode and number of robots
    coverage_data = {mode: {num_robots: [] for num_robots in num_robots_list} for mode in modes}

    # Run simulations for each combination of mode, speed, and num_robots
    for entry in data["data"]:
        mode = entry["mode"]
        speed = entry["speed"]
        num_robots = entry["num_robots"]
        coverage = entry["percent_coverage"]
        coverage_data[mode][num_robots].append((speed, coverage))

    # Plot percent coverage vs. speed for each mode and number of robots
    plt.figure()
    for mode, mode_data in coverage_data.items():
        for num_robots, data in mode_data.items():
            speed = [entry[0]*100 for entry in data]
            coverage = [entry[1] for entry in data]
            line_style = '-' if mode != "h1" or num_robots != 1024 else ':'
            plt.plot(speed, coverage, linestyle=line_style, marker='o', label=f'{mode.upper()}, Robots: {num_robots}')

    plt.ylim(0, 100)
    plt.text(0.15, 0.15, 'Constants: radius=212.13m, fcr=1.5, rp=3, ssd=3km', fontsize=8, transform=plt.gcf().transFigure)
    plt.xlabel('Robot Speed (m/s)')
    plt.ylabel('Field Coverage (%)')
    plt.title('Field Coverage vs. Robot Speed (Varying Number of Robots)')
    plt.legend()
    plt.grid(True)
    plt.savefig('coverage_vs_speed.png')
    # plt.show()


# TODO: Plot 3 is percent coverage vs number of robots for H1, H2, varing surveillance
# For this, speed will be constant 0.5 m/s -> three lines for different surveillance radii
#  (0.01, 0.05, 0.1) (3 values in total) -> Corresponds to nodes/axis of 100, 20, 10
# X axis (8, 16, 32, 64, 128, 256, 512, 1024) robots
def plot_coverage_vs_num_robots():
    with open('world_paths_graph3.json', 'r') as json_file:
        data = json.load(json_file)

    # Access parameters and data
    radius_list = data["parameters"]["radius"]
    modes = data["parameters"]["mode"]

    # Initialize dictionary to store coverage data for each mode and radius
    coverage_data = {mode: {radius: [] for radius in radius_list} for mode in modes}

    # Run simulations for each combination of mode, radius, and num_robots
    for entry in data["data"]:
        mode = entry["mode"]
        radius_f = entry["radius"]
        num_robots = entry["num_robots"]
        coverage = entry["percent_coverage"]
        coverage_data[mode][radius_f].append((num_robots, coverage))

    # Plot percent coverage vs. number of robots for each mode and surveillance radius
    plt.figure()
    for mode, mode_data in coverage_data.items():
        for radius_f, data in mode_data.items():
            n_a = 1/radius_f
            radius_m = 3000 / (n_a * sqrt(2))
            num_robots = [entry[0] for entry in data]
            coverage = [entry[1] for entry in data]
            plt.plot(num_robots, coverage, marker='o', label=f'{mode.upper()}, Surveillance Radius: {radius_m:.2f} m')

    plt.xscale('log', base=2)
    plt.gca().xaxis.set_major_formatter(ScalarFormatter())

    plt.ylim(0, 100)
    plt.text(0.45, 0.43, 'Constants: fcr=1.5, rp=3, ssd=3km, v=20m/s', fontsize=8, transform=plt.gcf().transFigure)

    plt.xlabel('Number of Robots')
    plt.ylabel('Field Coverage (%)')
    plt.title('Field Coverage vs. Number of Robots (Varying Surveillance Radius)')
    plt.legend()
    plt.grid(True)
    plt.savefig('coverage_vs_num_robots.png')
    # plt.show()


# TODO: Plot 4 is percent coverage vs number of robots = 1.5, 5, 10 for H1, H2 varing L_min
# For this, speed will be constant 0.5 m/s, number of robots = 64 -> three lines for different L_min ratios (1.5, 3, 4.5)
# X axis is number of robots (8, 16, 32, 64, 128, 256, 512, 1024) robots
def plot_coverage_vs_fuel_capacity():
    with open('world_paths_graph4-n_a30.json', 'r') as json_file:
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
            line_style = '-' if mode != "h2" or L_min != 3 else ':'  # Use dotted line for specific mode and L_min because it overlaps
            plt.plot(num_robots, coverage, linestyle=line_style, marker='o', label=f'{mode.upper()}, Fuel: {L_min*8.48 :.2f} km')

    plt.xscale('log', base=2)
    plt.gca().xaxis.set_major_formatter(ScalarFormatter())
    # plt.xlim(0, 1024)
    plt.ylim(0, 100)

    plt.text(0.13, 0.15, 'Constants: radius=70.71m, rp=3, ssd=3km, v=20m/s', fontsize=8, transform=plt.gcf().transFigure)
    plt.xlabel('Number of Robots')
    plt.ylabel('Field Coverage (%)')
    plt.title('Field Coverage vs. Number of Robots (Varying Fuel Capacities)')
    plt.legend()
    plt.grid(True)
    plt.savefig('coverage_vs_fuel_capacity.png')
    # plt.show()


if __name__ == "__main__":
    plot_coverage_vs_radius()
    plot_coverage_vs_speed()
    plot_coverage_vs_num_robots()
    plot_coverage_vs_fuel_capacity()
#%%


