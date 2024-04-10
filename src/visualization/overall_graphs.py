"""
Author: Samara Holmes
Date: 2/17/2024

This program defines the methods that are used to visualize the overall graphs needed for BAE Systems
"""
from matplotlib import pyplot as plt

from src.http_server.utils.visualize import visualize_coverage
from src.visualization.discretization import discretize_world_points
from src.visualization.pseudo_simulate import pseudo_simulate
from src.visualization.visitation_frequency import visualize_visitation_frequency

# 1. Define the robot world paths to test
robot_world_path = None
# 2. Pseudo-Simulate and create the 2d histogram of the normalized visitation frequency
all_world_points = pseudo_simulate(robot_world_path, v=1., t=30, dt=0.1)
metadata = visualize_visitation_frequency(all_world_points)
discretized = discretize_world_points(all_world_points, metadata)
# 3. percent coverage over time
metadata, avg_coverage = visualize_coverage(20, None, discretized, metadata)

# TODO: Plot 1 is percent coverage vs surveillance radius of robots for H1, H2
# Keep constant: robots = 8, 1.5 fcr, rp = 1 time = 30s, time step = 0.1s, vary speeds-> three lines for different speeds (0.1, 0.15, 0.2)
# X axis is radius
num_robots_list_h = [8, 64, 1024]   # Define different numbers of robots to test for H1 and H2
# Run simulations for each algorithm and each number of robots
percent_coverage_data = {
    'H1': [],
    'H2': [],
    'MILP': []
}
for num_robots in num_robots_list_h:
    # Simulate and get percent coverage for H1, H2, and MILP
    coverage_h1 = 0  # TODO: Replace with actual percent coverage for H1
    coverage_h2 = 0  # TODO: Replace with actual percent coverage for H2
    # Assuming you have the percent coverage data for each strategy
    percent_coverage_data['H1'].append(coverage_h1)
    percent_coverage_data['H2'].append(coverage_h2)
    # percent_coverage_data['MILP'].append(coverage_milp)

plt.xlabel('Number of Robots')
plt.ylabel('Percent Coverage')
plt.title('Percent Coverage vs. Number of Robots')
plt.legend()
plt.grid(True)
plt.show()

# TODO: Plot 2 is percent coverage vs robot speed for H1, H2
# Keep constant: speed = 0.2 m/s, time = 30s, time step = 0.1s, vary number of robots and speed -> three lines for different number of robots (8, 64, 1024)
# speed is X axis
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
plt.xlabel('Number of Robots')
plt.ylabel('Percent Coverage')
plt.title('Percent Coverage vs. Number of Robots (Different Surveillance Radii)')
plt.legend()
plt.grid(True)
plt.show()
# TODO: Plot 4 is percent coverage vs number of robots = 1.5, 5, 10 for H1, H2 varing L_min
# For this, speed will be constant 0.5 m/s, number of robots = 64 -> three lines for different L_min ratios (1.5, 3, 4.5)
# X axis is number of robots (8, 16, 32, 64, 128, 256, 512, 1024) robots
plt.xlabel('Number of Robots')
plt.ylabel('Percent Coverage')
plt.title('Percent Coverage vs. Number of Robots (Different Fuel Capacities')
plt.legend()
plt.grid(True)
plt.show()
