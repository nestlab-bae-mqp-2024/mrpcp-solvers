"""
Author: Samara Holmes
Date: 2/17/2024

This program defines the methods that are used to visualize the overall graphs needed for BAE Systems
"""
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

# 1. Define the robot world paths to test
robot_world_path = None
# 2. Pseudo-Simulate and create the 2d histogram of the normalized visitation frequency
all_world_points = pseudo_simulate(robot_world_path, metadata)
metadata = visualize_visitation_frequency(all_world_points)
discretized = discretize_world_points(all_world_points, metadata)
# 3. percent coverage over time
metadata = visualize_coverage_stepwise(discretized, metadata)
# TODO: Plot 1 is percent coverage vs number of robots for H1, H2, and MILP
# For this, the number of nodes per axis will be the same (nodes = ceil(1/sqrt(2)*r)^2
t = 30  # Define the time to simulate
dt = 0.1    # Define the time step
v = 0.5  # robot speed
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

# TODO: Plot 2 is percent coverage vs robot speed for H1, H2, and MILP
v_list = [0.5, 2.0, 5.0]   # Define different robot speeds m/s
plt.xlabel('Number of Robots')
plt.ylabel('Percent Coverage')
plt.title('Percent Coverage vs. Number of Robots (Different Robot Speeds)')
plt.legend()
plt.grid(True)
plt.show()

# TODO: Plot 3 is percent coverage vs robot field of view (number of nodes?) for H1, H2, and MILP
# For this, speed will be constant 0.5 m/s
plt.xlabel('Number of Robots')
plt.ylabel('Percent Coverage')
plt.title('Percent Coverage vs. Number of Robots (Different Surveillance Radii)')
plt.legend()
plt.grid(True)
plt.show()
# TODO: Plot 4 is percent coverage vs L_min ratio = 1.5, 5, 10 for H1, H2
plt.xlabel('Number of Robots')
plt.ylabel('Percent Coverage')
plt.title('Percent Coverage vs. Number of Robots (Different Fuel Capacities')
plt.legend()
plt.grid(True)
plt.show()
