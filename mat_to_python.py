import pulp as lp
import networkx as nx
import matplotlib.pyplot as plt
import os


# Parameters and inputs
T = int(input("Enter number of target nodes = "))
D = int(input("Number of depots = "))
N = T + D
K = int(input("Number of robots = "))
L = int(input(f"What's the fuel capacity? Enter value between 1 to {2*T} "))
depots = list(range(T, T + D))

qk = [0.5] * K  # You can modify this as needed
Bk = [lp.LpVariable(f"Bk_{k}", lowBound=depots[0], upBound=depots[D-1]) for k in range(K)]

# Nodes, positions, distance, and fuel cost
xt = [lp.LpVariable(f"xt_{i}", lowBound=0) for i in range(T)]
yt = [lp.LpVariable(f"yt_{i}", lowBound=0) for i in range(T)]
xd = [lp.LpVariable(f"xd_{i}", lowBound=0) for i in range(D)]
yd = [lp.LpVariable(f"yd_{i}", lowBound=0) for i in range(D)]

x_pos = xt + xd
y_pos = yt + yd

e_dist = [[lp.LpVariable(f"e_dist_{i}_{j}", lowBound=0) for j in range(N)] for i in range(N)]
cij_per_robot = [e_dist[i][j] for i in range(N) for j in range(N)]

# Define the LP problem
problem = lp.LpProblem("MultiRobotCoverage", lp.LpMinimize)

# Decision variables
x_kij = [[[lp.LpVariable(f"x_kij_{k}_{i}_{j}", cat=lp.LpBinary) for j in range(N)] for i in range(N)] for k in range(K)]
p_kij = [[[lp.LpVariable(f"p_kij_{k}_{i}_{j}", cat=lp.LpInteger, lowBound=0, upBound=T) for j in range(N)] for i in range(N)] for k in range(K)]
ri = [lp.LpVariable(f"ri_{i}", cat=lp.LpInteger, lowBound=0, upBound=L) for i in range(T)]

# Objective function
problem += lp.lpSum([1] + [0] * ((N**2 * K) * 2 + T * K))

# Integer (bound) constraints
lb1 = [0] * (N**2 * K)
ub1 = [T] * (N**2 * K)
for k in range(K):
    for i in range(N):
        for j in range(T):
            ub1[j + (i * N) + (k * N**2)] = 1

lb2 = [0] * (N**2 * K)
ub2 = [T] * (N**2 * K)

lb3 = [0] * (T * K)
ub3 = [L] * (T * K)

# MATLAB code constraints

# Equation 10:
for k in range(K):
    for i in range(N):
        problem += lp.lpSum(x_kij[k][i][j] for j in range(N)) == 1
        problem += lp.lpSum(x_kij[k][i][j] for j in range(N)) == 1


# Solve the problem
problem.solve()

# Output the results
if lp.LpStatus[problem.status] == "Optimal":
    # Access variable values
    X = [lp.value(var) for var in problem.variables()]
    # Perform visualization if needed
else:
    print("No optimal solution found.")

# Assuming you have already solved the LP problem
# Extract the values of the decision variables x_kij
x_values = [[[lp.value(x_kij[k][i][j]) for j in range(N)] for i in range(N)] for k in range(K)]

# Define an adjacency matrix (initialized with zeros)
adjacency_matrix = [[0 for _ in range(N)] for _ in range(N)]

# Populate the adjacency matrix based on x_kij values
for k in range(K):
    for i in range(N):
        for j in range(N):
            if x_values[k][i][j] == 1:
                # There is a path from node i to node j for robot k
                adjacency_matrix[i][j] = 1


def map(A, robot, T, D, K, N, x_pos, y_pos, L):
    # Create the data directory if it doesn't exist
    data_dir = 'data'
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    # Save the plot to the data directory
    filename = f"{data_dir}/{T}_{D}_{K}_{L}_Robot{robot}.jpeg"
    plt.savefig(filename)

    # Create a directed graph from the adjacency matrix A
    G = nx.DiGraph()
    for i in range(N):
        for j in range(N):
            if A[i][j] == 1:
                G.add_edge(i, j)

    # Create a figure
    plt.figure()

    # Position nodes at specified coordinates
    pos = {i: (x_pos[i] if lp.value(x_pos[i]) is not None else 0,
               y_pos[i] if lp.value(y_pos[i]) is not None else 0) for i in range(N)}

    # Draw nodes and edges
    nx.draw(G, pos, with_labels=True, node_size=200, node_color="lightblue", font_size=10)

    # Highlight targets and depots
    targets = range(T)
    depots = range(T, N)

    nx.draw_networkx_nodes(G, pos, nodelist=targets, node_color='g', node_size=300)
    nx.draw_networkx_nodes(G, pos, nodelist=depots, node_color='r', node_size=300)

    # Set the title
    plt.title(f"Path of Robot {robot}")

    # Save the plot to a file
    filename = f"data/{T}_{D}_{K}_{L}_Robot{robot}.jpeg"
    plt.savefig(filename)

    # Display the plot
    plt.show()



# Example usage
# Replace the arguments with actual data
# map(A, robot, T, D, K, N, x_pos, y_pos, L)
# A = [[0, 1, 0], [0, 0, 1], [0, 0, 0]]
# Call the map function
robot_number = 4
map(adjacency_matrix, robot_number, T, D, K, N, x_pos, y_pos, L)
