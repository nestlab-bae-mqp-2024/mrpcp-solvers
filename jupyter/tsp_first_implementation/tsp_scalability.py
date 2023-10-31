from matplotlib import pyplot as plt
from pprint import pp
import numpy as np
import gurobipy as gp
from gurobipy import GRB
from itertools import combinations
import itertools
import time

# Callback - use lazy constraints to eliminate sub-tours

def subtourelim(model, where):
    # print(where == GRB.Callback.MIPSOL)
    if where == GRB.Callback.MIPSOL:
        # make a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)
        selected = np.transpose(np.where(vals > 0.5))
        # find the shortest cycle in the selected edge list
        min_tour, _ = subtour(selected)
        # add subtour elimination constr. for every pair of cities in subtour
        # if len(min_tour) == 1:
        #     try:
        #         model.cbLazy(gp.quicksum(model._vars[min_tour[0][0], min_tour[0][1], min_tour[0][0], min_tour[0][1]]) <= 0)
        #     except Exception as e:
        #         print(f"...: {min_tour}")
        #         print(e)
        if len(min_tour) < n * n:
            model.cbLazy(gp.quicksum(model._vars[n1[0], n1[1], n2[0], n2[1]].item() for n1, n2 in combinations(min_tour, 2)) <= len(min_tour)-1)

# Given a list of edges, find the shortest subtour

def subtour(edges):
    unvisited = [(i, j) for i in range(n) for j in range(n)]
    cycle = range(n*n) # Dummy - guaranteed to be replaced
    all_cycles = []
    while unvisited:  # true if list is non-empty
        thiscycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            thiscycle.append(current)
            unvisited.remove(current)
            neighbors = []
            for i1, j1, i2, j2 in edges:
                if (i1, j1) == current and (i2, j2) in unvisited:
                    neighbors.append((i2, j2))
                elif (i2, j2) == current and (i1, j1) in unvisited:
                    neighbors.append((i1, j1))
        if len(thiscycle) <= len(cycle):
            cycle = thiscycle # New shortest subtour
            all_cycles.append(cycle)
    return cycle, all_cycles


if __name__ == "__main__":
    # Chose the number of nodes in an axis
    timings = []
    n = 1
    while True:
        start_of_timing = time.time()
        n += 1
        # Create a uniform (n, n, 2) numpy grid for MAXIMUM SPEED
        grid = np.mgrid[-1:1:n*1j, -1.:1:n*1j]
        grid = grid.reshape(grid.shape + (1,))
        grid = np.concatenate((grid[0], grid[1]), axis=2)
        # Sanity check
        # pp(grid)
        # pp(grid[:, :, 0].reshape(-1))
        # pp(grid[:, :, 1].reshape(-1))
        
        # Graphical sanity check
        # plt.figure()
        # plt.scatter(grid[:, :, 0], grid[:, :, 1], s=2)
        # plt.grid()
        # plt.show()
        
        
        m = gp.Model()
        
        # Calculate d_{ij} (d[i1, j1, i2, j2] is the cost from node (i1, j1) to (i2, j2))
        coeffs = np.zeros((n,n,n,n))
        for i1, j1, i2, j2 in itertools.product(range(n), range(n), range(n), range(n)):
            coeffs[i1,j1,i2,j2] = np.sqrt((grid[i1,j1,0]-grid[i2,j2,0]) ** 2 + (grid[i1,j1,1]-grid[i2,j2,1]) ** 2)
            # print(f"({i1},{j1},{i2},{j2}):({grid[i1,j1,0]},{grid[i1,j1,1]},{grid[i2,j2,0]},{grid[i2,j2,1]}): {coeffs[i1,j1,i2,j2]}")
        
        # Set the minimization problem
        vars = m.addMVar((n,n,n,n), obj=coeffs, vtype=GRB.BINARY, name='x')
        
        
        # Constraints: two edges incident to each city
        cons = m.addConstrs(vars[i,j].sum() == 2 for i in range(n) for j in range(n))
        # Constraints: no loopholes!
        cons = m.addConstrs(vars[i,j,i,j] == 0 for i in range(n) for j in range(n))
        # Constraints: symmetry!
        cons = m.addConstrs(vars[i1,j1, i2, j2] == vars[i2, j2, i1, j1] for i1 in range(n) for j1 in range(n) for i2 in range(n) for j2 in range(n))
        
        m._vars = vars
        m.Params.lazyConstraints = 1
        m.optimize(subtourelim)
        
        # Get the list "tour"
        selected = np.transpose(np.where(m._vars.x > 0.5))
        tour, all_t = subtour(selected)
        print(tour)
        tour.append(tour[0])
        # assert len(tour) == n ** 2 
        
        # Graphical sanity check for the tour
        plt.figure()
        plt.title(f"TSP Loop for N={n}")
        plt.scatter(grid[:, :, 0], grid[:, :, 1], color="blue", s=2)
        
        for small_tour in all_t:
            print(small_tour)
            tour_np = np.array([]).reshape(0, 2)
            for i, j in small_tour:
                tour_np = np.concatenate((tour_np, grid[i, j].reshape(1, 2)))
            plt.plot(tour_np[:, 0], tour_np[:, 1], color="green", linewidth=1)
        plt.grid()
        # plt.show()
        plt.savefig(f"graphs/TSP Loop for N={n}.png") 

        # Plot scalability
        timings.append(time.time() - start_of_timing)
        plt.figure()
        plt.title(f"Scalability (N vs. computation time)")
        plt.plot(np.linspace(2, n, n-1), timings, color="green", linewidth=1)
        plt.grid()
        plt.savefig(f"graphs/Scalability graph up to N={n}.png") 

        
    
    
    
