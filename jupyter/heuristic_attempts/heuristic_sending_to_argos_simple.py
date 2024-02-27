from queue import PriorityQueue
import math
import itertools
import numpy as np
import random

import cv2

from collections import Counter

from matplotlib import pyplot, colors, animation, functools

from PIL import Image
import io

import cProfile, pstats, io
from pstats import SortKey
pr = cProfile.Profile()
pr.enable()

#number of robots
k = 4
#nodes per axis
nodes_per_axis = 4
#physical size in meters of the field. the area will then be defined to go from -edge_length/2 to edge_length/2
edge_length = 2

RP = 2

#FAILURES
#we really should write this as an interrupt or in a thread, but want to discuss before doing that
#chance robot will fail as a value from 0 to 1
alpha = 0.05
#percent of robots that will fail as a value from 0 to 1
robot_failure_percent = 0.5

from PIL import Image
#robot paths as a list of lists
robot_paths = [[] for ki in range(k)]
#distance the robot can travel without needing to return to depot
fuel_capacity = math.sqrt(edge_length^2 + edge_length^2)
#robot fuel is a list storing each robot's fuel at the present moment
robot_fuel = [fuel_capacity for ki in range(k)]
#nodes covered is a set of every node that has been covered so far
nodes_covered = set()

#set containing all possible nodes in the map -- not a particularly efficient way of doing this
all_nodes = set()
for x in range(0,nodes_per_axis):
    for y in range(0,nodes_per_axis):
        all_nodes.add((x,y))

#THE MAP IS ASSUMED TO BE STRUCTURED SUCH THAT (0,0) IS THE DEPOT AND THE LOWER LEFT HAND CORNER OF THE GRID


#takes in a tuple representing node that's neighbors are desired
def neighbors(curr):
    ns = [(curr[0]+1, curr[1]), (curr[0]-1, curr[1]), (curr[0], curr[1]+1), (curr[0], curr[1]-1), (curr[0]+1, curr[1]+1), (curr[0]-1, curr[1]-1), (curr[0]+1, curr[1]-1), (curr[0]-1, curr[1]+1)]
    neighbors = []
    for n in ns:
        if n[0] < nodes_per_axis and n[0] >= 0 and n[1] < nodes_per_axis and n[1] >= 0:
            neighbors.append(n)
    return neighbors

def heuristic(node):
    if node in nodes_covered:
        return -1
    else:
        return -10

#https://www.redblobgames.com/pathfinding/a-star/implementation.html#python-astar
def a_star_search(start, goal):
    robot_failed = False
    frontier = PriorityQueue()
    frontier.put((0, (start[0],start[1])))

    came_from = dict()
    cost_so_far = dict()
    came_from[(start[0],start[1])] = None
    cost_so_far[(start[0],start[1])] = 0


    while not frontier.empty():
        current = frontier.get()
        if current == (goal[0], goal[1]):
            break

        neighbor_list = neighbors(current[1])
        for next in neighbor_list:
            new_cost = cost_so_far[current[1]] + math.dist(current[1], next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next)

                frontier.put((priority, next))
                came_from[next] = current[1]

    curr_val = goal
    final_path = []
    dist = 0
    while curr_val != start:
        final_path.append(curr_val)

        dist = dist + math.dist(curr_val, came_from[curr_val])
        curr_val = came_from[curr_val]


    #while i in range(0,len(final_path)+1) and robot_failed != True:
    #    if random.random() < alpha:
    #        robot_failed = True
    #        i += 1
    final_path.append(start)
    final_path.reverse()

    i = len(final_path)
    #generate random number
    if random.random() < alpha:
        i = random.randrange(1, len(final_path), 1)
        robot_failed = True

    return final_path[:i], dist, robot_failed



#define a costmap of the field in terms of distance from depot, in the form
def calculate_costmap():
    map = np.zeros((nodes_per_axis,nodes_per_axis))
    for x in range(0, nodes_per_axis):
        for y in range(0, nodes_per_axis):
            map[x][y] = math.dist((0,0), (x,y))

    return map

#finds k (# of robots) of the largest values in the costmap
#not super useful for this just anticipate it being useful i think
def find_max():
    map = calculate_costmap()
    max = [[0, (0, 0)] for r in range(k)]
    for x in range(0, nodes_per_axis):
        for y in range(0, nodes_per_axis):
            curr = map[x][y]
            if(curr > max[2][0]):
                if(curr > max[1][0]):
                    if(curr > max[0][0]):
                        max[2] = max[1]
                        max[1] = max[0]
                        max[0] = [curr, (x,y)]
                    else:
                        max[2] = max[1]
                        max[1] = [curr, (x,y)]
                else:
                    max[2] = [curr, (x,y)]
    return max

"""
def generate_robot_paths():
    last_node = [(0,0) for ki in range(k)]

    while nodes_per_axis*nodes_per_axis - len(nodes_covered) > 0:
        for ki in range(0,k):
            goal = (0,0)
            while goal in nodes_covered and math.dist(goal, (0,0)) < robot_fuel[ki] and len(nodes_covered) < nodes_per_axis*nodes_per_axis: #if goal is already covered, find a different one
                #nodes_uncovered = [item for item in all_nodes if item not in nodes_covered]
                #goal = random.choice(nodes_uncovered)

                max_dist = 0
                goal = (0,0)
                for n in nodes_uncovered:
                    if math.dist((0,0),n) > max_dist:
                        max_dist = math.dist((0,0),n)
                        goal = n


            path, distance_travelled, robot_failed = a_star_search(last_node[ki], goal)

            robot_paths[ki] = robot_paths[ki] + path
            nodes_covered.update(path)

            robot_fuel[ki] = robot_fuel[ki] - distance_travelled

            last_node[ki] = robot_paths[ki][len(robot_paths[ki])-1]

            if robot_failed == True:
                #print("ROBOT", ki, "FAILED")
                last_node[ki] = (0,0)

            #managing fuel levels
            if (0,0) == last_node[ki]:
                robot_fuel[ki] = fuel_capacity


    return robot_paths
"""

def generate_robot_paths_redundancy():
    last_node = [(0,0) for ki in range(k)]
    nodes_seen = []
    while nodes_per_axis*nodes_per_axis - len(nodes_covered) > 0:
        for ki in range(0,k):
            goal = (0,0)
            while goal in nodes_covered and math.dist(goal, (0,0)) < robot_fuel[ki] and len(nodes_covered) < nodes_per_axis*nodes_per_axis: #if goal is already covered, find a different one
                nodes_uncovered = [item for item in all_nodes if item not in nodes_covered]

                max_dist = 0
                goal = (0,0)
                for n in nodes_uncovered:
                    if math.dist((0,0),n) > max_dist:
                        max_dist = math.dist((0,0),n)
                        goal = n


            path, distance_travelled, robot_failed = a_star_search(last_node[ki], goal)

            robot_paths[ki] = robot_paths[ki] + path

            [nodes_seen.append(p) for p in path]

            counted_nodes_seen = Counter(nodes_seen)

            for n in nodes_seen:
                if counted_nodes_seen[n] >= RP:
                    nodes_covered.add(n)

            robot_fuel[ki] = robot_fuel[ki] - distance_travelled

            last_node[ki] = robot_paths[ki][len(robot_paths[ki])-1]

            if robot_failed == True:
                last_node[ki] = (0,0)

            #managing fuel levels
            if (0,0) == last_node[ki]:
                robot_fuel[ki] = fuel_capacity


    return robot_paths, nodes_seen

"""
def collision_resolver():
    for i in range(0, max(len(robot_paths[ki]) for ki in range(k))):
        vectors = []
        for ki in range(k):
            if i < len(robot_paths[ki])-1:
                #extract vector information
                curr = robot_paths[ki][i]
                next = robot_paths[ki][i+1]

                vectors[ki] = (curr, next)

        #compare vector results from each robot for each step
        for k in range(0, len(vectors)):
            for k2 in range(0, len(vectors)):
                if k != k2: # want to compare every vector to every other vector without comparing any vector to itself
                    if vectors[k][0] == vector[k2][0]: #starting node same
                        print("something has gone extremely wrong! starting node of two robots is the same")
                    if vectors[k][1] == vectors[k2][1]: #ending node same
                        #pick one, find nearest open node and send there instead

                    if vectors[k][1] == vectors[k2][0] and vectors[k][0] == vectors[k2][1]: #going to each other's nodes
"""


##VISUALIZATION

#visualizes robot paths
def visualize_paths_brute_force():
    for ki in range(k):
        fig = pyplot.figure()
        fig.suptitle(f"Path for robot #{ki}")

        past_node = (0,0)
        [pyplot.scatter(x, y, c='blue', s=10) for x in range(0, nodes_per_axis) for y in range(0, nodes_per_axis)]

        for node in robot_paths[ki]:
            pyplot.scatter(node[0], node[1], c="purple", s=8)
            pyplot.plot([node[0], past_node[0]], [node[1], past_node[1]], color="purple", linewidth=1)

            past_node = (node[0], node[1])

        pyplot.grid()

        pyplot.savefig("data/"+ str(nodes_per_axis) + "n" + str(k) + "r" + str(edge_length) +"e_"+str(ki)+ "rp" + str(RP) + "failure"+ str(alpha) + "_" + str(robot_failure_percent) + ".png")

print("generating paths")
generate_robot_paths_redundancy()

print("visualizing paths")
visualize_paths_brute_force()

print("creating videos of heatmaps")
#pyplot.ion()

#creates initial heatmap which just counts every time a node is visited over the period of the simulation
heatmap = np.zeros((nodes_per_axis, nodes_per_axis))
figure = pyplot.figure(figsize=(5,5))
plt = pyplot.imshow(heatmap[:,:], norm=colors.Normalize(0,25))
ax = pyplot.subplot()

txt = ax.text(-1, -1,"frame: ", fontsize=12)

pyplot.colorbar().set_ticks([0,25])

def animate(z):
    for ki in range(k):
        if z < len(robot_paths[ki]):
            (x,y) = robot_paths[ki][z]
            heatmap[x][y] = heatmap[x][y] + 1

    plt = pyplot.imshow(heatmap[:,:], norm=colors.Normalize(0,25))
    txt.set_text("frame: " + str(z))

    with io.BytesIO() as buffer:
        pyplot.savefig(buffer, format = "png")
        buffer.seek(0)
        image = Image.open(buffer)
        ar = np.asarray(image)

    return ar

number_of_steps = max(len(robot_paths[ki]) for ki in range(k))
#ani = animation.FuncAnimation(figure, animate, interval=number_of_steps, frames=number_of_steps)
#ani.save("data/"+ str(nodes_per_axis) + "n" + str(k) + "r" + str(edge_length) + "e" + str(RP) + "rp" + str(alpha) + "_" + str(robot_failure_percent) + "failure"+".gif", fps = 10)

#im = cv2.imread('text.png')
#print(im.shape[1])
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter('test.mp4', fourcc, 10, (500, 500))

for z in range(0,number_of_steps):
    img = animate(z)
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

    video.write(img)

print(heatmap)

#COUNTDOWN HEATMAP
#creates secondary heatmap that shows each node on a timer
countdown_heatmap = np.zeros((nodes_per_axis, nodes_per_axis))
step_requirement = 7 #trying to maintain at least 1 visit per step_requirement steps

countdown_figure = pyplot.figure(figsize=(5,5))
countdown_ax = pyplot.subplot()
countdown_plt = pyplot.imshow(countdown_heatmap[:,:], norm=colors.Normalize(0,25))

pyplot.colorbar().set_ticks([0,25])
countdown_txt = countdown_ax.text(-1, -1,"0", fontsize=12)

def animate_countdown(z):
    for a in range(0, nodes_per_axis):
        for b in range(0, nodes_per_axis):
            countdown_heatmap[a][b] = max(0, countdown_heatmap[a][b] - 1)
    #print(np.subtract(heatmap,1))

    for ki in range(k):
        if z < len(robot_paths[ki]):
            (x,y) = robot_paths[ki][z]
            countdown_heatmap[x][y] = step_requirement

    countdown_txt.set_text("frame: " + str(z) + ", node coverage: " + str(round(100*len(countdown_heatmap[countdown_heatmap > 0])/(nodes_per_axis*nodes_per_axis),2)) + "%")
    countdown_plt = pyplot.imshow(countdown_heatmap[:,:], norm=colors.Normalize(0,25))

    with io.BytesIO() as buffer:
        pyplot.savefig(buffer, format = "png")
        buffer.seek(0)
        image = Image.open(buffer)
        ar = np.asarray(image)

    return ar

#print(im.shape[1])
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter('countdown.mp4', fourcc, 10, (500, 500))

for z in range(0,number_of_steps):
    img = animate_countdown(z)
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

    video.write(img)

cv2.destroyAllWindows()
video.release()


# display image with opencv or any operation you like

#ani = animation.FuncAnimation(countdown_figure, animate_countdown, interval=number_of_steps, frames=number_of_steps)
#ani.save("data/"+ str(nodes_per_axis) + "n" + str(k) + "r" + str(edge_length) + "e" + str(RP) + "rp" + str(alpha) + "_" + str(robot_failure_percent) + "failure_countdown.gif", fps = 10)

pr.disable()
s = io.StringIO()
sortby = SortKey.TIME
ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
ps.print_stats()
print(s.getvalue())


#sending to ArGoS
"""
import xml.etree.ElementTree as ET

mytree = ET.parse('diffusion_1_positioning.argos')
myroot = mytree.getroot()
print(myroot.attrib)

edge = [x/10 for x in range(int(-10*edge_length/2) , int(10*edge_length/2)+1, int(10*edge_length/(nodes_per_axis-1)))]
path_string = "-1 -1, "
for r in robot_paths[0]:
    path_string = path_string + str(edge[r[0]]) + " " + str(edge[r[1]]) + ", "

for x in myroot.iter('params'):
  print(x.get('path'))
  x.set('path', path_string)
  x.set('path_length', str(1+len(robot_paths[0])))

mytree.write('diffusion_2_positioning.argos')
"""
