from queue import PriorityQueue
import math
import itertools
import numpy as np
import random

from tqdm import tqdm

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
k = 5
#nodes per axis
nodes_per_axis = 15
#physical size in meters of the field. the area will then be defined to go from -edge_length/2 to edge_length/2
edge_length = 2

RP = 1

#FAILURES
#we really should write this as an interrupt or in a thread, but want to discuss before doing that
#chance robot will fail as a value from 0 to 1
alpha = 0
#percent of robots that will fail as a value from 0 to 1
robot_failure_percent = 0.5

from PIL import Image
#robot paths as a list of lists
robot_paths = [[] for ki in range(k)]
#distance the robot can travel without needing to return to depot
#(given here in node distance, not physical distance)
fuel_capacity = 2*math.dist((0,0),(nodes_per_axis-1, nodes_per_axis-1))
#robot fuel is a list storing each robot's fuel at the present moment
robot_fuel = [fuel_capacity for ki in range(k)]
#nodes covered is a set of every node that has been covered so far
nodes_covered = []

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
        #print(len(final_path))
        i = random.randrange(1, len(final_path)+1, 1)
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

#

#BINARY HEATMAP

#
number_of_steps = 2000 #number of times the algorithm will run
step_requirement = 45 #trying to maintain at least 1 visit per step_requirement steps


last_node = [(0,0) for ki in range(k)]
nodes_seen = []

countdown = np.zeros((nodes_per_axis, nodes_per_axis))

def next_robot_node(ki):
    if len(robot_paths[ki]) == 0:
        goal = (0,0)
            #this math.dist here does not work like you think it does ?? ahhh

        while goal in nodes_covered and math.dist(goal, (0,0)) < robot_fuel[ki] and len(nodes_covered) < nodes_per_axis*nodes_per_axis: #if goal is already covered, find a different one
            nodes_uncovered = [item for item in all_nodes if item not in nodes_covered]

            max_dist = 0
            goal = (0,0)
            for n in nodes_uncovered:
                if math.dist((0,0),n) > max_dist:
                    max_dist = math.dist((0,0),n)
                    goal = n

        robot_paths[ki], distance_travelled, robot_failed = a_star_search(last_node[ki], goal)

        nodes_covered.extend(robot_paths[ki])

        for x in range(0, nodes_per_axis):
            for y in range(0, nodes_per_axis):
                if countdown[x][y] > 0:
                    nodes_covered.append((x,y))

        if robot_failed == True:
            last_node[ki] = (0,0)

        robot_fuel[ki] = robot_fuel[ki] - distance_travelled

    if len(robot_paths[ki]) == 1:
        last_node[ki] = robot_paths[ki][0]

        #managing fuel levels
        if (0,0) == last_node[ki]:
            robot_fuel[ki] = fuel_capacity


    if ki == k-1: #if this is the last robot we're giving a path for, -1 to all
        for a in range(0, nodes_per_axis):
            for b in range(0, nodes_per_axis):
                countdown[a][b] = max(0, countdown[a][b] - 1)


    if len(nodes_covered) >= nodes_per_axis*nodes_per_axis:
        nodes_covered.clear()


    next_node = robot_paths[ki].pop(0)

    countdown[next_node[0]][next_node[1]] = step_requirement

    return next_node


last_node = [(0,0) for ki in range(0,k)]
suggested_node = [(0,0) for ki in range(0,k)]
path_to_depot = [[] for ki in range(0,k)]
#once reached number of nodes away from the depot, need to return to depot
def gradient_explorer(ki):
    #find all neighbors of last node

    if path_to_depot[ki] == []:
        curr_neighbors = []
        path = []
        for neighbor in neighbors(last_node[ki]):
            #only consider nodes that you could a* back from if necessary
            path, distance_travelled, robot_failed = a_star_search(neighbor, (0,0))

            if distance_travelled <= robot_fuel[ki]: curr_neighbors.append(neighbor)

        curr_countdown = [random.random()*countdown[n] for n in curr_neighbors]
        if curr_countdown == []: #if we can't go anywhere legally, go back to the depot
            path_to_depot[ki] = path

        else: #else go to the minimum visited neighbor of your neighbors
            min_val = min(curr_countdown)
            suggested_node[ki] = curr_neighbors[curr_countdown.index(min_val)]

    else:
        suggested_node[ki] = path_to_depot[ki].pop(0)

    #maintenance
    robot_fuel[ki] = robot_fuel[ki] - math.dist(last_node[ki], suggested_node[ki])

    if (0,0) == suggested_node[ki]:
        robot_fuel[ki] = fuel_capacity

    if ki == k-1: #if this is the last robot we're giving a path for, -1 to all
        for a in range(0, nodes_per_axis):
            for b in range(0, nodes_per_axis):
                countdown[a][b] = max(0, countdown[a][b] - 1)

    countdown[suggested_node[ki]] = step_requirement

    last_node[ki] = suggested_node[ki]
    return suggested_node[ki]


print("creating videos of heatmaps")

binary_heatmap = np.zeros((nodes_per_axis, nodes_per_axis))

binary_figure = pyplot.figure(figsize=(5,5))
binary_ax = pyplot.subplot()
binary_ax.set_xticks(np.arange(0, nodes_per_axis+0.1))
binary_ax.set_yticks(np.arange(0, nodes_per_axis+0.1))
binary_plt = pyplot.imshow(binary_heatmap[:,:], norm=colors.Normalize(0,1))

pyplot.colorbar().set_ticks([0,1])
binary_txt = binary_ax.text(-1, -1, "0", fontsize=12)

coverage_list = []

def animate_binary(z):
    coverage = round(100*len(binary_heatmap[binary_heatmap > 0])/(nodes_per_axis*nodes_per_axis),2)
    #current percent coverage text
    binary_txt.set_text("frame: " + str(z) + ", node coverage: " + str(coverage) + "%")
    #binary plot
    binary_plt = pyplot.imshow(binary_heatmap[:,:], norm=colors.Normalize(0,1))

    #putting into a video

    with io.BytesIO() as buffer:
        pyplot.savefig(buffer, format = "png")
        buffer.seek(0)
        image = Image.open(buffer)
        ar = np.asarray(image)


    return ar

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter('data/gradient_explorer'+ str(nodes_per_axis) + "n" + str(k) + "r" + str(edge_length) + "e" + str(RP) + "rp" + str(alpha) + "_" + str(robot_failure_percent) + "st_rq" + str(step_requirement) + 'failure.mp4', fourcc, 10, (500, 500))

for z in tqdm(range(0,number_of_steps)):
    for a in range(0, nodes_per_axis):
        for b in range(0, nodes_per_axis):
            binary_heatmap[a][b] = max(0, binary_heatmap[a][b] - 1)

    for ki in range(k):
        (x,y) = next_robot_node(ki)

        binary_heatmap[x][y] = step_requirement

    coverage = round(100*len(binary_heatmap[binary_heatmap > 0])/(nodes_per_axis*nodes_per_axis),2)
    coverage_list.append(coverage)

    #if z % 10 == 0:
    #    img = animate_binary(z)
    #    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    #    video.write(img)

pyplot.close()


coverage_figure = pyplot.figure(figsize=(5,5))
coverage_ax = pyplot.subplot()
pyplot.xlabel('number of steps')
pyplot.ylabel('% coverage')

comparison = 100*step_requirement/(nodes_per_axis*nodes_per_axis/k)

def animate_coverage(z):
    r = [*range(0, len(coverage_list))]
    coverage_ax.plot(r, coverage_list[0:z])
    coverage_ax.plot(r, [comparison]*number_of_steps, '--')

    #pyplot.savefig("data/gradient_explorer_coverage_graph"+ str(nodes_per_axis) + "n" + str(k) + "r" + str(edge_length) +"e_"+str(ki)+ "rp" + str(RP) + "failure"+ str(alpha) + "_" + str(robot_failure_percent) + "st_rq" + str(step_requirement) + ".png")
    pyplot.savefig("data/updating_path_graph"+ str(nodes_per_axis) + "n" + str(k) + "r" + str(edge_length) +"e_"+str(ki)+ "rp" + str(RP) + "failure"+ str(alpha) + "_" + str(robot_failure_percent) + "st_rq" + str(step_requirement) + ".png")


animate_coverage(number_of_steps)

#for z in range(0,number_of_steps):
#    print(z)
#    animate_coverage(z)

cv2.destroyAllWindows()
video.release()


pr.disable()

#timing

#s = io.StringIO()
#sortby = SortKey.TIME
#ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
#ps.print_stats()
#print(s.getvalue())


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
