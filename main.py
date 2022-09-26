import json
import time
from queue import PriorityQueue
from math import sqrt, radians, sin, cos, sqrt, atan2, atan, tan

'''
Search Functions
'''


def ucs(graph, dist, energy, start, goal):
    count = 0
    q = PriorityQueue()
    q.put((0, start, [start]))  # store starting state, cost = 0, node = start, [path]
    explored = set()
    while not q.empty():
        cost, curr_node, goal_path = q.get()  # priority queue returns item-in-queue with smallest value
        explored.add(curr_node)
        if curr_node == goal:
            # goal found!
            # goal_distance = get_distance(goal_path,dist)
            goal_e_cost = get_energy(goal_path, energy)
            goal_path = '->'.join(goal_path)
            print(f"Shortest path: {goal_path}.\n\nShortest distance: {cost}.\nTotal Expansions: {count}")
            return
        for node in graph[curr_node]:  # expand nodes, eg: curr_node = 1
            if node not in explored:
                count += 1
                pair = f'{curr_node},{node}'  # eg: pair = '1,2', cost from 1->2
                total_cost = cost + dist[pair]  # cumulative cost + cost(curr_node->neighbour_node)
                q.put((total_cost, node, goal_path + [node]))
    # no path found
    return -1


def ucs_budget(graph, dist, energy, start, goal, budget):
    count = 0
    q = PriorityQueue()
    q.put((0, 0, start, [start]))
    explored = set()
    while not q.empty():
        count += 1
        cost, e_cost, curr_node, goal_path = q.get()
        explored.add(curr_node)
        if curr_node == goal:
            # goal found!
            goal_distance = get_distance(goal_path, dist)
            goal_path = '->'.join(goal_path)
            print(
                f"Shortest path: {goal_path}.\n\nShortest distance: {goal_distance}.\nTotal energy cost: {e_cost}. \nExplored Nodes: {count}")
            return
        for node in graph[curr_node]:
            if node not in explored:
                pair = f'{curr_node},{node}'
                total_cost = cost + dist[pair]
                total_energy = e_cost + energy[pair]
                if total_energy > budget:
                    break
                q.put((total_cost, total_energy, node, goal_path + [node]))
    # no path found
    return -1


def asearch(graph, dist, energy, start, goal, budget, coords):
    count = 0
    goal_coord = coords[goal]
    q = PriorityQueue()
    q.put((0, 0, start, [start]))
    explored = set()
    while not q.empty():
        cost, e_cost, curr_node, goal_path = q.get()
        explored.add(curr_node)
        # goal node is reached
        if curr_node == goal:
            goal_distance = get_distance(goal_path, dist)
            goal_path = '->'.join(goal_path)
            print(
                f"Shortest path: {goal_path}.\n\nShortest distance: {goal_distance}.\nTotal energy cost: {e_cost}. \nExplored Nodes: {count}")
            return
        for node in graph[curr_node]:
            if node not in explored:
                count += 1  # Not necessary?
                pair = f'{curr_node},{node}'
                total_energy = e_cost + energy[pair]
                if total_energy > budget:
                    break
                gn = cost + dist[pair]
                hn = euclidean_heuristic(coords[node], goal_coord)
                fn = gn + hn
                q.put((fn, total_energy, node, goal_path + [node]))
    # no path found
    print("Energy limit exceeded")
    return -1


'''
Utility Functions
'''


# returns the total distance of a given path
def get_distance(path, dist):
    distance = 0
    for i in range(0, len(path) - 1):
        pair = f'{path[i]},{path[i + 1]}'
        distance += dist[pair]
    return distance


# returns the total energy count of a given path
def get_energy(path, cost):
    e_cost = 0
    for i in range(0, len(path) - 1):
        pair = f'{path[i]},{path[i + 1]}'
        e_cost += cost[pair]
    return e_cost


# returns the euclidean distance between two points
def euclidean_heuristic(coord1, coord2):
    (lon1, lat1) = coord1
    (lon2, lat2) = coord2

    dx = abs(lon1 / 1000000 - lon2 / 1000000)
    dy = abs(lat1 / 1000000 - lat2 / 1000000)
    return sqrt(dx ** 2 + dy ** 2)


# returns the haversine distince between two cooridnates on Earth
def haversine_heuristic(coord1, coord2):
    R = 6371
    (lon1, lat1) = coord1
    (lon2, lat2) = coord2
    lon1 = lon1 / 1000000
    lat1 = lat1 / 1000000
    lon2 = lon2 / 1000000
    lat2 = lat2 / 1000000
    dLat = radians(lat2 - lat1)  # why radians?
    dLon = radians(lon2 - lon1)  # why radians?

    a = sin(dLat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dLon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c


'''
Main Program
'''
with open('G.json') as json_file:
    G = json.load(json_file)
with open('Coord.json') as json_file:
    Coord = json.load(json_file)
with open('Dist.json') as json_file:
    Dist = json.load(json_file)
with open('Cost.json') as json_file:
    Cost = json.load(json_file)

START_NODE = '1'
GOAL_NODE = '50'
ENERGY_LIMIT = 287932

print("performing A* Search")
start = time.time()
asearch(G, Dist, Cost, START_NODE, GOAL_NODE, ENERGY_LIMIT, Coord)
print("time taken:")
print(time.time() - start)

ucs(G, Dist, Cost, START_NODE, GOAL_NODE)