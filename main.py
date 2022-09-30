import json
import time
from queue import PriorityQueue
from math import radians, sin, cos, sqrt, atan2, atan, tan, acos

'''
Search Functions
'''


# For task 1
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
            goal_distance = get_distance(goal_path, dist)  # this was commented out?
            goal_e_cost = get_energy(goal_path, energy)
            goal_path = '->'.join(goal_path)
            print(
                f"Shortest path: {goal_path}.\n\nShortest distance: {goal_distance}.\nTotal energy cost: {goal_e_cost}. "
                f"\nNodes added in search space: {count}")
            return
        for node in graph[curr_node]:  # expand nodes, eg: curr_node = 1
            if node not in explored:
                count += 1
                pair = f'{curr_node},{node}'  # eg: pair = '1,2', cost from 1->2
                total_cost = cost + dist[pair]  # cumulative cost + cost(curr_node->neighbour_node)
                q.put((total_cost, node, goal_path + [node]))
    # no path found
    return -1


# For testing of heuristics
def asearch(graph, dist, energy, start, goal, coords, heuristic):
    count = 0
    goal_coord = coords[goal]
    q = PriorityQueue()
    q.put((0, start, [start]))
    explored = set()
    while not q.empty():
        cost, curr_node, goal_path = q.get()
        explored.add(curr_node)
        if curr_node == goal:
            goal_distance = get_distance(goal_path, dist)
            goal_e_cost = get_energy(goal_path, energy)
            goal_path = '->'.join(goal_path)
            print(
                f"Shortest path: {goal_path}.\n\nShortest distance: {goal_distance}.\nTotal energy cost: {goal_e_cost}. "
                f"\nNodes added in search space: {count}")
            return
        for node in graph[curr_node]:
            if node not in explored:
                count += 1
                pair = f'{curr_node},{node}'
                gn = cost + dist[pair]
                hn = heuristic(coords[node], goal_coord)
                fn = gn + hn
                q.put((fn, node, goal_path + [node]))


# For task 2
def ucs_budget(graph, dist, energy, start, goal, budget):
    count = 0
    q = PriorityQueue()
    q.put((0, 0, start, [start]))
    explored = set()
    while not q.empty():
        cost, e_cost, curr_node, goal_path = q.get()
        explored.add(curr_node)
        if curr_node == goal:
            # goal found!
            goal_distance = get_distance(goal_path, dist)
            goal_path = '->'.join(goal_path)
            print(
                f"Shortest path: {goal_path}.\n\nShortest distance: {goal_distance}.\nTotal energy cost: {e_cost}. "
                f"\nNodes added in search space: {count}")
            return
        for node in graph[curr_node]:
            if node not in explored:
                count += 1
                pair = f'{curr_node},{node}'
                total_cost = cost + dist[pair]
                total_energy = e_cost + energy[pair]
                if total_energy > budget:
                    break
                q.put((total_cost, total_energy, node, goal_path + [node]))
    # no path found
    print("Energy limit exceeded")
    return -1


# For task 3
def asearch_budget(graph, dist, energy, start, goal, budget, coords, hw, heuristic):
    count = 0
    goal_coord = coords[goal]
    q = PriorityQueue()
    q.put((0, 0, start, [start]))
    explored = set()
    while not q.empty():
        d_cost, e_cost, curr_node, goal_path = q.get()
        explored.add(curr_node)
        # goal node is reached, print details
        if curr_node == goal:
            goal_distance = get_distance(goal_path, dist)
            goal_path = '->'.join(goal_path)
            print(
                f"Shortest path: {goal_path}.\n\nShortest distance: {goal_distance}.\nTotal energy cost: {e_cost}. "
                f"\nNodes added in search space: {count}")
            return
        for node in graph[curr_node]:
            if node not in explored:
                count += 1
                pair = f'{curr_node},{node}'
                total_energy = e_cost + energy[pair]
                if total_energy > budget:
                    break
                gn = d_cost + dist[pair]
                hn = heuristic(coords[node], goal_coord)
                fn = gn + hw*hn
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


# returns the total energy cost of a given path
def get_energy(path, cost):
    e_cost = 0
    for i in range(0, len(path) - 1):
        pair = f'{path[i]},{path[i + 1]}'
        e_cost += cost[pair]
    return e_cost


# returns the euclidean distance between two points
def euclidean(coord1, coord2):
    (lon1, lat1) = coord1
    (lon2, lat2) = coord2

    dx = abs(lon1 / 1000000 - lon2 / 1000000)
    dy = abs(lat1 / 1000000 - lat2 / 1000000)
    return sqrt(dx ** 2 + dy ** 2)


# returns the haversine distance between two coordinates on Earth
def haversine(coord1, coord2):
    R = 6371
    (lon1, lat1) = coord1
    (lon2, lat2) = coord2
    lon1 = lon1 / 1000000
    lat1 = lat1 / 1000000
    lon2 = lon2 / 1000000
    lat2 = lat2 / 1000000
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    return 6371 * (
        acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon1 - lon2))
    )

# returns the manhattan distance between two points
def manhattan(coord1, coord2):
    (lon1,lat1) = coord1
    (lon2,lat2) = coord2

    lon1 = lon1/1000000
    lat1 = lat1/1000000
    lon2 = lon2/1000000
    lat2 = lat2/1000000

    return abs(lon1 - lon2) + abs(lat1 - lat2)


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

print("UCS")
start = time.time()
ucs(G, Dist, Cost, START_NODE, GOAL_NODE)
print(f"time taken: {time.time() - start}")
print("---------------------------------")

print("A* Search(haversine)")
start = time.time()
asearch(G, Dist, Cost, START_NODE, GOAL_NODE, Coord, haversine)
print(f"time taken: {time.time() - start}")
print("---------------------------------")

print("A* Search(euclidean)")
start = time.time()
asearch(G, Dist, Cost, START_NODE, GOAL_NODE, Coord, euclidean)
print(f"time taken: {time.time() - start}")
print("---------------------------------")

print("A* Search(manhattan)")
start = time.time()
asearch(G, Dist, Cost, START_NODE, GOAL_NODE, Coord, manhattan)
print(f"time taken: {time.time() - start}")
print("---------------------------------")

print("UCS with energy limit")
start = time.time()
ucs_budget(G, Dist, Cost, START_NODE, GOAL_NODE, ENERGY_LIMIT)
print(f"time taken: {time.time() - start}")
print("---------------------------------")

print("A* Search with energy limit")
start = time.time()
asearch_budget(G, Dist, Cost, START_NODE, GOAL_NODE, ENERGY_LIMIT, Coord,1,haversine)
print(f"time taken: {time.time() - start}")
print("---------------------------------")

print("A* Search with energy limit (0.8)")
start = time.time()
asearch_budget(G, Dist, Cost, START_NODE, GOAL_NODE, ENERGY_LIMIT, Coord,0.8,haversine)
print(f"time taken: {time.time() - start}")
print("---------------------------------")

print("A* Search with energy limit(0.6)")
start = time.time()
asearch_budget(G, Dist, Cost, START_NODE, GOAL_NODE, ENERGY_LIMIT, Coord,0.6,haversine)
print(f"time taken: {time.time() - start}")
print("---------------------------------")

print("A* Search with energy limit(0.4)")
start = time.time()
asearch_budget(G, Dist, Cost, START_NODE, GOAL_NODE, ENERGY_LIMIT, Coord,0.4,haversine)
print(f"time taken: {time.time() - start}")
print("---------------------------------")
