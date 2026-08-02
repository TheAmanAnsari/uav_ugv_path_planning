import numpy as np
from geometry_utils import eucl_dist
from config import grid_size 
from obstacle import obstacles
import networkx as nx

def calculate_starting_point(reference_coordinate, points):
    candidates = [
        point + (round(eucl_dist(reference_coordinate, point), 3),)
        for point in points
    ]
    sorted_candidates = sorted(candidates, key=lambda x: x[2])
    starting_point = (sorted_candidates[0][0], sorted_candidates[0][1])
    return starting_point

def store_map(map, grid_size, idx, folder="Maps"):
    filename = f"{folder}/maps_{grid_size}x{grid_size}_{idx+1}.npy"
    np.save(filename, map)

def store_map_v2(map, grid_size, idx, folder="z_experiments_0"):
    filename = f"{folder}/local_maps/maps_{grid_size}x{grid_size}_{idx+1}.npy"
    np.save(filename, map)

def store_global_map(map, grid_size, folder="Maps"):
    filename = f"{folder}/global_map_{grid_size}x{grid_size}.npy"
    np.save(filename, map)

def store_global_map_with_ones(map, grid_size, folder="Maps"):
    filename = f"{folder}/global_map_{grid_size}x{grid_size}_1s.npy"
    np.save(filename, map)

if __name__ == '__main__':
    WITH = '0s'

    if WITH == '0s':
        np_map = np.zeros((grid_size, grid_size), dtype=int)
    else:
        np_map = np.ones((grid_size, grid_size), dtype=int)

    waypoints = nx.grid_2d_graph(grid_size, grid_size)
    print(list(waypoints))

    for (x, y) in waypoints:
        if (x, y) in obstacles:
            np_map[x][y] = 3

    print(np_map)
    store_global_map(np_map, grid_size)
    # store_global_map_with_ones(np_map, grid_size)

