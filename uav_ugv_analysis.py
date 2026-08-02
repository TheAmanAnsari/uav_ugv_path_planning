import networkx as nx
import numpy as np
import re
from tsp import nearest_neighbor_tsp
from map_utils_v2 import store_local_map
from a_star import astar_nearest_target_with_path
from config import grid_size, tether_length, height
from milp_v3 import milp_main
from connectivity_check import is_map_connected
from connect_regions import connect_regions
from transition_v2 import transition, plan_ugv_path_v2, inter_region_map
from anchor_ordering import order_clusters

# grid = np.load(f'Maps/global_map_{grid_size}x{grid_size}.npy')
# grid = np.load(f'z_experiments_0/global_map_{grid_size}x{grid_size}.npy')
# ugv_road_network_json = f'ugv_points_{grid_size}x{grid_size}_v004.json'

def get_map(obstacles):
    np_map = np.ones((grid_size, grid_size), dtype=int)
    for (x, y) in obstacles:
        np_map[x][y] = 3
    
    return np_map

def anchor_pairs(points):
    return [(points[i], points[i+1]) for i in range(len(points) - 1)]

def parse_circle_data(results):
    circle_data = {
        tuple(map(float, re.search(r'\(([^,]+),\s*([^)]+)\)', circle_str).groups())): points
        for circle_str, points in results
    }
    return circle_data

def analyze_uav_ugv_coverage(results, grid_size, ugv_candidate_inverted, obstacles,
                             ugv_road_network_json=None, global_map=None,
                             den=None, exp=None, folder='experiments_0'):
    """
    Processes UAV/UGV coverage data and prepares paths and maps for execution.
    """
    COVERAGE_PATH = []
    UGV_PATH = []
    circle_data = parse_circle_data(results)
    VISITED = set()

    # circle_centers = nearest_neighbor_tsp(reference_coordinate, list(circle_data.keys()))
    circle_centers, uav_start = order_clusters(circle_data, ugv_road_network_json)
    print(f"circle_centers: {circle_centers}")
    circle_waypoints = [circle_data[center] for center in circle_centers]

    for i in range(len(circle_waypoints)):
        print(f" {circle_centers[i]}: {circle_waypoints[i]}")
    print("\n")
    print("-" * 20)

    # print(f"circle_waypoints (from uav_ugv_analysis): {circle_waypoints}")

    np_map = get_map(obstacles)
    num = 0
    starting_point_list = []
    last = None
    c = 0
    anchors = anchor_pairs(circle_centers)
    uav_transition_path, ugv_transition_path = None, None
    ugv_num = 0

    for waypoints in circle_waypoints:
        idx = circle_waypoints.index(waypoints)
        print(f" ---> New iteration starts for idx: {idx} <--- ")
        print(f"circle_centers: {circle_centers}")
        print(f"circle_centers[idx]: {circle_centers[idx]} where idx = {idx}")
        print(f"waypoints: {waypoints}")
        print("-" * 20)
        
        # starting_point = calculate_starting_point(reference_coordinate, waypoints)
        if num == 0:
            starting_point = uav_start
            print(f" [num = 0] starting_point: {starting_point}")
        else:
            # starting_point, path_to_next_point, cost = astar_nearest_target_with_path(grid, last, waypoints)

            # ugv_path = plan_ugv_path_v2(P=anchors[c][0], Q=anchors[c][1])
            # INTER_REGION_MAP, reachable_cells = inter_region_map(ugv_path=ugv_path, ugv_candidate_inverted=ugv_candidate_inverted)

            uav_transition_path, ugv_transition_path = transition(A=last, P=anchors[c][0], Q=anchors[c][1], 
                                            ugv_candidate_inverted=ugv_candidate_inverted, obstacles=obstacles,
                                            road_network=ugv_road_network_json, global_map_path=global_map)
            c += 1
            
            starting_point = uav_transition_path[-1]
            print(f" [num > 0] starting_point: {starting_point}")

        if starting_point not in waypoints:
            a, b = starting_point
            np_map[a][b] = 2

        for (x, y) in waypoints:
            if (x, y) == starting_point:
                np_map[x][y] = 2
            elif (x, y) in VISITED:
                continue
            else:
                np_map[x][y] = 0

        # starting_point_list.append(starting_point)
        # reference_coordinate = starting_point

        print("---------------------------------")
        if not is_map_connected(np_map):
            np_map = connect_regions(np_map)

        # print(np_map)
        store_local_map(np_map, grid_size, num, den, exp, folder)
        
        num += 1
        npy = f"{folder}/density_{den:02d}%/run_{exp:03d}/local_maps/maps_{grid_size}x{grid_size}_{num}.npy" 

        # if num > 2:
        #     break

        # waypoints
        print(f"waypoints: {waypoints}")

        has_free_cells = np.any(np.load(npy) == 0)

        # path code start
        if has_free_cells:
            # path, path_with_ori = main(npy, initial_orientation=ori, time_limit=None)
            path = milp_main(npy)
        else:
            path = [starting_point]
        
        last = path[-1]

        print(f"path (milp): {path}")
        print(f"last: {last}")

        ground_vehicle_path = []
        for _ in range(len(path)):
            ground_vehicle_path.append(circle_centers[idx])

        # print(f"path_to_next_point: {path_to_next_point}")
        print(f"uav_transition_path: {uav_transition_path}")
        print(f"ugv_transition_path: {ugv_transition_path}")

        # if path_to_next_point is not None:
        #     if len(path_to_next_point) > 2:
        #         trimed_path_to_next_point = path_to_next_point[1:-1]
        #         path = trimed_path_to_next_point + path

        if uav_transition_path is not None:
            if len(uav_transition_path) > 2:
                trimed_path_to_next_point = uav_transition_path[1:-1]
                path = trimed_path_to_next_point + path
        
        print(f"path (extension): {path}")
        print("-" * 20)
        # path code end

        for (x, y) in path:
            VISITED.add((x, y))

        COVERAGE_PATH += path

        # print(f"UGV_PATH: {UGV_PATH}")
        # print("-" * 20)
        
        path_2 = []
        if ugv_transition_path is not None:
            if len(ugv_transition_path) > 2:
                trimed_ugv_path_to_next_point = ugv_transition_path[1:-1]
                path_2 = trimed_ugv_path_to_next_point + ground_vehicle_path
        
        if ugv_num > 0:
            UGV_PATH += path_2
        else:
            UGV_PATH += ground_vehicle_path
        ugv_num += 1

        # UGV_PATH += path_2

        path.clear()
        np_map = get_map(obstacles)

    return COVERAGE_PATH, UGV_PATH, circle_centers
