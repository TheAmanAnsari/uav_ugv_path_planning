import numpy as np
import networkx as nx
import math
import heapq
import json
from config import height, tether_length, grid_size, printdict
from path_teller_v3 import path_teller

def load_ugv_points(ugv_road_network_json):
    with open(ugv_road_network_json) as f:
        data = json.load(f)

    ugv_points = {
        (tuple(entry["from"]), tuple(entry["to"])): [tuple(p) for p in entry["points"]]
        for entry in data
    }
    return ugv_points

def get_map(obstacles):
    np_map = np.ones((grid_size, grid_size), dtype=int)
    for (x, y) in obstacles:
        np_map[x][y] = 3
    
    return np_map

# Graph Construction
def build_uav_graph(grid):
    M = nx.Graph()
    rows, cols = grid.shape
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 0:
                M.add_node((r, c))
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nr, nc = r+dr, c+dc
                    if 0 <= nr < rows and 0 <= nc < cols and grid[nr,nc] == 0:
                        M.add_edge((r,c),(nr,nc))
    return M

def build_ugv_graph(ugv_points):
    R = nx.Graph()
    for (A, B), intermediates in ugv_points.items():
        full_sequence = [A] + intermediates + [B]
        for i in range(len(full_sequence) - 1):
            R.add_edge(full_sequence[i], full_sequence[i+1])
    return R

def tether_check(uav_node, ugv_node, L):
    uav_r, uav_c = uav_node
    ugv_x, ugv_y = ugv_node
    dist = math.sqrt((uav_r-ugv_x)**2 + (uav_c-ugv_y)**2 + height**2)
    return dist <= L


# UGV Functions
def plan_ugv_path(R, P, Q):
    return nx.shortest_path(R, P, Q)

def plan_ugv_path_v2(P, Q, ugv_points):
    R = build_ugv_graph(ugv_points)
    return nx.shortest_path(R, P, Q)

def get_reachable_cells(ugv_path, ugv_candidate_inverted):
    reachable_cells = dict()
    free_cells = list()
    for (x, y) in ugv_path:
        # w = []
        # for i, cells in enumerate(ugv_candidate_sets):
        #     if (x, y) in cells:
        #         w.append(Waypoints[i])
        #         free_cells = free_cells + [Waypoints[i]]
        
        try:
            reachable_cells[(x, y)] = ugv_candidate_inverted[(x, y)]
            free_cells = free_cells + ugv_candidate_inverted[(x, y)]
        except KeyError:
            print(f"KeyError: {(x, y)} not found in ugv_candidate_inverted")
            reachable_cells[(x, y)] = []
            free_cells = free_cells + []

    return reachable_cells, free_cells

def inter_region_map(free_cells, A, obstacles):
    MAP = get_map(obstacles)
    for (x, y) in set(free_cells):
        if (x, y) == A:
            MAP[x][y] = 7
        else:
            MAP[x][y] = 0
    
    return MAP

# Main Transition Function 
def transition(A, P, Q, 
               ugv_candidate_inverted, L=tether_length,
               obstacles=None,
               road_network=None, global_map_path=None):
    print(f"A: {A}, P: {P}, Q: {Q}, L: {L}")
    print()

    ugv_points = load_ugv_points(road_network)

    grid = np.load(global_map_path)
    # build graphs
    M = build_uav_graph(grid)
    R = build_ugv_graph(ugv_points)
    # print(f"UAV graph: {M.number_of_nodes()} nodes, {M.number_of_edges()} edges")
    # print(f"UGV graph: {R.number_of_nodes()} nodes, {R.number_of_edges()} edges")
    print()

    # plan UGV path
    ugv_path = plan_ugv_path(R, P, Q)
    print(f"UGV path ({len(ugv_path)} steps): {ugv_path}")

    reachable_cells, free_cells = get_reachable_cells(ugv_path=ugv_path, ugv_candidate_inverted=ugv_candidate_inverted)

    MAP = inter_region_map(free_cells, A, obstacles)

    # print(f"MAP\n{MAP}")     
    # print(f"reachable_cells:")
    # printdict(reachable_cells)
    # print("-" * 10)
    # print(f"free_cells: {free_cells}")
    # print("-" * 10)
    # print(f"free_cell_set: {set(free_cells)}")

    uav_transition_path, ugv_transition_path = path_teller(A, ugv_path=ugv_path, reachable_cells=reachable_cells, M=M)

    print(f"uav_transition_path: {uav_transition_path}")
    print(f"ugv_transition_path: {ugv_transition_path}")

    print(f"len(uav_transition_path): {len(uav_transition_path)}, len(ugv_transition_path): {len(ugv_transition_path)}")

    return uav_transition_path, ugv_transition_path


if __name__ == "__main__":

    A = (14, 11)
    B = (12, 13)
    P = (10.207, 8.793)
    Q = (8.294, 15.353)

    uav_path, ugv_path = transition(A, B, P, Q)

    if uav_path is not None:
        print()
        print(f"UAV path: {uav_path}")
        print(f"UGV path: {ugv_path}")
        