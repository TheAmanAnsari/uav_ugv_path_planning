import numpy as np
import networkx as nx
import heapq
import math
import json
from config import grid_size, height, tether_length

UAV_GRID_PATH = f"Maps/global_map_{grid_size}x{grid_size}.npy" 
grid = np.load(UAV_GRID_PATH)

ugv_filename = f"ugv_points_{grid_size}x{grid_size}.json"

with open(ugv_filename) as f:
    data = json.load(f)

ugv_points = {
    (tuple(entry["from"]), tuple(entry["to"])): [tuple(p) for p in entry["points"]]
    for entry in data
}


def build_uav_graph(grid: np.ndarray) -> nx.Graph:
    M = nx.Graph()
    rows, cols = grid.shape

    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 0:
                M.add_node((r, c))
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                        M.add_edge((r, c), (nr, nc))
    return M


def build_ugv_graph(ugv_points: dict) -> nx.Graph:
    """
    Build UGV road network graph from the ugv_points dictionary.
    Keys: (A, B) — road segment endpoints (intersections)
    Values: list of intermediate points along the segment
    Edges: between consecutive points in the full sequence [A, p1, p2, ..., pn, B]
    """
    R = nx.Graph()

    for (A, B), intermediates in ugv_points.items():
        full_sequence = [A] + intermediates + [B]
        for i in range(len(full_sequence) - 1):
            R.add_edge(full_sequence[i], full_sequence[i + 1])

    return R


# ── Helper Functions ──────────────────────────────────────────────────────────

def tether_check(uav_node: tuple, ugv_node: tuple, L: float) -> bool:
    """
    Check if the 3D Euclidean distance between UAV and UGV is within tether length L.
    UAV is at altitude z = 5.0, UGV is at z = 0.0
    """
    uav_r, uav_c = uav_node
    ugv_x, ugv_y = ugv_node
    dist = math.sqrt((uav_r - ugv_x)**2 + (uav_c - ugv_y)**2 + height**2)
    return dist <= L


def heuristic(uav_node: tuple, ugv_node: tuple, B: tuple, Q: tuple) -> float:
    """
    Admissible A* heuristic.
    UAV: Manhattan distance to goal B (4-connected grid)
    UGV: Euclidean distance to goal Q (lower bound on graph distance)
    h = max(uav_h, ugv_h) since both agents advance one step per tick
    """
    uav_h = abs(uav_node[0] - B[0]) + abs(uav_node[1] - B[1])
    ugv_h = math.sqrt((ugv_node[0] - Q[0])**2 + (ugv_node[1] - Q[1])**2)
    return max(uav_h, ugv_h)


# ── Path Reconstruction ───────────────────────────────────────────────────────

def reconstruct_path(came_from: dict, goal: tuple):
    """Walk back through came_from to recover the joint path, then split."""
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()

    uav_path = [s[0] for s in path]
    ugv_path = [s[1] for s in path]
    return uav_path, ugv_path


# ── Joint A* Search ───────────────────────────────────────────────────────────

# def joint_astar(A: tuple, B: tuple, M: nx.Graph,
#                 P: tuple, Q: tuple, R: nx.Graph,
#                 L: float):
#     """
#     Plan synchronized UAV and UGV paths from (A, P) to (B, Q).

#     Parameters
#     ----------
#     A : UAV start node on lattice graph M
#     B : UAV goal node on lattice graph M
#     M : UAV lattice graph
#     P : UGV start node on road network graph R
#     Q : UGV goal node on road network graph R
#     R : UGV road network graph
#     L : Maximum tether length

#     Returns
#     -------
#     (uav_path, ugv_path) — two lists of equal length, or None if infeasible
#     """
#     # sanity checks
#     if A not in M:
#         raise ValueError(f"UAV start node {A} not found in graph M")
#     if B not in M:
#         raise ValueError(f"UAV goal node {B} not found in graph M")
#     if P not in R:
#         raise ValueError(f"UGV start node {P} not found in graph R")
#     if Q not in R:
#         raise ValueError(f"UGV goal node {Q} not found in graph R")
#     if not tether_check(A, P, L):
#         raise ValueError(f"Tether constraint violated at start state: d(A={A}, P={P}) > L={L}")
#     if not tether_check(B, Q, L):
#         raise ValueError(f"Tether constraint violated at goal state: d(B={B}, Q={Q}) > L={L}")

#     start = (A, P)
#     goal  = (B, Q)

#     counter = 0
#     open_heap = []
#     heapq.heappush(open_heap, (0, counter, start))

#     g_score   = {start: 0}
#     came_from = {start: None}

#     while open_heap:
#         f, _, current = heapq.heappop(open_heap)
#         uav_cur, ugv_cur = current

#         if current == goal:
#             return reconstruct_path(came_from, goal)

#         uav_moves = list(M.neighbors(uav_cur)) + [uav_cur]  # move or stay
#         ugv_moves = list(R.neighbors(ugv_cur)) + [ugv_cur]  # move or stay

#         for uav_next in uav_moves:
#             for ugv_next in ugv_moves:

#                 # exclude both-stay to prevent infinite loops
#                 if uav_next == uav_cur and ugv_next == ugv_cur:
#                     continue

#                 # tether constraint check
#                 if not tether_check(uav_next, ugv_next, L):
#                     continue

#                 g_new     = g_score[current] + 1
#                 neighbour = (uav_next, ugv_next)

#                 if g_new < g_score.get(neighbour, math.inf):
#                     g_score[neighbour]   = g_new
#                     came_from[neighbour] = current
#                     h = heuristic(uav_next, ugv_next, B, Q)
#                     counter += 1
#                     heapq.heappush(open_heap, (g_new + h, counter, neighbour))

#     return None  # no feasible path found

def joint_bfs(A, B, M, P, Q, R, L):
    start = (A, P)
    goal  = (B, Q)

    queue = [start]
    came_from = {start: None}

    while queue:
        current = queue.pop(0)
        uav_cur, ugv_cur = current

        if current == goal:
            return reconstruct_path(came_from, goal)

        uav_moves = list(M.neighbors(uav_cur)) + [uav_cur]
        ugv_moves = list(R.neighbors(ugv_cur)) + [ugv_cur]

        for uav_next in uav_moves:
            for ugv_next in ugv_moves:
                if uav_next == uav_cur and ugv_next == ugv_cur:
                    continue
                if not tether_check(uav_next, ugv_next, L):
                    continue
                neighbour = (uav_next, ugv_next)
                if neighbour not in came_from:
                    came_from[neighbour] = current
                    queue.append(neighbour)

    return None

def transition(A, B, P, Q, L=tether_length):
    print(f"A: {A}, B: {B}, \ngrid:\n{grid}\nP:{P}, Q:{Q}, L:{L}")
    # ── Build graphs ──────────────────────────────────────────────────────────
    print("Building graphs...")
    M = build_uav_graph(grid)
    R = build_ugv_graph(ugv_points)
    print(f"  UAV graph: {M.number_of_nodes()} nodes, {M.number_of_edges()} edges")
    print(f"  UGV graph: {R.number_of_nodes()} nodes, {R.number_of_edges()} edges")
    print()

    # ── Run planner ───────────────────────────────────────────────────────────
    print("Running joint A* planner...")
    result = joint_bfs(A, B, M, P, Q, R, L)
    print()

    if result is None:
        print("No feasible path found.")
    else:
        uav_path, ugv_path = result
        print(f"Path found in {len(uav_path)} synchronized steps.")
        print()
        print(f"{'Step':<6} {'UAV node':<15} {'UGV node':<25} {'Tether dist'}")
        print("-" * 65)
        for i, (u, g) in enumerate(zip(uav_path, ugv_path)):
            d = math.sqrt((u[0] - g[0])**2 + (u[1] - g[1])**2 + height**2)
            print(f"{i:<6} {str(u):<15} {str(g):<25} {d:.3f}")
    
        return uav_path, ugv_path


if __name__ == "__main__":

    UAV_GRID_PATH = f"Maps/global_map_{grid_size}x{grid_size}.npy" 
    grid = np.load(UAV_GRID_PATH)

    ugv = [(3.243, 3.243), (7.847, 2.306), (10.207, 8.793), (8.294, 15.353), (14.789, 17.394), (16.694, 11.153), (16.164, 2.836), (0.789, 10.394), (0.789, 19.106)]

    ugv_filename = f"ugv_points_{grid_size}x{grid_size}.json"

    with open(ugv_filename) as f:
        data = json.load(f)

    ugv_points = {
        (tuple(entry["from"]), tuple(entry["to"])): [tuple(p) for p in entry["points"]]
        for entry in data
    }

    # ── Transition parameters ─────────────────────────────────────────────────
    A = (14, 11)            # UAV start node (row, col)
    B = (12, 13)            # UAV goal node  (row, col)
    P = (10.207, 8.793)     # UGV start node (x, y)
    Q = (8.294, 15.353)     # UGV goal node  (x, y)
    # L = 7.0               # tether length

    uav_path, ugv_path = transition(A, B, P, Q)
    print(f"type(uav_path): {type(uav_path)}\n {uav_path}")
    print(f"type(ugv_path): {type(ugv_path)}\n {ugv_path}")

    M = build_uav_graph(grid)
    R = build_ugv_graph(ugv_points)

    # # Check if UAV goal is reachable from UAV start ignoring tether
    # print(nx.has_path(M, A, B))

    # # Check if UGV goal is reachable from UGV start ignoring tether
    # print(nx.has_path(R, P, Q))

    # # Find the shortest UGV path from P to Q
    # ugv_shortest = nx.shortest_path(R, P, Q)

    # # At each UGV step, find the closest UAV node and check minimum possible tether distance
    # print(f"{'Step':<6} {'UGV node':<25} {'Min tether dist to any UAV node'}")
    # print("-" * 60)
    # for ugv_node in ugv_shortest:
    #     min_dist = min(
    #         math.sqrt((ugv_node[0] - r)**2 + (ugv_node[1] - c)**2 + 25.0)
    #         for (r, c) in M.nodes()
    #     )
    #     feasible = "✓" if min_dist <= 7.0 else "✗ TETHER VIOLATED"
    #     print(f"  {str(ugv_node):<25} {min_dist:.3f}   {feasible}")

    # print(f"{'Step':<6} {'UGV node':<25} {'dist(B, ugv)'}")
    # print("-" * 50)
    # for ugv_node in ugv_shortest:
    #     d = math.sqrt((B[0] - ugv_node[0])**2 + (B[1] - ugv_node[1])**2 + 25.0)
    #     feasible = "✓" if d <= 7.0 else "✗ TETHER VIOLATED"
    #     print(f"  {str(ugv_node):<25} {d:.3f}   {feasible}")

    # problem_steps = [4, 5, 6]  # 0-indexed

    # for i in problem_steps:
    #     ugv_node = ugv_shortest[i]
    #     valid_uav_nodes = [
    #         (r, c) for (r, c) in M.nodes()
    #         if math.sqrt((r - ugv_node[0])**2 + (c - ugv_node[1])**2 + 25.0) <= 7.0
    #     ]
    #     print(f"UGV step {i} {ugv_node}: {len(valid_uav_nodes)} valid UAV nodes")
    #     print(f"  {valid_uav_nodes}")