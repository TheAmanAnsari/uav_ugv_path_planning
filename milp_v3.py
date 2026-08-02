import numpy as np
import gurobipy as gp
from gurobipy import GRB
from collections import defaultdict, deque


DIRS = [
    ( 0,  1),   # right
    ( 1,  0),   # down
    ( 0, -1),   # left
    (-1,  0),   # up
]

# Graph construction

def build_graph(MAP):
    R, C = MAP.shape
    free_cells = [(i, j) for i in range(R) for j in range(C) if MAP[i, j] in (0, 2)]
    free_set = set(free_cells)

    starts = [(i, j) for (i, j) in free_cells if MAP[i, j] == 2]
    if len(starts) != 1:
        raise ValueError("Exactly one start cell required")
    start_cell = starts[0]


    # cell arcs
    cell_arcs = []
    for (i, j) in free_cells:
        for (di, dj) in DIRS:
            ni, nj = i + di, j + dj

            if (ni, nj) in free_set:
                cell_arcs.append(((i, j), (ni, nj), 1.0))

    cell_arcs = list(set(cell_arcs))

    return {
        "R": R,
        "C": C,
        "free_cells": free_cells,
        "start_cell": start_cell,
        "cell_arcs": cell_arcs,
    }


# ------------------------------------------------------------
# MILP with Gurobi

def solve_milp(MAP, time_limit=None):
    G = build_graph(MAP)

    free_cells = G["free_cells"]
    start_cell = G["start_cell"]
    cell_arcs = G["cell_arcs"]

    V = len(free_cells)
    bigM_t = 4 * MAP.size
    bigM_f = V - 1

    model = gp.Model("coverage")

    if time_limit:
        model.setParam(GRB.Param.TimeLimit, time_limit)

    # VARIABLES
    t = {
        (u, v) : model.addVar(vtype=GRB.INTEGER, lb=0, ub=V)
         for (u, v, _) in cell_arcs
    }
    
    end = {
        cell: model.addVar(vtype=GRB.BINARY)
        for cell in free_cells
    }

    y = {(u, v): model.addVar(vtype=GRB.BINARY) for (u, v, _) in cell_arcs}

    f = {(u, v): model.addVar(lb=0, ub=bigM_f) for (u, v, _) in cell_arcs}

    # OBJECTIVE
    model.setObjective(
        gp.quicksum(cost * t[(u, v)] 
                    for (u, v, cost) in cell_arcs),
                    GRB.MINIMIZE
    )

    model.addConstr(
        gp.quicksum(end[cell] for cell in free_cells) == 1
    )

    model.addConstr(end[start_cell] == 0)

    # --------------------------------------------------------
    # LINK t → y

    for (u, v, _) in cell_arcs:
        model.addConstr(t[(u, v)] >= 1 * y[(u, v)])
        model.addConstr(t[(u, v)] <= bigM_t * y[(u, v)])

    # --------------------------------------------------------
    # SCF CONNECTIVITY
    for (u, v, _) in cell_arcs:
        model.addConstr(f[(u, v)] <= bigM_f * y[(u, v)])

    for cell in free_cells:

        incident = gp.quicksum(
            y[(u, v)]
            for (u, v, _) in cell_arcs
            if u == cell or v == cell
        )

        model.addConstr(incident >= 1)

    for u_cell in free_cells:
        out_f = gp.quicksum(
            f[(u, v)] 
            for (u, v, _) in cell_arcs 
            if u == u_cell
        )
        
        in_f = gp.quicksum(
            f[(u, v)] 
            for (u, v, _) in cell_arcs 
            if v == u_cell
        )

        if u_cell == start_cell:
            model.addConstr(out_f - in_f == V - 1)
        else:
            model.addConstr(in_f - out_f == 1)

    for cell in free_cells:

        in_expr = gp.quicksum(
            t[(u, v)]
            for (u, v, _) in cell_arcs
            if v == cell
        )

        out_expr = gp.quicksum(
            t[(u, v)]
            for (u, v, _) in cell_arcs
            if u == cell
        )

        if cell == start_cell:
            model.addConstr(out_expr - in_expr == 1)
        else:
            model.addConstr(in_expr - out_expr == end[cell])

    # --------------------------------------------------------
    # SOLVE
    model.optimize()

    if model.Status not in (GRB.OPTIMAL, GRB.TIME_LIMIT):
        raise RuntimeError("No solution")

    t_sol = {
        (u, v): int(round(t[(u, v)].X))
        for (u, v) in t
        if t[(u, v)].X > 0
    }

    print("Used arcs:", len(t_sol))
    
    # print(f"t_sol: {t_sol}")

    return t_sol, G


# ------------------------------------------------------------
# Extract Path

def extract_path(t_sol, start_cell):

    adj = defaultdict(deque)

    for (u, v), mult in t_sol.items():
        for _ in range(mult):
            adj[u].append(v)

    stack = [start_cell]
    path = []

    while stack:
        u = stack[-1]

        if adj[u]:
            stack.append(adj[u].popleft())
        else:
            path.append(stack.pop())

    return path[::-1]


# ------------------------------------------------------------
# MAIN

def milp_main(npy_path="map.npy"):
    MAP = np.load(npy_path)
    # print(MAP)
    t_sol, G = solve_milp(MAP)

    path = extract_path(
        t_sol,
        G["start_cell"]
    )

    print(f"Path:\n{path}")

    visited = set(path)
    missing = [c for c in G["free_cells"] if c not in visited]

    print("Visited:", len(visited))
    print("Free cells:", len(G["free_cells"]))
    print("Missing:", len(missing))

    if missing:
        print("Missing cells:", missing[:10])  # print a few

    return path


if __name__ == "__main__":
    milp_main("Maps/maps_30x30_2.npy")