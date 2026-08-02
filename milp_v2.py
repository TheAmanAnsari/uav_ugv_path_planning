import numpy as np
import gurobipy as gp
from gurobipy import GRB
from collections import defaultdict, deque

# ------------------------------------------------------------
# Orientation helpers
ORI_TO_DELTA = {
    1: (0, 1),   # right
    2: (1, 0),   # down
    3: (0, -1),  # left
    4: (-1, 0),  # up
}
DELTA_TO_ORI = {v: k for k, v in ORI_TO_DELTA.items()}


def turn_cost(o_from, o_to):
    d = (o_to - o_from) % 4
    if d == 0:
        return 0.0
    if d == 2:
        return 0.2
    return 0.1


# ------------------------------------------------------------
# Graph construction

def build_graph(M, init_orientation):
    R, C = M.shape
    free_cells = [(i, j) for i in range(R) for j in range(C) if M[i, j] in (0, 2)]
    free_set = set(free_cells)

    starts = [(i, j) for (i, j) in free_cells if M[i, j] == 2]
    if len(starts) != 1:
        raise ValueError("Exactly one start cell required")
    start_cell = starts[0]

    state_id = {}
    id_state = {}
    idx = 0
    start_state = None

    for (i, j) in free_cells:
        for o in (1, 2, 3, 4):
            state_id[(i, j, o)] = idx
            id_state[idx] = (i, j, o)
            if (i, j) == start_cell and o == init_orientation:
                start_state = idx
            idx += 1

    # cell arcs
    cell_arcs = []
    for (i, j) in free_cells:
        for (di, dj) in ORI_TO_DELTA.values():
            ni, nj = i + di, j + dj
            if (ni, nj) in free_set:
                cell_arcs.append(((i, j), (ni, nj)))
    cell_arcs = list(set(cell_arcs))

    # oriented arcs
    oriented_arcs = []
    for (i, j) in free_cells:
        for (di, dj) in ORI_TO_DELTA.values():
            ni, nj = i + di, j + dj
            if (ni, nj) in free_set:
                d = DELTA_TO_ORI[(di, dj)]
                for o in (1, 2, 3, 4):
                    u = state_id[(i, j, o)]
                    v = state_id[(ni, nj, d)]
                    c = 1.0 + turn_cost(o, d)
                    oriented_arcs.append((u, v, (i, j), (ni, nj), o, d, c))

    return {
        "R": R,
        "C": C,
        "free_cells": free_cells,
        "start_cell": start_cell,
        "state_id": state_id,
        "id_state": id_state,
        "start_state": start_state,
        "N_states": idx,
        "cell_arcs": cell_arcs,
        "oriented_arcs": oriented_arcs,
    }


# ------------------------------------------------------------
# MILP with Gurobi

def solve_milp(M, init_orientation=1, time_limit=None):
    G = build_graph(M, init_orientation)

    free_cells = G["free_cells"]
    start_cell = G["start_cell"]
    start_state = G["start_state"]
    oriented_arcs = G["oriented_arcs"]
    cell_arcs = G["cell_arcs"]
    N_states = G["N_states"]

    V = len(free_cells)
    bigM_t = 4 * M.size
    bigM_f = V - 1

    model = gp.Model("coverage")

    if time_limit:
        model.setParam(GRB.Param.TimeLimit, time_limit)

    # VARIABLES
    t = {i: model.addVar(vtype=GRB.INTEGER, lb=0, ub=bigM_t)
         for i in range(len(oriented_arcs))}

    z = {n: model.addVar(vtype=GRB.BINARY) for n in range(N_states)}

    y = {(u, v): model.addVar(vtype=GRB.BINARY) for (u, v) in cell_arcs}

    f = {(u, v): model.addVar(lb=0, ub=bigM_f) for (u, v) in cell_arcs}

    # NEW: FLOW ON STATE GRAPH

    f_state = {
        i: model.addVar(lb=0, ub=V, vtype=GRB.CONTINUOUS, name=f"f_state_{i}")
        for i in range(len(oriented_arcs))
    }

    # OBJECTIVE
    model.setObjective(
        gp.quicksum(cost * t[i]
                    for i, (_, _, _, _, _, _, cost) in enumerate(oriented_arcs)),
        GRB.MINIMIZE
    )

    # --------------------------------------------------------
    # STRONG COVERAGE (FIXED)
    # for u_cell in free_cells:
    #     in_expr = gp.quicksum(
    #         t[i] for i, (_, _, _, vc, _, _, _) in enumerate(oriented_arcs)
    #         if vc == u_cell
    #     )
    #     out_expr = gp.quicksum(
    #         t[i] for i, (_, _, uc, _, _, _, _) in enumerate(oriented_arcs)
    #         if uc == u_cell
    #     )
    #     model.addConstr(in_expr >= 1)
    #     model.addConstr(out_expr >= 1)

    for u_cell in free_cells:

        in_expr = gp.quicksum(
            t[i]
            for i, (_, _, _, vc, _, _, _) in enumerate(oriented_arcs)
            if vc == u_cell
        )

        out_expr = gp.quicksum(
            t[i]
            for i, (_, _, uc, _, _, _, _) in enumerate(oriented_arcs)
            if uc == u_cell
        )

        model.addConstr(in_expr + out_expr >= 1)

        # if u_cell == start_cell:
        #     # model.addConstr(out_expr >= 1)
        #     pass
        # else:
        #     # model.addConstr(in_expr >= 1)
        #     # model.addConstr(out_expr >= 1)
        #     model.addConstr(in_expr + out_expr >= 1)

    # --------------------------------------------------------
    # EULER FLOW
    model.addConstr(z[start_state] == 0)

    for n in range(N_states):
        out_expr = gp.quicksum(
            t[i] for i, (u, _, _, _, _, _, _) in enumerate(oriented_arcs) if u == n
        )
        in_expr = gp.quicksum(
            t[i] for i, (_, v, _, _, _, _, _) in enumerate(oriented_arcs) if v == n
        )

        if n == start_state:
            model.addConstr(out_expr - in_expr == 1)
        else:
            model.addConstr(out_expr - in_expr == -z[n])

    model.addConstr(gp.quicksum(z[n] for n in range(N_states)) == 1)

    # --------------------------------------------------------
    # LINK t → y
    arcs_by_cell = defaultdict(list)
    for i, (_, _, uc, vc, _, _, _) in enumerate(oriented_arcs):
        arcs_by_cell[(uc, vc)].append(i)

    for (uc, vc), idxs in arcs_by_cell.items():
        model.addConstr(gp.quicksum(t[i] for i in idxs) >= y[(uc, vc)])
        model.addConstr(gp.quicksum(t[i] for i in idxs) <= bigM_t * y[(uc, vc)])

    # LINK t → f_state (ADD THIS BLOCK HERE)

    for i in range(len(oriented_arcs)):
        model.addConstr(f_state[i] <= V * t[i])
        # model.addConstr(f_state[i] >= t[i])

    # # --------------------------------------------------------
    # # SCF CONNECTIVITY
    # for (u, v) in cell_arcs:
    #     model.addConstr(f[(u, v)] <= bigM_f * y[(u, v)])

    # for u_cell in free_cells:
    #     out_f = gp.quicksum(f[(u, v)] for (u, v) in cell_arcs if u == u_cell)
    #     in_f = gp.quicksum(f[(u, v)] for (u, v) in cell_arcs if v == u_cell)

    #     if u_cell == start_cell:
    #         model.addConstr(out_f - in_f == V - 1)
    #     else:
    #         model.addConstr(in_f - out_f == 1)

    # --------------------------------------------------------
    # STATE FLOW CONSERVATION (ADD HERE, where SCF was)

    for n in range(N_states):
        out_f = gp.quicksum(
            f_state[i] for i, (u, _, _, _, _, _, _) in enumerate(oriented_arcs) if u == n
        )
        in_f = gp.quicksum(
            f_state[i] for i, (_, v, _, _, _, _, _) in enumerate(oriented_arcs) if v == n
        )

        if n == start_state:
            model.addConstr(out_f - in_f == V - 1)
        else:
            model.addConstr(in_f - out_f <= 1)

    # --------------------------------------------------------
    # SOLVE
    model.optimize()

    if model.Status not in (GRB.OPTIMAL, GRB.TIME_LIMIT):
        raise RuntimeError("No solution")

    t_sol = {i: int(round(t[i].X)) for i in t if t[i].X > 0}

    print("Used arcs:", len(t_sol))
    print("States in solution graph:",
        len(set(u for i in t_sol for u,_,_,_,_,_,_ in [oriented_arcs[i]])))

    return t_sol, G


# ------------------------------------------------------------
# Euler trail reconstruction

def euler_trail(t_sol, oriented_arcs, start_state):
    adj = defaultdict(list)
    for i, mult in t_sol.items():
        u, v, *_ = oriented_arcs[i]
        for _ in range(mult):
            adj[u].append(v)

    stack = [start_state]
    trail = []
    local = {u: deque(vs) for u, vs in adj.items()}

    while stack:
        v = stack[-1]
        if v in local and local[v]:
            stack.append(local[v].popleft())
        else:
            trail.append(stack.pop())

    return trail[::-1]


# ------------------------------------------------------------

def milp_main(npy_path="map.npy"):
    M = np.load(npy_path)
    # print(M)
    t_sol, G = solve_milp(M)

    path_states = euler_trail(t_sol, G["oriented_arcs"], G["start_state"])
    path = [(G["id_state"][s][0], G["id_state"][s][1]) for s in path_states]

    print("Path:")
    print(path)

    visited = set(path)
    missing = [c for c in G["free_cells"] if c not in visited]

    print("Visited:", len(visited))
    print("Free cells:", len(G["free_cells"]))
    print("Missing:", len(missing))

    if missing:
        print("Missing cells:", missing[:10])  # print a few

    return path


if __name__ == "__main__":
    milp_main()