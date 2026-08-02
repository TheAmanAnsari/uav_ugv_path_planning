import numpy as np
from ortools.linear_solver import pywraplp
import matplotlib.pyplot as plt
from collections import defaultdict, deque

# ------------------------------------------------------------
# Orientation / direction helpers
# Codes: 1=+x (right), 2=+y (down), 3=-x (left), 4=-y (up)
# Note: NumPy row index increases downward, so +y == (dr=+1, dc=0)
ORI_TO_DELTA = {
    1: (0, 1),   # right
    2: (1, 0),   # down
    3: (0, -1),  # left
    4: (-1, 0),  # up
}
DELTA_TO_ORI = {v: k for k, v in ORI_TO_DELTA.items()}


def turn_cost(ori_from: int, ori_to: int) -> float:
    """Return 0.0 for 0°, 0.1 for 90°, 0.2 for 180° turns between orientations.
    Orientation labels are 1..4 in cyclic order (right,down,left,up).
    """
    d = (ori_to - ori_from) % 4
    if d == 0:
        return 0.0
    if d == 2:
        return 0.2
    return 0.1  # 90°


# ------------------------------------------------------------
# Graph construction (cell × orientation states)

def build_oriented_graph(M: np.ndarray, init_orientation: int):
    R, C = M.shape

    # Free cells: treat 1 and 3 as obstacles; 0 and 2 are traversable
    free_cells = [(i, j) for i in range(R) for j in range(C) if M[i, j] in (0, 2)]
    free_set = set(free_cells)

    # Start cell
    starts = [(i, j) for i in range(R) for j in range(C) if M[i, j] == 2]
    if len(starts) != 1:
        raise ValueError("Map must contain exactly one start cell with value 2.")
    start_cell = starts[0]

    # State ids: (cell, orientation)
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
    N_states = idx

    # Directed cell-level arcs (for connectivity SCF)
    cell_arcs = []
    for (i, j) in free_cells:
        for di, dj in ORI_TO_DELTA.values():
            ni, nj = i + di, j + dj
            if 0 <= ni < R and 0 <= nj < C and (ni, nj) in free_set:
                cell_arcs.append(((i, j), (ni, nj)))
    cell_arcs = list(dict.fromkeys(cell_arcs))  # dedup

    # Oriented arcs: from (u,o) to (v, d) where d is the direction of u->v
    oriented_arcs = []  # list of tuples: (u_state, v_state, u_cell, v_cell, o_from, o_to, cost)
    for (i, j) in free_cells:
        for di, dj in ORI_TO_DELTA.values():
            ni, nj = i + di, j + dj
            if 0 <= ni < R and 0 <= nj < C and (ni, nj) in free_set:
                d = DELTA_TO_ORI[(di, dj)]  # required orientation for this move
                for o in (1, 2, 3, 4):
                    u = state_id[(i, j, o)]
                    v = state_id[(ni, nj, d)]  # arrive facing movement direction
                    c = 1.0 + turn_cost(o, d)  # movement + turning penalty
                    oriented_arcs.append((u, v, (i, j), (ni, nj), o, d, c))

    return {
        'R': R, 'C': C,
        'free_cells': free_cells,
        'free_set': free_set,
        'start_cell': start_cell,
        'state_id': state_id,
        'id_state': id_state,
        'start_state': start_state,
        'N_states': N_states,
        'cell_arcs': cell_arcs,
        'oriented_arcs': oriented_arcs,
    }


# ------------------------------------------------------------
# MILP with OR-Tools (CBC)

def solve_oriented_milp(M: np.ndarray, init_orientation: int, time_limit: int | None = None):
    G = build_oriented_graph(M, init_orientation)
    R, C = G['R'], G['C']
    free_cells = G['free_cells']
    start_cell = G['start_cell']
    state_id, id_state = G['state_id'], G['id_state']
    start_state = G['start_state']
    oriented_arcs = G['oriented_arcs']
    cell_arcs = G['cell_arcs']

    if start_state is None:
        raise ValueError("Initial orientation at start cell does not define a valid start state.")

    V = len(free_cells)
    bigM_t = 4 * R * C  # safe upper bound on times an oriented arc could be used
    bigM_f = V - 1      # commodity flow upper bound per arc

    solver = pywraplp.Solver.CreateSolver('CBC')
    if solver is None:
        raise RuntimeError('OR-Tools CBC solver not available')
    if time_limit is not None:
        solver.SetTimeLimit(int(time_limit * 1000))  # ms

    # Variables
    # t[a] integer multiplicity for oriented arc a
    t = {}
    for idx_a, (u, v, ucell, vcell, o_from, o_to, cost) in enumerate(oriented_arcs):
        t[idx_a] = solver.IntVar(0, bigM_t, f"t_{idx_a}")

    # z[n] end-state indicator (binary) for each oriented state
    z = {}
    for n in range(G['N_states']):
        z[n] = solver.BoolVar(f"z_{n}")

    # y_cell[(i,j)->(ni,nj)] binary arc activation for SCF connectivity
    y = {}
    for (u_cell, v_cell) in cell_arcs:
        y[(u_cell, v_cell)] = solver.BoolVar(f"y_{u_cell}_{v_cell}")

    # f_cell[(i,j)->(ni,nj)] continuous flow for SCF
    f = {}
    for (u_cell, v_cell) in cell_arcs:
        f[(u_cell, v_cell)] = solver.NumVar(0.0, bigM_f, f"f_{u_cell}_{v_cell}")

    # Objective: minimize sum(cost * t)
    objective_terms = []
    for idx_a, (u, v, ucell, vcell, o_from, o_to, cost) in enumerate(oriented_arcs):
        objective_terms.append(cost * t[idx_a])
    solver.Minimize(solver.Sum(objective_terms))

    # Coverage: each free cell must be visited at least once
    for u_cell in free_cells:
        # sum of t over oriented arcs entering or leaving any oriented state of this cell >= 1
        cover_terms = []
        for idx_a, (u, v, uc, vc, o_from, o_to, cost) in enumerate(oriented_arcs):
            if uc == u_cell or vc == u_cell:
                cover_terms.append(t[idx_a])
        solver.Add(solver.Sum(cover_terms) >= 1)

    # # Strong coverage: each free cell must be entered AND exited
    # for u_cell in free_cells:
    #     in_terms = []
    #     out_terms = []

    #     for idx_a, (u, v, uc, vc, o_from, o_to, cost) in enumerate(oriented_arcs):
    #         if vc == u_cell:   # arc enters the cell
    #             in_terms.append(t[idx_a])
    #         if uc == u_cell:   # arc leaves the cell
    #             out_terms.append(t[idx_a])

    #     solver.Add(solver.Sum(in_terms) >= 1)   # must enter
    #     solver.Add(solver.Sum(out_terms) >= 1)  # must exit

    # Flow conservation on oriented states (Eulerian trail with single end)
    # out(n) - in(n) = 1 for start_state; = -z[n] for others; forbid start as end.
    solver.Add(z[start_state] == 0)
    for n in range(G['N_states']):
        out_terms = []
        in_terms = []
        for idx_a, (u, v, uc, vc, o_from, o_to, cost) in enumerate(oriented_arcs):
            if u == n:
                out_terms.append(t[idx_a])
            if v == n:
                in_terms.append(t[idx_a])
        if n == start_state:
            solver.Add(solver.Sum(out_terms) - solver.Sum(in_terms) == 1)
        else:
            solver.Add(solver.Sum(out_terms) - solver.Sum(in_terms) == - z[n])

    # Exactly one end oriented state
    solver.Add(solver.Sum(z[n] for n in range(G['N_states'])) == 1)

    # Link oriented t-arcs to cell-level activation y (for SCF)
    # For each cell arc (i->j), associate all oriented arcs whose (ucell,vcell)=(i,j)
    arcs_by_cell = defaultdict(list)
    for idx_a, (u, v, ucell, vcell, o_from, o_to, cost) in enumerate(oriented_arcs):
        arcs_by_cell[(ucell, vcell)].append(idx_a)

    for (ucell, vcell), idx_list in arcs_by_cell.items():
        # If any oriented t uses this cell arc, y must be 1; and t must be 0 if y=0
        solver.Add(solver.Sum(t[i] for i in idx_list) >= y[(ucell, vcell)])
        solver.Add(solver.Sum(t[i] for i in idx_list) <= bigM_t * y[(ucell, vcell)])

    # SCF connectivity on cell graph to ensure all cells are in one component with the start
    # Capacity: f <= (V-1) * y
    for (ucell, vcell) in cell_arcs:
        solver.Add(f[(ucell, vcell)] <= bigM_f * y[(ucell, vcell)])

    # Flow balance: start_cell supplies V-1, every other free cell demands 1
    for u_cell in free_cells:
        out_f = [f[(ucell, vcell)] for (ucell, vcell) in cell_arcs if u_cell == ucell]
        in_f  = [f[(ucell, vcell)] for (ucell, vcell) in cell_arcs if u_cell == vcell]
        if u_cell == start_cell:
            solver.Add(solver.Sum(out_f) - solver.Sum(in_f) == V - 1)
        else:
            solver.Add(solver.Sum(in_f) - solver.Sum(out_f) == 1)

    # Solve
    status = solver.Solve()
    if status not in (pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE):
        raise RuntimeError(f"No feasible solution (status={status})")

    # Extract solution
    t_sol = {}
    for idx_a in t:
        val = int(round(t[idx_a].solution_value()))
        if val > 0:
            t_sol[idx_a] = val

    z_sol = {n: int(round(z[n].solution_value())) for n in z}
    end_states = [n for n, val in z_sol.items() if val == 1]
    end_state = end_states[0] if end_states else None

    return {
        't_sol': t_sol,
        'start_state': start_state,
        'end_state': end_state,
        'id_state': id_state,
        'oriented_arcs': oriented_arcs,
        'R': R, 'C': C,
    }


# ------------------------------------------------------------
# Reconstruct Eulerian trail from oriented arc multiplicities

def euler_trail_from_t(t_sol: dict, oriented_arcs: list, start_state: int):
    # Build adjacency with multiplicity on oriented states
    adj = defaultdict(list)
    for idx_a, mult in t_sol.items():
        (u, v, ucell, vcell, o_from, o_to, cost) = oriented_arcs[idx_a]
        for _ in range(mult):
            adj[u].append(v)

    # Hierholzer's algorithm (directed)
    local = {u: deque(vs) for u, vs in adj.items()}
    stack = [start_state]
    trail = []
    while stack:
        v = stack[-1]
        if v in local and local[v]:
            stack.append(local[v].popleft())
        else:
            trail.append(stack.pop())
    trail.reverse()
    return trail  # list of oriented state ids


# ------------------------------------------------------------
# Plotting with arrows and multi-visit labels like "1,3"

def plot_path(M: np.ndarray, state_path: list, id_state: dict, annotate=True):
    R, C = M.shape
    coords = [id_state[n] for n in state_path]  # (r,c,o)

    fig, ax = plt.subplots(figsize=(C, R))
    cmap = {0: "white", 1: "black", 2: "lightgray", 3: "black"}

    # Draw grid
    for i in range(R):
        for j in range(C):
            ax.add_patch(plt.Rectangle((j, R-1-i), 1, 1,
                                       facecolor=cmap[int(M[i, j])],
                                       edgecolor="gray"))

    # Draw arrows and collect visit indices per CELL (ignore orientation)
    visits = defaultdict(list)
    for k, (r, c, o) in enumerate(coords):
        visits[(r, c)].append(k)

    for k in range(len(coords) - 1):
        r1, c1, o1 = coords[k]
        r2, c2, o2 = coords[k + 1]
        x1, y1 = c1 + 0.5, R - 1 - r1 + 0.5
        x2, y2 = c2 + 0.5, R - 1 - r2 + 0.5
        ax.annotate("",
                    xy=(x2, y2), xycoords="data",
                    xytext=(x1, y1), textcoords="data",
                    arrowprops=dict(arrowstyle="->", color="blue", lw=2))

    # Highlight start and end cells
    if coords:
        ax.add_patch(plt.Rectangle((coords[0][1], R - 1 - coords[0][0]), 1, 1,
                                   facecolor="green", alpha=0.5))
        ax.add_patch(plt.Rectangle((coords[-1][1], R - 1 - coords[-1][0]), 1, 1,
                                   facecolor="red", alpha=0.5))

    # Annotate all visits per cell as "i,j,k"
    if annotate:
        for (r, c), idxs in visits.items():
            x, y = c + 0.5, R - 1 - r + 0.5
            label = ",".join(str(i) for i in idxs)
            ax.text(x, y, label, fontsize=12, fontweight="bold",
                    ha='center', va='center', color="darkred")

    ax.set_xlim(0, C)
    ax.set_ylim(0, R)
    ax.set_xticks(range(C + 1))
    ax.set_yticks(range(R + 1))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_aspect('equal')
    plt.grid(True, color='gray', linewidth=0.5)
    plt.tight_layout()
    plt.show()


# ------------------------------------------------------------
# Example main

def milp_main(npy_path: str = "map.npy", initial_orientation: int = 1, time_limit: int | None = None):
    M = np.load(npy_path)
    sol = solve_oriented_milp(M, initial_orientation, time_limit=time_limit)

    t_sol = sol['t_sol']
    state_path = euler_trail_from_t(t_sol, sol['oriented_arcs'], sol['start_state'])
    total_cost = 0.0
    total_steps = 0
    total_turn = 0.0
    for idx_a, mult in t_sol.items():
        _, _, _, _, o_from, o_to, cost = sol['oriented_arcs'][idx_a]
        total_cost += cost * mult
        total_steps += 1 * mult
        total_turn += (cost - 1.0) * mult

    print("Total cost:", round(total_cost, 3))
    print("  steps:", total_steps, " turn penalty:", round(total_turn, 3))

    # Pretty-print ordered path (r,c,o)
    print("Ordered oriented states:")
    path = list()
    path_with_ori = list()
    for k, sid in enumerate(state_path):
        r, c, o = sol['id_state'][sid]
        print(f"{k:3d}: ({r},{c})  o={o}")
        path.append((r,c))
        path_with_ori.append((r,c,o))


    # Plot
    # plot_path(M, state_path, sol['id_state'], annotate=True)
    # print(path)
    return path, path_with_ori


if __name__ == "__main__":
    import sys
    npy = sys.argv[1] if len(sys.argv) >= 2 else "maps.npy"
    init_o = int(sys.argv[2]) if len(sys.argv) >= 3 else 1
    tlim = int(sys.argv[3]) if len(sys.argv) >= 4 else None
    path = milp_main(npy, init_o, tlim)

    print(f"path: {path}")

