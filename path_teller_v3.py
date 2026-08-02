import networkx as nx

def path_teller(A, ugv_path, reachable_cells, M):
    """
    Plan UAV path from A following the UGV path.

    Parameters
    ----------
    A               : UAV start cell
    ugv_path        : [X1, X2, ..., Xn] — fixed UGV path
    reachable_cells : {Xi: [valid UAV cells at step i]}
    M               : UAV graph

    Returns
    -------
    uav_path  — sequence of UAV cells
    ugv_path  — expanded UGV path (UGV waits while UAV moves)
    both of equal length
    """

    uav_path_full = [A]
    ugv_path_full = [ugv_path[0]]
    uav_cur = A

    for i in range(len(ugv_path) - 1):
        Xi      = ugv_path[i]
        Xi_next = ugv_path[i + 1]

        rc_curr = set(reachable_cells[Xi])
        rc_next = set(reachable_cells[Xi_next])

        # step 1: find intersection
        intersection = rc_curr & rc_next

        if len(intersection) == 0:
            # no common cells — UGV moves, UAV stays
            uav_path_full.append(uav_cur)
            ugv_path_full.append(Xi_next)
            continue

        # step 2: pick target from intersection
        # closest cell in intersection to uav_cur
        target = min(
            intersection,
            key=lambda c: abs(c[0] - uav_cur[0]) + abs(c[1] - uav_cur[1])
        )

        # step 3: plan full path from uav_cur to target
        # restricted to reachable_cells[Xi] (tether constraint while UGV at Xi)
        valid_nodes = rc_curr | {uav_cur, target}
        subgraph = M.subgraph(valid_nodes)

        try:
            local_path = nx.shortest_path(subgraph, uav_cur, target)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            # no path found — UAV stays, UGV moves
            uav_path_full.append(uav_cur)
            ugv_path_full.append(Xi_next)
            continue

        # step 4: UGV waits at Xi for each UAV step
        # then UGV moves to Xi_next, UAV stays at target
        for step in local_path[1:]:
            uav_path_full.append(step)
            ugv_path_full.append(Xi)      # UGV waits at Xi

        # UGV moves to Xi_next, UAV stays at target
        uav_cur = target
        uav_path_full.append(uav_cur)
        ugv_path_full.append(Xi_next)

    return uav_path_full, ugv_path_full