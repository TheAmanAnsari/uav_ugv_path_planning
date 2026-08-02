import networkx as nx
from collections import deque

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
    uav_path  — sequence of UAV cells, equal length to ugv_path
                last cell = B (entry point for next cluster MILP)
    ugv_path  — unchanged UGV path
    """

    uav_path = [A]
    uav_cur  = A

    for i in range(len(ugv_path) - 1):
        Xi      = ugv_path[i]
        Xi_next = ugv_path[i + 1]

        rc_curr = set(reachable_cells[Xi])
        rc_next = set(reachable_cells[Xi_next])

        # if next UGV node has empty reachable cells
        # UAV stays at current position
        if len(rc_next) == 0:
            uav_path.append(uav_cur)
            continue

        # target = intersection of current and next reachable cells
        # UAV moves toward this intersection
        intersection = rc_curr & rc_next

        if len(intersection) == 0:
            # no common cells — UAV stays at current position
            uav_path.append(uav_cur)
            continue

        # pick target — cell in intersection closest to uav_cur
        target = min(
            intersection,
            key=lambda c: abs(c[0] - uav_cur[0]) + abs(c[1] - uav_cur[1])
        )

        # build subgraph restricted to current reachable cells
        # add uav_cur in case it's not in rc_curr
        valid_nodes = rc_curr | {uav_cur, target}
        subgraph = M.subgraph(valid_nodes)

        # plan path from uav_cur to target using nx.shortest_path
        try:
            local_path = nx.shortest_path(subgraph, uav_cur, target)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            # if no path found, UAV stays at current position
            uav_path.append(uav_cur)
            continue

        # UAV moves one step along local path
        if len(local_path) > 1:
            uav_cur = local_path[1]
        
        uav_path.append(uav_cur)

    return uav_path, ugv_path