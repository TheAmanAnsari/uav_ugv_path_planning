import networkx as nx
import matplotlib.pyplot as plt

def visualize_subgraph(M, subgraph, start=None, goal=None, path=None):
    """
    Visualize the UAV graph M with the restricted subgraph highlighted.
    
    Parameters
    ----------
    M        : full UAV graph
    subgraph : restricted subgraph
    start    : start node (optional)
    goal     : goal node (optional)
    path     : planned path (optional)
    """
    fig, ax = plt.subplots(figsize=(10, 10))

    # positions — (col, row) for correct x,y orientation
    pos = {node: (node[1], -node[0]) for node in M.nodes()}

    # draw full graph in light gray
    nx.draw_networkx_nodes(M, pos, node_size=20, 
                           node_color="lightgray", ax=ax)
    nx.draw_networkx_edges(M, pos, edge_color="lightgray", 
                           width=0.5, ax=ax)

    # draw subgraph nodes in blue
    nx.draw_networkx_nodes(subgraph, pos, node_size=40,
                           node_color="blue", ax=ax)
    nx.draw_networkx_edges(subgraph, pos, edge_color="blue",
                           width=1.0, ax=ax)

    # draw path in red if provided
    if path is not None:
        path_edges = list(zip(path[:-1], path[1:]))
        nx.draw_networkx_nodes(M, pos, nodelist=path,
                               node_size=60, node_color="red", ax=ax)
        nx.draw_networkx_edges(M, pos, edgelist=path_edges,
                               edge_color="red", width=2.0, ax=ax)

    # draw start and goal
    if start is not None:
        nx.draw_networkx_nodes(M, pos, nodelist=[start],
                               node_size=100, node_color="green", ax=ax)
    if goal is not None:
        nx.draw_networkx_nodes(M, pos, nodelist=[goal],
                               node_size=100, node_color="orange", ax=ax)

    # legend
    from matplotlib.lines import Line2D
    legend = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='lightgray', markersize=8, label='Full graph'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=8, label='Subgraph'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=8, label='Path'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=8, label='Start'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', markersize=8, label='Goal'),
    ]
    ax.legend(handles=legend, loc='upper right')
    ax.set_title("UAV Subgraph Visualization")
    plt.tight_layout()
    plt.show()


def path_teller(A, B, ugv_path, reachable_cells, M):
    """
    Plan UAV path from A to B given fixed UGV path.

    Parameters
    ----------
    A               : UAV start cell
    B               : UAV goal cell
    ugv_path        : [X1, X2, ..., Xn] — fixed UGV path
    reachable_cells : {Xi: [valid UAV cells at step i]}
    M               : UAV graph

    Returns
    -------
    uav_path, ugv_path — two lists of equal length
    """

    # ── Step 1: Identify non-empty UGV nodes ─────────────────────────────────
    non_empty = [X for X in ugv_path if len(reachable_cells[X]) > 0]

    # ── Step 2: Compute intersections between consecutive non-empty nodes ─────
    intersections = []
    for i in range(len(non_empty) - 1):
        common = set(reachable_cells[non_empty[i]]) & set(reachable_cells[non_empty[i + 1]])
        intersections.append((non_empty[i], non_empty[i + 1], common))

    # ── Step 3: Pick waypoints from intersections ─────────────────────────────
    waypoints = [A]

    for i, (Xi, Xi_next, common) in enumerate(intersections):
        if i < len(intersections) - 1:
            # pick cell in common closest to next intersection's common cells
            next_common = intersections[i + 1][2]
            waypoint = min(
                common,
                key=lambda c: min(
                    abs(c[0] - nc[0]) + abs(c[1] - nc[1]) for nc in next_common
                )
            )
        else:
            # last intersection — pick cell closest to B
            waypoint = min(
                common,
                key=lambda c: abs(c[0] - B[0]) + abs(c[1] - B[1])
            )
        waypoints.append(waypoint)

    waypoints.append(B)

    # ── Step 4: Plan local UAV path between consecutive waypoints ────────────
    full_uav_path = []

    for i in range(len(waypoints) - 1):
        wp_start = waypoints[i]
        wp_end   = waypoints[i + 1]

        # valid nodes = reachable_cells of current non-empty UGV node
        if i < len(non_empty):
            valid_nodes = set(reachable_cells[non_empty[i]])
        else:
            valid_nodes = set(reachable_cells[non_empty[-1]])

        # ensure start and end are in valid nodes
        valid_nodes.add(wp_start)
        valid_nodes.add(wp_end)

        # shortest path restricted to valid nodes
        local_path = restricted_shortest_path(wp_start, wp_end, M, valid_nodes)

        if local_path is None:
            print(f"  [!] No path from {wp_start} to {wp_end}")
            return None, None

        # avoid duplicating nodes at waypoint boundaries
        if full_uav_path:
            full_uav_path.extend(local_path[1:])
        else:
            full_uav_path.extend(local_path)

    # ── Step 5: Align UAV and UGV path lengths ────────────────────────────────
    aligned_ugv_path = ugv_path[:]

    # pad UGV path with Q if UAV path is longer
    while len(aligned_ugv_path) < len(full_uav_path):
        aligned_ugv_path.append(ugv_path[-1])

    # pad UAV path with B if UGV path is longer
    while len(full_uav_path) < len(aligned_ugv_path):
        full_uav_path.append(B)

    return full_uav_path, aligned_ugv_path


def restricted_shortest_path(start, goal, M, valid_nodes):
    """
    Shortest path on graph M restricted to valid_nodes using NetworkX.

    Parameters
    ----------
    start       : start node
    goal        : goal node
    M           : UAV graph
    valid_nodes : set of nodes UAV is allowed to visit

    Returns
    -------
    path as list of nodes, or None if not found
    """
    subgraph = M.subgraph(valid_nodes)

    # visualize
    visualize_subgraph(M, subgraph, start=start, goal=goal)

    try:
        return nx.shortest_path(subgraph, start, goal)
    except nx.NetworkXNoPath:
        print(f"  [!] No path from {start} to {goal} in restricted subgraph")
        return None
    except nx.NodeNotFound:
        print(f"  [!] Node not found in restricted subgraph")
        return None