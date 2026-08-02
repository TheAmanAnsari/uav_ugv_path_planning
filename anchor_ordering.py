import json
import networkx as nx
import math


def build_ugv_graph(ugv_points: dict) -> nx.Graph:
    R = nx.Graph()
    for (A, B), intermediates in ugv_points.items():
        full_sequence = [A] + intermediates + [B]
        for i in range(len(full_sequence) - 1):
            R.add_edge(full_sequence[i], full_sequence[i + 1])
    return R


def load_ugv_points(json_file: str) -> dict:
    with open(json_file) as f:
        data = json.load(f)
    return {
        (tuple(entry["from"]), tuple(entry["to"])): [tuple(p) for p in entry["points"]]
        for entry in data
    }


def order_clusters(clusters: dict, json_file: str) -> list:
    """
    Orders anchor points using nearest neighbor heuristic
    based on road network distance.

    Parameters
    ----------
    clusters  : {anchor: [free_cells]} dictionary
    json_file : path to ugv_points json file

    Returns
    -------
    ordered list of anchor points to visit
    """
    # build road network graph
    ugv_points = load_ugv_points(json_file)
    R = build_ugv_graph(ugv_points)

    # # find first anchor — cluster containing (0, 0)
    # first_anchor = None
    # for anchor, cells in clusters.items():
    #     if (0, 0) in cells:
    #         first_anchor = anchor
    #         break

    # if first_anchor is None:
    #     raise ValueError("No cluster contains (0, 0)")

    # fixed depot = top-left road node
    depot = min(R.nodes, key=lambda p: (p[1], p[0]))

    # first cluster = closest anchor to the depot
    first_anchor = min(
        clusters.keys(),
        key=lambda a: nx.shortest_path_length(R, depot, a)
    )

    # nearest neighbor ordering on road network
    unvisited = list(clusters.keys())
    unvisited.remove(first_anchor)
    ordered = [first_anchor]
    current = first_anchor

    while unvisited:
        # find nearest unvisited anchor by road network distance
        next_anchor = min(
            unvisited,
            key=lambda a: nx.shortest_path_length(R, current, a)
        )
        ordered.append(next_anchor)
        unvisited.remove(next_anchor)
        current = next_anchor

    first_cluster = ordered[0]

    uav_start = min(
        clusters[first_cluster],
        key=lambda c: (c[1], c[0])
    )

    return ordered, uav_start