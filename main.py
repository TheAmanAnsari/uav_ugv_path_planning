import os
import networkx as nx
import ortools_hitting_set
import numpy as np

from config import tether_length, height, grid_size, o, d, printl, printdict
from geometry_utils import *
from plotting import (
    setup_plot,
    plot_possible_ugv_points,
    plot_grid_graph,
    plot_hitting_set_circles,
    plot_road_network_edges,
    plot_circles_for_gv_points,
    plot_ugv_coverage_rectangles,
    plot_lines,
    plot_obstacles,
    get_road_network,
    tether_map_plot
)
from uav_ugv_analysis import analyze_uav_ugv_coverage
from obstacle import obstacles, generate_obstacle_coord, calculate_vertices, top_or_base
from line_segement_intersection_v2 import *
from split_intersections import *
from voronoi import voronoi
from map_utils_v2 import create_global_map
from path_plot_v2 import save_uav_path
from map_2D import save_2d_map

import json
import yaml
from collections import defaultdict

def write_yaml(uav, ugv, ugv_nodes, obs, waypoints, den, exp, folder='experiments_0'):
    filename = f"{folder}/density_{den:02d}%/run_{exp:03d}/config_{exp:03d}.yaml"

    with open(filename, "w") as f:
        f.write(f"uav_waypoints: {uav}\n\n")
        f.write(f"ugv_waypoints: {ugv}\n\n")
        f.write(f"obstacles: {obs}\n\n")
        f.write(f"num_obstacles: {len(obs)}\n\n")
        f.write(f"uav_travel_cost: {travel_cost(uav)}  # meters\n\n")
        f.write(f"free_cells: {len(waypoints)}\n\n")
        f.write(f"visited_cells: {len(set(tuple(p) for p in uav))}\n\n")
        f.write(f"coverage_accuracy: {(len(set(tuple(p) for p in uav))/len(waypoints))*100}\n\n")
        f.write(f"selected_ugv_nodes: {ugv_nodes}\n\n")
        f.write(f"local_milps: {len(ugv_nodes)}\n\n")
        f.write(f"grid_size: {grid_size}         # meters\n\n")
        f.write(f"tether_length: {tether_length}      # meters\n\n")
        f.write(f"height: {height}             # meters\n\n")
        f.write(f"mission_time: {mission_time(uav)}    # seconds\n")
    print(f"Data written to {filename}")

def travel_cost(uav):
    cost = 0
    for i in range(len(uav)):
        if uav[i] == uav[i-1]:
            pass
        else:
            cost += 1
    return cost

def mission_time(uav):
    num_segments = len(uav) - 1
    time_per_segment = 6
    return num_segments * time_per_segment

def print_and_save_data(coverage_path, ugv_path, ugv_nodes, obs, waypoints, den, exp, folder='experiments_0'):
    COVERAGE_PATH = list(map(list, coverage_path))

    # print(f"COVERAGE_PATH: {COVERAGE_PATH}\n len(COVERAGE_PATH): {len(COVERAGE_PATH)}")
    # print(f"len of unique elements in COVERAGE_PATH: {len(set(coverage_path))}")

    print("-" * 20)

    UGV_PATH = list(map(list, ugv_path))

    # print(f"UGV_PATH: {UGV_PATH}\n len(UGV_PATH): {len(UGV_PATH)}")
    # print(f"len of unique elements in UGV_PATH: {len(set(ugv_path))}")

    # print("-" * 20)

    UGV_NODES = list(map(list, ugv_nodes))

    # print(f"UGV_NODES: {UGV_NODES}\n len(UGV_NODES): {len(UGV_NODES)}")

    # print("-" * 20)

    OBSTACLES = list(map(list, obs))
    # print(f"For OBSTACLES:\n{OBSTACLES}\n len(OBSTACLES): {len(OBSTACLES)}")

    save_uav_path(uav_path=COVERAGE_PATH, obstacles=OBSTACLES, grid_size=grid_size, den=den, exp=exp, folder=folder)

    write_yaml(uav=COVERAGE_PATH, ugv=UGV_PATH, ugv_nodes=UGV_NODES, obs=OBSTACLES, waypoints=waypoints, den=den, exp=exp, folder=folder)

def create_ugv_road_network_json(gvPoints, den, exp, folder):
    data = [
        {
            "from"  : list(k[0]),
            "to"    : list(k[1]),
            "points": [list(p) for p in v]
        }
        for k, v in gvPoints.items()
    ]

    ugv_points_filename = f"{folder}/density_{den:02d}%/run_{exp:03d}/ugv_points_{grid_size}x{grid_size}_{exp:03d}.json"
    with open(ugv_points_filename, "w") as f:
        json.dump(data, f, indent=2)

    return ugv_points_filename

def load_obstacles(den, exp, folder='obstacle_files_0'):
    obs_filename = f'{folder}/density_{den:02d}%/run_{exp:03d}/obstacle_{exp:03d}.yaml'

    with open(obs_filename, "r") as f:
        obs_yaml = yaml.safe_load(f)

    return list(map(tuple, obs_yaml['obstacles']))


def main(den, exp, folder):
    os.makedirs(f"{folder}/density_{den:02d}%/run_{exp:03d}", exist_ok=True)
    os.makedirs(f"{folder}/density_{den:02d}%/run_{exp:03d}/local_maps", exist_ok=True)

    Graph = nx.grid_2d_graph(grid_size, grid_size)
    waypoints =  list(Graph.nodes())

    obs = load_obstacles(den, exp)
    waypoints = [w for w in waypoints if w not in set(obs)]

    global_map_filename = create_global_map(grid_size=grid_size, obstacles=obs, den=den, exp=exp, folder=folder)

    save_2d_map(grid_size=grid_size, obstacles=obs, den=den, exp=exp, folder=folder)

    # print(f"waypoints: {waypoints} \nlen(waypoints): {len(waypoints)}")

    peripheral_nodes = generate_peripheral_nodes(o, d)
    road_network_edges = generate_non_cardinal_edges(peripheral_nodes)
    road_network = get_road_network()

    # Add peripheral nodes to the graph
    for p_node in peripheral_nodes:
        Graph.add_node(p_node)

    # # green_edges = generate_non_cardinal_edges(peripheral_nodes)

    # # Draw the grid graph
    pos = setup_plot(Graph)
    plot_grid_graph(Graph, pos, peripheral_nodes)
    # plot_road_network_edges(Graph, pos, road_network_edges)


    Lines = generate_lines(o, d)
    # print(f"len(Lines): {len(Lines)} \nLines(original):")
    # printl(Lines)

    result, intersection_points = split_all_intersections(Lines)
    Lines = result
    # print(f"len(Lines): {len(Lines)} \nLines(intersection_points):")
    # print(Lines)

    obstacles_coord = generate_obstacle_coord(obs)
    vertices = calculate_vertices(obstacles_coord)
    # print(f"len(vertices): {len(vertices)} \nvertices:")
    # printl(vertices)

    top_or_base_ = top_or_base(vertices)
    # print(f"len(top_or_base_): {len(top_or_base_)} \ntop_or_base_:")
    # printl(top_or_base_)

    base_vertices = []
    top_vertices = []
    for v in top_or_base_:
        base_vertices.append(v[0])
        top_vertices.append(v[1])

    # print(f"len(base_vertices): {len(base_vertices)} \nbase_vertices:")
    # printl(base_vertices)
    # print(f"len(top_vertices): {len(top_vertices)} \ntop_vertices:")
    # printl(top_vertices)

    base_vertices_line_segments = list()
    for v in base_vertices:
        base_vertices_line_segments.append([v[0], v[1]])
        base_vertices_line_segments.append([v[1], v[2]])
        base_vertices_line_segments.append([v[2], v[3]])
        base_vertices_line_segments.append([v[3], v[0]])

    # print(f"len(base_vertices_line_segments): {len(base_vertices_line_segments)} \nbase_vertices_line_segments:")
    # printl(base_vertices_line_segments)

    filtered_lines = [
        line for line in Lines
        if not any(segments_intersect(line, vline)['intersects'] for vline in base_vertices_line_segments)
    ]

    Lines = filtered_lines
    # print(f"len(Lines): {len(Lines)} \nLines(filtered_lines):")
    # printl(Lines)

    # print(Lines)

    plot_lines(Lines)
    plot_obstacles(obs)

    gvPoints, new_gvPoints = compute_gv_points_v2(Lines)
    # gvPoints = compute_gv_points(Lines)
    # print(f"gvPoints: \n{gvPoints} and len(gvPoints): {len(gvPoints)}")
    print("\n")
    # print(f"new_gvPoints: \n{new_gvPoints} and len(new_gvPoints): {len(new_gvPoints)}")
    # print("\n")

    ugv_points_filename = create_ugv_road_network_json(gvPoints=gvPoints, den=den, exp=exp, folder=folder)

    # # Plotting the possible UGV points 
    # plot_possible_ugv_points(new_gvPoints)

    radius = calculate_radius(tether_length, height)    # Radius of the circular base 

    # ugv_candidate_sets = generate_ugv_candidate_sets(waypoints, Lines, new_gvPoints, radius)
    ugv_candidate_sets, ugv_candidate_dict = generate_ugv_candidate_sets_v3(waypoints, gvPoints, radius, cuboids=vertices)
    # print(f"ugv_candidate_sets: \n{ugv_candidate_sets}")
    # print(f"len(ugv_candidate_sets): {len(ugv_candidate_sets)}")
    # print(f"ugv_candidate_dict:")
    # printdict(ugv_candidate_dict)
    print(f"len(ugv_candidate_dict): {len(ugv_candidate_dict)}")
    print("\n")
    # printl(ugv_candidate_sets)

    ugv_candidate_inverted = defaultdict(set)
    for k, values in ugv_candidate_dict.items():
        for v in values:
            ugv_candidate_inverted[v].add(k)
    
    ugv_candidate_inverted = {k: sorted(v) for k, v in ugv_candidate_inverted.items()}
    # printdict(ugv_candidate_inverted)

    min_hitting_set = ortools_hitting_set.hitting_set_with_ortools(ugv_candidate_sets)
    # type(min_hitting_set): <class 'set'>
    min_hitting_set.discard(None)

    ugv_dict = {key: ugv_candidate_inverted[key] for key in min_hitting_set if key in ugv_candidate_inverted}
    new_ugv_dict = voronoi(ugv_dict_inverted=ugv_dict, height=height)

    # plot_hitting_set_circles(min_hitting_set, radius)
    plot_ugv_coverage_rectangles(ugv_dict_inverted=new_ugv_dict)

    tether_map_plot(grid_size, tether_length, den=den, exp=exp, folder=folder)

    circles = list(min_hitting_set)
    # for (x, y) in circles:
    #     print(f"{(x, y)} : {ugv_candidate_inverted[(x, y)]}")

    # print(f"\ncircles: {circles}\n")
    # results = points_in_circle.points_in_multiple_circles(waypoints, circles, radius=radius)

    results = []
    for i, (x, y) in enumerate(circles):
        results.append((f"Circle {i + 1}: Center {(x, y)}", ugv_candidate_inverted[(x, y)]))

    '''
    print(f"results: {results}")
    print(f"type(results): {type(results)}, len(results): {len(results)}")        
    '''

    coverage_path, ugv_path, ugv_nodes = analyze_uav_ugv_coverage(
        results, grid_size, ugv_candidate_inverted, obstacles=obs,
        ugv_road_network_json=ugv_points_filename, global_map=global_map_filename,
        den=den, exp=exp, folder=folder
    )
    print_and_save_data(coverage_path, ugv_path, ugv_nodes, obs, waypoints, den, exp, folder)

    print(f"number of waypoints: {len(waypoints)}")

    '''
    print(f"\ncircle_centers: {circle_centers}")
    print(f"circle_waypoints: {circle_waypoints}")

    final_edges, final_gv_points = generate_final_road_edges_and_points(circle_centers, gvPoints, Lines)
    plot_possible_ugv_points(final_gv_points)
    plot_circles_for_gv_points(final_gv_points, radius)
    plot_road_network_edges(Graph, pos, final_edges)
    '''

    # Finalize and display the plot
    # finalize_plot(grid_size, tether_length, den=den, exp=exp, folder=folder)

# if __name__ == "__main__":
#     experiment_number = 11
#     density = 5
#     try:
#         main(den=density, exp=experiment_number, folder='z_experiments_0')
#     except Exception as e:
#         print(e)


if __name__ == "__main__":
    folder_name = 'experiments_0'

    obstacle_density = [5, 10, 15, 20, 25, 30]
    iterate_till = 25
    experiment_number = 1

    # for density in obstacle_density:      
    #     for i in range(iterate_till):
    #         try:
    #             main(den=density, exp=experiment_number, folder=folder_name)
    #         except Exception as e:
    #             print(e)

    #         experiment_number += 1
    #     experiment_number = 21

    avg_mission_time_list = list()
    std_mission_time_list = list()
    avg_accuracy_list = list()
    std_accuracy_list = list()
    avg_uav_travel_distance_list = list()
    std_uav_travel_distance_list = list()

    for density in obstacle_density:
        avg_mission_time = []
        std_mission_time = []
        avg_accuracy = []
        uav_travel_distance = []
        
        for i in range(iterate_till):
            filename = f"{folder_name}/density_{density:02d}%/run_{experiment_number:03d}/config_{experiment_number:03d}.yaml"

            with open(filename, "r") as f:
                data = yaml.safe_load(f)

            avg_mission_time.append(data["mission_time"])
            avg_accuracy.append(data["coverage_accuracy"])
            uav_travel_distance.append(data["uav_travel_cost"])

            experiment_number += 1
        
        experiment_number = 1

        avg_mission_time_list.append(np.mean(avg_mission_time))
        std_mission_time_list.append(np.std(avg_mission_time, ddof=1))
        avg_accuracy_list.append(np.mean(avg_accuracy))
        std_accuracy_list.append(np.std(avg_accuracy, ddof=1))
        avg_uav_travel_distance_list.append(np.mean(uav_travel_distance))
        std_uav_travel_distance_list.append(np.std(uav_travel_distance, ddof=1))


    print(f"avg_mission_time: {avg_mission_time_list}")
    print(f"std_mission_time: {std_mission_time_list}")
    print(f"avg_accuracy: {avg_accuracy_list}")
    print(f"std_accuracy: {std_accuracy_list}")
    print(f"avg_uav_travel_distance: {avg_uav_travel_distance_list}")
    print(f"std_uav_travel_distance: {std_uav_travel_distance_list}")
