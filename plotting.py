import matplotlib.pyplot as plt_1
import matplotlib.patches as patches
from matplotlib.patches import Rectangle
import matplotlib.cm as cm
import networkx as nx

def setup_plot(Graph, figsize=(20, 15)):
    # fig, ax = plt_1.subplots(figsize=figsize)
    plt_1.figure(figsize=figsize)
    pos = {n: n for n in Graph.nodes()}
    return pos

def plot_possible_ugv_points(gvPoints):
    for points in gvPoints:
        x_coords, y_coords = zip(*points)
        plt_1.scatter(x_coords, y_coords, color='red', s=40)

def plot_grid_graph(Graph, pos, peripheral_nodes):
    # Draw main grid nodes
    nx.draw(Graph, pos, with_labels=True, node_size=30, node_color='skyblue', font_size=10, font_color='black', font_weight='bold', edge_color='gray', alpha=0.35)

    # Draw peripheral nodes
    nx.draw_networkx_nodes(Graph, pos, nodelist=peripheral_nodes, node_size=300, node_color='lightgreen')

def plot_hitting_set_circles(min_hitting_set, radius):
    for (x, y) in min_hitting_set:
        plt_1.scatter(x, y, color='red', marker='x', s=40)
        circle = plt_1.Circle((x, y), radius, color='red', linestyle='dotted', fill=False)
        plt_1.gca().add_patch(circle)

def plot_ugv_coverage_rectangles(ugv_dict_inverted):
    ax = plt_1.gca()

    # Choose a colormap with many distinct colors.
    # For up to 20 UGVs, tab20 works well.
    cmap = cm.get_cmap('tab20', len(ugv_dict_inverted))

    for i, (ugv_pos, covered_cells) in enumerate(ugv_dict_inverted.items()):
        color = cmap(i)

        x, y = ugv_pos

        # Plot the UGV center
        ax.scatter(x, y,
                   color=color,
                   marker='h',
                   s=100,
                   linewidths=2,
                   zorder=5)

        # Plot all covered cells
        for row, col in covered_cells:
            
            rect = Rectangle(
                (row - 0.45, col - 0.45),  # lower-left corner
                0.9,
                0.9,
                fill=False,
                edgecolor=color,
                linewidth=2.5,
                zorder=3
            )
            ax.add_patch(rect)

def plot_circles_for_gv_points(final_gv_points, radius):
    for points in final_gv_points:
        for (x, y) in points:
            plt_1.scatter(x, y, color='red', marker='x', s=40)
            circle = plt_1.Circle((x, y), radius, color='red', linestyle='dotted', fill=False)
            plt_1.gca().add_patch(circle)

def tether_map_plot(grid_size, tether_length, den, exp, folder):
    plt_1.scatter([], [], marker='>', label=f"grid size: {grid_size}x{grid_size}")
    plt_1.scatter([], [], marker='>', label=f"tether length: {tether_length} unit")
    plt_1.scatter([], [], marker='h', color='red', s=40, label="Ground Vehicles Points")
    plt_1.scatter([], [], color='lightblue', s=100, label="UAV Points")

    plt_1.gca().invert_yaxis()

    plt_1.title(f"{grid_size}x{grid_size} Environment with {den}% Obstacle Density", pad=20)
    plt_1.legend(
        loc='upper right', 
        bbox_to_anchor=(1, 1), 
        bbox_transform=plt_1.gcf().transFigure
    )
    plt_1.axis('equal')

    filepath = f"{folder}/density_{den:02d}%/run_{exp:03d}/tether_map_{exp:03d}.png"
    plt_1.savefig(filepath)
    plt_1.close()
    # plt_1.show()

def plot_road_network_edges(Graph, pos, edges):
    nx.draw_networkx_edges(Graph, pos, edges)

def plot_lines(lines):
    for (x1, y1), (x2, y2) in lines:
        plt_1.plot([x1, x2], [y1, y2], color='green', linewidth=1)

def plot_obstacles(obstacles):
    for (x, y) in obstacles:
        # rect = patches.Rectangle((x - 0.25, y - 0.25), 0.5, 0.5, color='black')
        rect = patches.Rectangle((x-0.45, y-0.45), 0.9, 0.9, color='black')
        plt_1.gca().add_patch(rect)

def get_road_network():
    nodes = [(3,9), (6,9), (9,9), (6,6), (13,9), (16,9), (16,6), (19,6), (19,12), (14,12)]
    edges = [
        [(3,9), (6,9)],
        [(6,9), (9,9)],
        [(6,9), (6,6)],
        [(6,6), (16,6)],
        [(16,6), (16,9)],
        [(16,9), (13,9)],
        [(16,6), (19,6)],
        [(19,6), (19,12)],
        [(19,12), (14,12)],
    ]
    return nodes, edges
