import numpy as np
import matplotlib.pyplot as plt
from main import main

def batch_run(num_maps: int, prefix="maps_10x10_", init_o=1, tlim=None):
    all_paths = []
    for i in range(1, num_maps + 1):
        filename = f"{prefix}{i}.npy"
        print(f"\n=== Solving {filename} ===")
        path = main(filename, init_o, tlim)
        all_paths.append(path)
    return all_paths

def plot_global_paths(global_map, all_paths, extra_points=None):
    plt.figure(figsize=(10, 10))
    plt.imshow(global_map == 3, cmap="gray_r", origin="upper")  # obstacles in black
    plt.grid(True, which="both")
    plt.xticks(range(global_map.shape[1]))
    plt.yticks(range(global_map.shape[0]))

    colors = plt.cm.tab10(np.linspace(0, 1, len(all_paths)))

    for idx, path in enumerate(all_paths):
        if len(path) > 1:
            xs, ys = zip(*[(c, r) for r, c in path])  # swap (row,col) -> (x,y)
            plt.plot(xs, ys, marker="o", markersize=3,
                     linewidth=2, color=colors[idx], label=f"UAV Path {idx+1}")

            # Draw arrows between consecutive points
            for (x0, y0), (x1, y1) in zip(zip(xs, ys), zip(xs[1:], ys[1:])):
                plt.arrow(x0, y0, x1 - x0, y1 - y0,
                          head_width=0.2, head_length=0.2,
                          fc=colors[idx], ec=colors[idx], length_includes_head=True)

            # Mark start (S) and end (E)
            plt.text(xs[0], ys[0]-0.2, f"S{idx+1}", color=colors[idx],
                     fontsize=13, fontweight="bold", ha="center", va="center")
            plt.text(xs[-1], ys[-1]-0.2, f"E{idx+1}", color=colors[idx],
                     fontsize=13, fontweight="bold", ha="center", va="center")
            
    # Plot extra points if provided
    if extra_points:
        xp, yp = zip(*extra_points)
        plt.scatter(xp, yp, marker="x", color="black", s=100, label="UGV Points")

    # Move legend outside
    plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left")
    plt.gca().invert_yaxis()  # match matrix indexing
    plt.tight_layout()
    plt.show()




if __name__ == "__main__":
    # load global map
    global_map = np.load("map_10x10_0.npy")

    ugv_points = [(-0.106, 0.789), (3.472, 1.055), (4.657, 4.343), (7.183, 4.633), (7.485, 7.485), (3.472, 7.945), (-0.106, 8.211), (1.683, 4.367), (7.485, 1.515)]
    
    # paths = [[(0, 0), (0, 1), (0, 2), (0, 3), (0, 2), (1, 2), (1, 1), (1, 0)], [(2, 0), (3, 0), (4, 0), (5, 0), (5, 1), (5, 2), (4, 2), (4, 3), (3, 3), (4, 3), (4, 2), (4, 1), (3, 1), (2, 1), (2, 2)], [(2, 3), (1, 3), (1, 4), (0, 4), (0, 5), (1, 5), (2, 5), (2, 4), (3, 4), (3, 5), (3, 6), (2, 6), (1, 6)], [(4, 4), (4, 5), (4, 6), (5, 6), (5, 5)], [(5, 4), (6, 4), (7, 4), (7, 5), (6, 5), (6, 6), (7, 6), (8, 6), (8, 5), (9, 5), (9, 4), (8, 4), (8, 3), (7, 3), (6, 3)], [(6, 2), (7, 2), (8, 2), (8, 1), (7, 1), (6, 1), (6, 0), (7, 0), (8, 0), (9, 0), (9, 1), (9, 2), (9, 3)], [(6, 7), (6, 8), (6, 9), (7, 9), (8, 9), (9, 9), (9, 8), (8, 8), (7, 8), (7, 7), (8, 7), (9, 7), (9, 6)], [(5, 7), (5, 8), (5, 9), (4, 9), (4, 8), (4, 7), (3, 7), (2, 7), (3, 7), (3, 8), (3, 9), (2, 9)], [(2, 8), (1, 8), (0, 8), (0, 9), (1, 9), (1, 8), (1, 7), (0, 7), (0, 6)]]
    paths = batch_run(9)  # adjust number of maps
    print("Finished batch run.")
    print(f"Collected {len(paths)} paths.")

    print(paths)

    # Plot them together
    plot_global_paths(global_map, paths, extra_points=ugv_points)
