import numpy as np

def create_global_map(grid_size, obstacles, den, exp, folder='experiments_0'):
    n = grid_size
    np_arr = np.zeros((n, n), dtype=int)

    for (x, y) in obstacles:
        np_arr[x][y] = 3

    global_map_filename = f"{folder}/density_{den:02d}%/run_{exp:03d}/global_map_{exp:03d}.npy"
    np.save(global_map_filename, np_arr)
    # print(f"np_arr: \n{np_arr}")

    return global_map_filename

def store_local_map(map, grid_size, idx, den, exp, folder="experiments_0"):
    filename = f"{folder}/density_{den:02d}%/run_{exp:03d}/local_maps/maps_{grid_size}x{grid_size}_{idx+1}.npy"
    np.save(filename, map)
