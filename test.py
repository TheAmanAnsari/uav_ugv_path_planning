import numpy as np
grid_size = 20
np_map = np.zeros((grid_size, grid_size), dtype=int)

obstacles = [(2, 2), (3, 5), (3, 6), (6, 2), (6, 3), (6, 4), (7, 4)]

for (x, y) in obstacles:
    np_map[x][y] = 3

print(np_map)
print(type(np_map))
idx = 0
filename = f"Maps/map_{grid_size}x{grid_size}_{idx}.npy"
np.save(filename, np_map)