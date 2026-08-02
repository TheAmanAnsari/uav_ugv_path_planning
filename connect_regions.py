import numpy as np
from collections import deque


# მოძრაობ directions (4-connectivity)
DIRS = [(1,0), (-1,0), (0,1), (0,-1)]


def find_start(grid):
    pos = np.argwhere(grid == 2)
    if len(pos) == 0:
        raise ValueError("Start position (2) not found")
    return tuple(pos[0])


def bfs_component(grid, start):
    rows, cols = grid.shape
    visited = set()
    q = deque([start])
    visited.add(start)

    while q:
        r, c = q.popleft()

        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if (nr, nc) not in visited and grid[nr, nc] in [0, 2]:
                    visited.add((nr, nc))
                    q.append((nr, nc))

    return visited


def get_disconnected_free_cells(grid, main_comp):
    all_free = set(map(tuple, np.argwhere(grid == 0)))
    return all_free - main_comp


def find_path_to_component(grid, main_comp, target_cells):
    rows, cols = grid.shape
    visited = set(main_comp)
    q = deque()

    # multi-source BFS
    for cell in main_comp:
        q.append((cell, []))

    while q:
        (r, c), path = q.popleft()

        if (r, c) in target_cells:
            return path + [(r, c)]

        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc

            if 0 <= nr < rows and 0 <= nc < cols:
                if (nr, nc) not in visited and grid[nr, nc] != 3:
                    visited.add((nr, nc))
                    q.append(((nr, nc), path + [(r, c)]))

    return None


def connect_regions(grid):
    start = find_start(grid)
    main_comp = bfs_component(grid, start)

    disconnected = get_disconnected_free_cells(grid, main_comp)

    while disconnected:
        path = find_path_to_component(grid, main_comp, disconnected)

        if path is None:
            print("No valid path exists to connect all regions.")
            break

        # carve corridor
        for r, c in path:
            if grid[r, c] == 1:
                grid[r, c] = 0

        # recompute components
        main_comp = bfs_component(grid, start)
        disconnected = get_disconnected_free_cells(grid, main_comp)

    return grid


if __name__ == "__main__":

    grid = np.array([
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,3,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,3,3,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,3,3,3,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,3,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,2,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,0,1,1,1,1,1,1,1,1,1,1,1,1],
        [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1],
        [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1],
        [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1]
    ])

    print("Original Map:\n")
    print(grid)

    updated_grid = connect_regions(grid)

    print("\nConnected Map:\n")
    print(updated_grid)
