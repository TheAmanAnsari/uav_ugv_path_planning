import numpy as np
from collections import deque

DIRS = [(1,0), (-1,0), (0,1), (0,-1)]


def find_start(grid):
    pos = np.argwhere(grid == 2)
    if len(pos) == 0:
        raise ValueError("Start position (2) not found")
    return tuple(pos[0])


def get_reachable_cells(grid, start):
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


def is_map_connected(grid):
    # print(f"[is_map_connected]: \n{grid}")
    start = find_start(grid)

    reachable = get_reachable_cells(grid, start)
    all_free = set(map(tuple, np.argwhere((grid == 0) | (grid == 2))))

    return reachable == all_free


def main():
    map_1 = np.array([
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,3,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,3,3,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,3,3,3,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,3,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,0,0,0,2,1,1,1,1,1,1,1],
        [1,1,1,0,0,0,0,0,1,1,1,1,1,1,1],
        [1,1,1,0,0,0,0,0,1,1,1,1,1,1,1],
        [1,1,1,0,0,0,0,0,1,1,1,1,1,1,1],
        [1,1,1,0,0,0,0,0,1,1,1,1,1,1,1]
    ])

    map_2 = np.array([
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

    print("Map 1 connected:", is_map_connected(map_1))
    print("Map 2 connected:", is_map_connected(map_2))

    # if is_map_connected(map_2):
    #     pass
    # else:
    #     print('else printed')


if __name__ == "__main__":
    main()