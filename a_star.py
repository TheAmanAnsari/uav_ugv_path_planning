import heapq
import numpy as np

def heuristic(cell, targets):
    x, y = cell
    # Manhattan distance to nearest target
    return min(abs(x - tx) + abs(y - ty) for tx, ty in targets)

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def astar_nearest_target_with_path(grid, start, targets):
    rows, cols = grid.shape
    targets = set(targets)

    open_list = []
    heapq.heappush(open_list, (0, 0, start))

    g_cost = {start: 0}
    came_from = {}
    visited = set()

    # 4-connected motion
    neighbors = [(-1,0), (1,0), (0,-1), (0,1)]

    while open_list:
        f, g, current = heapq.heappop(open_list)

        if current in visited:
            continue
        visited.add(current)

        # Stop as soon as we reach any target
        if current in targets:
            path = reconstruct_path(came_from, current)
            return current, path, g

        cx, cy = current

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            neighbor = (nx, ny)

            # Bounds check
            if not (0 <= nx < rows and 0 <= ny < cols):
                continue

            # Obstacle check
            if grid[nx, ny] == 3:
                continue

            new_cost = g + 1

            if neighbor not in g_cost or new_cost < g_cost[neighbor]:
                g_cost[neighbor] = new_cost
                came_from[neighbor] = current

                h = heuristic(neighbor, targets)
                heapq.heappush(open_list, (new_cost + h, new_cost, neighbor))

    return None, None, float("inf")


if __name__ == '__main__':
    # grid = np.array([
    # [0,0,0,0,0,0,0,0,0,0],
    # [0,0,0,0,0,0,0,0,0,0],
    # [0,0,3,0,0,0,0,0,0,0],
    # [0,0,0,0,0,3,3,0,0,0],
    # [0,0,0,0,0,0,0,0,0,0],
    # [0,0,0,0,0,0,0,0,0,0],
    # [0,0,3,3,3,0,0,0,0,0],
    # [0,0,0,0,3,0,0,0,0,0],
    # [0,0,0,0,0,0,0,0,0,0],
    # [0,0,0,0,0,0,0,0,0,0]
    # ])

    grid_size = 20
    grid = np.zeros((grid_size, grid_size), dtype=int)

    obstacles = [(2, 2), (3, 5), (3, 6), (6, 2), (6, 3), (6, 4), (7, 4)]

    for (x, y) in obstacles:
        grid[x][y] = 3

    # start = (1, 0)
    # targets = [(2, 0), (2, 1), (3, 0), (3, 1), (3, 2), (3, 3), (4, 0), (4, 1), (4, 2), (4, 3), (5, 0), (5, 1), (5, 2)]

    start = (8, 13)
    targets = [(15, 8), (15, 9), (16, 8), (16, 9), (16, 10), (16, 11), (17, 8), (17, 9), (17, 10), (17, 11), (18, 8), (18, 9), (18, 10), (18, 11), (19, 7), (19, 8), (19, 9), (19, 10), (19, 11), (19, 12)]

    closest, path, cost = astar_nearest_target_with_path(grid, start, targets)

    print("Closest target:", closest)
    print("Path:", path)
    print("Cost:", cost)

