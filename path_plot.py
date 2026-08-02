import matplotlib.pyplot as plt
from config import exp
import yaml

filename = f"experiments/run_{exp}/config_{exp}.yaml"

with open(filename, "r") as f:
    config = yaml.safe_load(f)

grid_size = config["grid_size"]
flat_waypoints = config["uav_waypoints"]
obstacles = config["obstacles"]

plt.figure(figsize=(10, 10))
visited = set()

for i in range(len(flat_waypoints) - 1):
    x1, y1 = flat_waypoints[i]
    x2, y2 = flat_waypoints[i + 1]

    if (x1, y1) == (x2, y2):
        continue

    dx = x2 - x1
    dy = y2 - y1

    arrow_scale = 0.5
    dx_scaled = dx * arrow_scale
    dy_scaled = dy * arrow_scale

    start_x = x1 + 0.5 - dx_scaled / 2
    start_y = y1 + 0.5 - dy_scaled / 2

    if (x1, y1) in visited:
        offset = 0.25
        if dx != 0:
            start_y += offset
        if dy != 0:
            start_x += offset
        color = 'orange'
    else:
        color = 'blue'

    plt.arrow(start_x, start_y,
              dx_scaled, dy_scaled,
              head_width=0.15,
              length_includes_head=True,
              color=color)

    visited.add((x1, y1))

# Plot obstacles
obs_x = [x + 0.5 for x, y in obstacles]
obs_y = [y + 0.5 for x, y in obstacles]
plt.scatter(obs_x, obs_y, marker='s', s=150, color='black')

# Mark Start and End
start = flat_waypoints[0]
end = flat_waypoints[-1]

plt.text(start[0] + 0.25, start[1] + 0.5, 'S',
         ha='center', va='center', fontsize=16, fontweight='bold', color='green')

plt.text(end[0] + 0.5, end[1] + 0.5, 'E',
         ha='center', va='center', fontsize=16, fontweight='bold', color='black')

# Grid formatting
plt.xlim(0, grid_size)
plt.ylim(0, grid_size)
plt.xticks(range(grid_size+1))
plt.yticks(range(grid_size+1))
plt.grid(True)
plt.gca().set_aspect('equal')

plt.title("UAV Path")

plt.savefig(f'experiments/run_{exp}/uav_path_{exp}.png')
plt.show()