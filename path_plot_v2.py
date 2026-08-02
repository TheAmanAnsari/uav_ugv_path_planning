import matplotlib.pyplot as plt_2
from matplotlib.lines import Line2D
from collections import defaultdict

def save_uav_path(uav_path, obstacles, grid_size, den, exp, folder):
    plt_2.figure(figsize=(11, 10))
    visit_count = defaultdict(int)

    colors = [
        "blue",      # 1st visit
        "orange",    # 2nd visit
        "red",       # 3rd visit
        "green",     # 4th visit
        "purple",    # 5th visit
        "brown",     # 6th visit
        "magenta",   # 7th visit
        "cyan",      # 8th visit
        "black"      # 9th+ visit
    ]

    for i in range(len(uav_path) - 1):
        x1, y1 = uav_path[i]
        x2, y2 = uav_path[i + 1]

        if (x1, y1) == (x2, y2):
            continue

        dx = x2 - x1
        dy = y2 - y1

        arrow_scale = 0.5
        dx_scaled = dx * arrow_scale
        dy_scaled = dy * arrow_scale

        start_x = x1 + 0.5 - dx_scaled / 2
        start_y = y1 + 0.5 - dy_scaled / 2

        # Number of previous visits to this cell
        count = visit_count[(x1, y1)]

        # Choose color
        color = colors[min(count, len(colors) - 1)]

        # Offset each revisit
        offset = 0.15 * count
        if dx != 0:      # horizontal arrow
            start_y += offset
        if dy != 0:      # vertical arrow
            start_x += offset

        plt_2.arrow(
            start_x,
            start_y,
            dx_scaled,
            dy_scaled,
            head_width=0.15,
            length_includes_head=True,
            color=color,
        )

        visit_count[(x1, y1)] += 1

    # Plot obstacles
    obs_x = [x + 0.5 for x, y in obstacles]
    obs_y = [y + 0.5 for x, y in obstacles]
    plt_2.scatter(obs_x, obs_y, marker='s', s=150, color='black')

    # Mark Start and End
    start = uav_path[0]
    end = uav_path[-1]

    plt_2.text(start[0] + 0.25, start[1] + 0.5, 'S',
            ha='center', va='center', fontsize=16, fontweight='bold', color='green')

    plt_2.text(end[0] + 0.5, end[1] + 0.5, 'E',
            ha='center', va='center', fontsize=16, fontweight='bold', color='black')

    # Grid formatting
    plt_2.xlim(0, grid_size)
    plt_2.ylim(0, grid_size)
    plt_2.xticks(range(grid_size+1))
    plt_2.yticks(range(grid_size+1))
    plt_2.grid(True)

    legend_handles = [
        Line2D([0], [0], color='blue', lw=3, label='1st visit'),
        Line2D([0], [0], color='orange', lw=3, label='2nd visit'),
        Line2D([0], [0], color='red', lw=3, label='3rd visit'),
        Line2D([0], [0], color='green', lw=3, label='4th visit'),
    ]

    plt_2.legend(
            handles=legend_handles, 
            loc='upper right',
            bbox_to_anchor=(1, 1),
            bbox_transform=plt_2.gcf().transFigure,
            title='Arrow Color'
    )

    plt_2.gca().invert_yaxis()
    plt_2.gca().set_aspect('equal')

    plt_2.title("UAV Path")

    filepath = f"{folder}/density_{den:02d}%/run_{exp:03d}/uav_path_{exp:03d}.png"
    plt_2.savefig(filepath)
    plt_2.close()
    # plt_2.show()

if __name__ == '__main__':
    save_uav_path()