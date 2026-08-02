import matplotlib.pyplot as plt_3

def save_2d_map(grid_size, obstacles, den, exp, folder):

    fig, ax = plt_3.subplots(figsize=(10, 10))

    # Draw grid lines (black)
    for x in range(grid_size + 1):
        ax.plot([x, x], [0, grid_size], color='black', linewidth=1)

    for y in range(grid_size + 1):
        ax.plot([0, grid_size], [y, y], color='black', linewidth=1)

    # Draw obstacles
    for (x, y) in obstacles:
        rect = plt_3.Rectangle((x, y), 1, 1, color='black')
        ax.add_patch(rect)

    # Axis setup
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_aspect('equal')

    ax.invert_yaxis()

    # Set ticks at every 1 unit
    ax.set_xticks(range(0, grid_size + 1, 1))
    ax.set_yticks(range(0, grid_size + 1, 1))

    # Show labels (important for debugging coordinates)
    ax.tick_params(labelbottom=True, labelleft=True)

    plt_3.title(f"{grid_size}x{grid_size} Environment with {den}% Obstacle Density", pad=10)

    filepath = f"{folder}/density_{den:02d}%/run_{exp:03d}/map_{exp:03d}.png"

    plt_3.savefig(filepath)
    plt_3.close()
    # plt_3.show()

if __name__ == "__main__":
    save_2d_map()