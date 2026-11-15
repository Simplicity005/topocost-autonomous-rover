import heapq
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d
from src.config import (
    ARENA_SIZE_M, GRID_SIZE, RESOLUTION_M, ROVER_MASS_KG, GRAVITY, FRICTION_COEFF,
    world_to_grid, grid_to_world
)
from src.costmap import compute_slope_grid, calculate_step_energy

# 8-connected grid neighbors: (d_row, d_col, distance_multiplier)
NEIGHBORS = [
    (-1,  0, 1.0), (1,  0, 1.0), (0, -1, 1.0), (0,  1, 1.0),
    (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)
]

def heuristic(a: tuple[int, int], b: tuple[int, int], mode: str) -> float:
    """Admissible heuristic for A* search."""
    dist_m = np.hypot(a[0] - b[0], a[1] - b[1]) * RESOLUTION_M
    if mode == "baseline":
        return dist_m
    return FRICTION_COEFF * ROVER_MASS_KG * GRAVITY * dist_m

def smooth_waypoints(waypoints: list[tuple[float, float]], window: int = 3) -> list[tuple[float, float]]:
    """Applies a moving average filter so the rover drives smooth curves instead of zig-zags."""
    if len(waypoints) <= window:
        return waypoints
    pts = np.array(waypoints)
    smoothed = uniform_filter1d(pts, size=window, axis=0, mode='nearest')
    smoothed[0] = pts[0]
    smoothed[-1] = pts[-1]
    return [(float(x), float(y)) for x, y in smoothed]

def plan_path(
    start_world: tuple[float, float],
    goal_world: tuple[float, float],
    elevation_map: np.ndarray,
    mode: str = "topocost"
) -> tuple[list[tuple[float, float]], float]:
    """
    Main interface called by main_loop.py.
    Returns:
      - waypoints: List of (x, y) tuples in world meters
      - total_cost: Total path distance (m) or energy (Joules)
    """
    start = world_to_grid(*start_world)
    goal = world_to_grid(*goal_world)
    slope_grid = compute_slope_grid(elevation_map)

    open_set = []
    heapq.heappush(open_set, (0.0, start))
    came_from = {}
    g_score = {start: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            grid_path = []
            while current in came_from:
                grid_path.append(current)
                current = came_from[current]
            grid_path.append(start)
            grid_path.reverse()

            world_waypoints = [grid_to_world(r, c) for r, c in grid_path]
            return smooth_waypoints(world_waypoints), g_score[goal]

        r, c = current
        z_curr = elevation_map[r, c]

        for dr, dc, dist_mult in NEIGHBORS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                z_next = elevation_map[nr, nc]
                slope_next = slope_grid[nr, nc]
                step_dist = dist_mult * RESOLUTION_M

                step_cost = calculate_step_energy(z_curr, z_next, slope_next, step_dist, mode=mode)
                if np.isinf(step_cost):
                    continue

                tentative_g = g_score[current] + step_cost
                neighbor = (nr, nc)

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal, mode)
                    heapq.heappush(open_set, (f_score, neighbor))

    return [], float('inf')

def generate_test_terrains() -> list[tuple[str, np.ndarray]]:
    """Generates three 100x100 elevation terrains for planner benchmarking."""
    half = ARENA_SIZE_M / 2.0
    coords = np.linspace(-half, half, GRID_SIZE)
    X, Y = np.meshgrid(coords, coords)

    # Terrain 1: Single Central Hill (tests gentle contour detour)
    terrain_1 = 1.6 * np.exp(-((X - 0.2)**2 + (Y - 0.0)**2) / 2.5)

    # Terrain 2: Twin Peaks & Winding Valley Corridor (tests S-curve valley routing)
    peak_a = 1.8 * np.exp(-((X + 1.4)**2 + (Y + 1.2)**2) / 1.7)
    peak_b = 1.8 * np.exp(-((X - 1.4)**2 + (Y - 1.2)**2) / 1.7)
    terrain_2 = peak_a + peak_b

    # Terrain 3: Diagonal Ridge with Low Saddle Pass (tests rollover cliff avoidance + saddle selection)
    ridge = 2.4 * np.exp(-((X + Y)**2) / 1.2)
    saddle_cut = 1.7 * np.exp(-((X - 1.8)**2 + (Y + 1.8)**2) / 1.6)
    terrain_3 = np.maximum(0.0, ridge - saddle_cut)

    return [
        ("Terrain 1: Single Central Hill", terrain_1),
        ("Terrain 2: Twin Peaks & Valley", terrain_2),
        ("Terrain 3: Steep Ridge & Saddle Pass", terrain_3),
    ]

if __name__ == "__main__":
    half = ARENA_SIZE_M / 2.0
    coords = np.linspace(-half, half, GRID_SIZE)
    X, Y = np.meshgrid(coords, coords)

    terrains = generate_test_terrains()
    start_pos = (-4.0, -4.0)
    goal_pos = (4.0, 4.0)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.8))

    for ax, (title, elev_map) in zip(axes, terrains):
        baseline_path, base_dist = plan_path(start_pos, goal_pos, elev_map, mode="baseline")
        topocost_path, topo_energy = plan_path(start_pos, goal_pos, elev_map, mode="topocost")

        print(f"\n--- {title} ---")
        print(f"Baseline A* -> Waypoints: {len(baseline_path)} | Distance: {base_dist:.2f} m")
        print(f"TopoCost A* -> Waypoints: {len(topocost_path)} | Energy Cost: {topo_energy:.2f} J")

        contour = ax.contourf(X, Y, elev_map, levels=25, cmap="terrain")
        fig.colorbar(contour, ax=ax, label="Elevation Z (m)")

        if baseline_path:
            bx, by = zip(*baseline_path)
            ax.plot(bx, by, "r--", linewidth=2.2, label="Baseline A* (Shortest Dist)")
        if topocost_path:
            tx, ty = zip(*topocost_path)
            ax.plot(tx, ty, "g-", linewidth=2.8, label="TopoCost A* (Energy Optimal)")

        ax.scatter(*start_pos, color="blue", s=80, zorder=5, label="Start (-4, -4)")
        ax.scatter(*goal_pos, color="gold", edgecolor="black", s=100, zorder=5, label="Goal (4, 4)")

        ax.set_title(title, fontsize=11)
        ax.set_xlabel("World X (m)")
        ax.set_ylabel("World Y (m)")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.suptitle("Member 2 Multi-Terrain Benchmark: Baseline vs. TopoCost A*", fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.show()