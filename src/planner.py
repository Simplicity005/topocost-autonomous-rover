import heapq
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d
from src.config import (
    GRID_SIZE, RESOLUTION_M, ROVER_MASS_KG, GRAVITY, FRICTION_COEFF,
    world_to_grid, grid_to_world
)
from src.costmap import compute_slope_grid, calculate_step_energy

NEIGHBORS = [
    (-1,  0, 1.0), (1,  0, 1.0), (0, -1, 1.0), (0,  1, 1.0),
    (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)
]

def heuristic(a: tuple[int, int], b: tuple[int, int], mode: str) -> float:
    dist_m = np.hypot(a[0] - b[0], a[1] - b[1]) * RESOLUTION_M
    if mode == "baseline":
        return dist_m
    return FRICTION_COEFF * ROVER_MASS_KG * GRAVITY * dist_m

def smooth_waypoints(waypoints: list[tuple[float, float]], window: int = 3) -> list[tuple[float, float]]:
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

if __name__ == "__main__":
    x = np.linspace(-5, 5, GRID_SIZE)
    y = np.linspace(-5, 5, GRID_SIZE)
    X, Y = np.meshgrid(x, y)
    mock_elevation_map = 1.6 * np.exp(-((X - 0.2)**2 + (Y - 0.0)**2) / 2.5)

    start_pos = (-4.0, -4.0)
    goal_pos = (4.0, 4.0)

    baseline_path, base_dist = plan_path(start_pos, goal_pos, mock_elevation_map, mode="baseline")
    topocost_path, topo_energy = plan_path(start_pos, goal_pos, mock_elevation_map, mode="topocost")

    print(f"Baseline Path Waypoints: {len(baseline_path)} | Distance: {base_dist:.2f} m")
    print(f"TopoCost Path Waypoints: {len(topocost_path)} | Energy Cost: {topo_energy:.2f} J")

    plt.figure(figsize=(8, 7))
    contour = plt.contourf(X, Y, mock_elevation_map, levels=25, cmap="terrain")
    plt.colorbar(contour, label="Terrain Elevation Z (meters)")

    bx, by = zip(*baseline_path)
    tx, ty = zip(*topocost_path)

    plt.plot(bx, by, "r--", linewidth=2.5, label="Baseline A* (Shortest Distance)")
    plt.plot(tx, ty, "g-", linewidth=3.0, label="TopoCost A* (Energy Optimal)")
    plt.scatter(*start_pos, color="blue", s=100, zorder=5, label="Start (-4, -4)")
    plt.scatter(*goal_pos, color="gold", edgecolor="black", s=120, zorder=5, label="Goal (4, 4)")

    plt.title("Member 2 Test: Baseline vs. TopoCost Elevation-Aware A*")
    plt.xlabel("World X (meters)")
    plt.ylabel("World Y (meters)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()