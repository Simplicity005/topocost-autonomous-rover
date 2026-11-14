import numpy as np

# Arena & Grid Specifications (Agreed Team Contract)
ARENA_SIZE_M = 10.0          # 10m x 10m (-5.0m to +5.0m)
GRID_SIZE = 100              # 100 x 100 cells
RESOLUTION_M = ARENA_SIZE_M / GRID_SIZE  # 0.1m per cell

# Rover Physics Parameters
ROVER_MASS_KG = 10.0         # kg
GRAVITY = 9.81               # m/s^2
FRICTION_COEFF = 0.15        # Rolling resistance (mu)
MAX_SLOPE_DEG = 25.0         # Rollover limit (degrees)
MAX_SLOPE_RAD = np.deg2rad(MAX_SLOPE_DEG)

def world_to_grid(x_m: float, y_m: float) -> tuple[int, int]:
    """Converts CoppeliaSim world meters (X, Y) to NumPy grid indices (row, col)."""
    col = int(np.clip((x_m + ARENA_SIZE_M / 2.0) / RESOLUTION_M, 0, GRID_SIZE - 1))
    row = int(np.clip((y_m + ARENA_SIZE_M / 2.0) / RESOLUTION_M, 0, GRID_SIZE - 1))
    return row, col

def grid_to_world(row: int, col: int) -> tuple[float, float]:
    """Converts NumPy grid indices (row, col) to CoppeliaSim world meters (X, Y)."""
    x_m = (col * RESOLUTION_M) - (ARENA_SIZE_M / 2.0) + (RESOLUTION_M / 2.0)
    y_m = (row * RESOLUTION_M) - (ARENA_SIZE_M / 2.0) + (RESOLUTION_M / 2.0)
    return float(x_m), float(y_m)