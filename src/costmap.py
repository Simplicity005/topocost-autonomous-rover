import numpy as np
from scipy.ndimage import gaussian_filter
from src.config import RESOLUTION_M, ROVER_MASS_KG, GRAVITY, FRICTION_COEFF, MAX_SLOPE_RAD

def compute_slope_grid(elevation_map: np.ndarray, smooth_sigma: float = 0.8) -> np.ndarray:
    """
    Computes the local terrain slope angle (in radians) for every cell
    in the 100x100 elevation map. Applies light Gaussian smoothing by default
    to suppress depth-sensor quantization noise.
    """
    if smooth_sigma > 0.0:
        filtered_map = gaussian_filter(elevation_map, sigma=smooth_sigma, mode="nearest")
    else:
        filtered_map = elevation_map

    dz_drow, dz_dcol = np.gradient(filtered_map, RESOLUTION_M)
    gradient_magnitude = np.sqrt(dz_drow**2 + dz_dcol**2)
    return np.arctan(gradient_magnitude)

def calculate_step_energy(
    z_current: float,
    z_next: float,
    slope_next_rad: float,
    horizontal_dist: float,
    mode: str = "topocost"
) -> float:
    """
    Calculates traversal cost from current cell to an adjacent cell.
    - 'baseline': Pure Euclidean distance (ignores hills unless lethal).
    - 'topocost': Mechanical energy in Joules (gravitational climb + friction).
    """
    if slope_next_rad > MAX_SLOPE_RAD:
        return float('inf')

    if mode == "baseline":
        return horizontal_dist

    delta_h = z_next - z_current
    uphill_climb = max(0.0, delta_h)

    potential_energy = ROVER_MASS_KG * GRAVITY * uphill_climb
    friction_energy = FRICTION_COEFF * ROVER_MASS_KG * GRAVITY * np.cos(slope_next_rad) * horizontal_dist

    return potential_energy + friction_energy