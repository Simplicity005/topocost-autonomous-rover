import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def create_arena():
    client = RemoteAPIClient()
    sim = client.require('sim')

    # 100x100 grid covering -5.0m to +5.0m (10m x 10m total)
    nx, ny = 100, 100
    x = np.linspace(-5.0, 5.0, nx)
    y = np.linspace(-5.0, 5.0, ny)
    X, Y = np.meshgrid(x, y)

    # Steep hill at the center (0, 0) + smaller side mound
    steep_hill = 1.35 * np.exp(-((X - 0.0)**2 + (Y - 0.0)**2) / 2.2)
    side_mound = 0.70 * np.exp(-((X + 2.0)**2 + (Y - 2.2)**2) / 2.5)
    Z = steep_hill + side_mound

    heights = Z.flatten().tolist()

    # Spawn the 3D heightfield terrain in CoppeliaSim
    terrain_handle = sim.createHeightfieldShape(2, 0.5, nx, ny, 10.0, heights)
    sim.setObjectAlias(terrain_handle, 'Terrain')
    sim.setObjectPosition(terrain_handle, [0.0, 0.0, 0.0])

    np.save('ground_truth_elevation.npy', Z.astype(np.float32))
    print("Success! Terrain generated in CoppeliaSim.")

if __name__ == '__main__':
    create_arena()