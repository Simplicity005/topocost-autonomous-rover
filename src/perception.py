import math
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class TerrainMapper:
    def __init__(self, sim, cam_alias='/PioneerP3DX/DepthCamera', grid_size=100, arena_size=10.0):
        self.sim = sim
        try:
            self.cam_handle = sim.getObject(cam_alias)
        except Exception:
            self.cam_handle = sim.getObject('/DepthCamera')

        self.grid_size = grid_size
        self.arena_size = arena_size
        self.resolution = arena_size / grid_size  # 0.1m per cell
        self.half_arena = arena_size / 2.0        # 5.0m

        # Persistent 2.5D elevation memory (initialized to 0.0m flat assumption)
        self.elevation_map = np.zeros((grid_size, grid_size), dtype=np.float32)
        self.visited_mask = np.zeros((grid_size, grid_size), dtype=bool)

        # Camera intrinsics matching CoppeliaSim Vision Sensor settings
        self.res_x = 64
        self.res_y = 64
        self.fov_rad = math.radians(60.0)
        self.near_clip = 0.10
        self.far_clip = 3.50

        # Precompute pixel ray multipliers for fast vectorized projection
        u = np.arange(self.res_x, dtype=np.float32)
        v = np.arange(self.res_y, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        tan_half_fov = math.tan(self.fov_rad / 2.0)
        # CoppeliaSim sensor frame: +Z forward, +Y up, +X left
        self.ray_x = -((uu - (self.res_x - 1) / 2.0) / (self.res_x / 2.0)) * tan_half_fov
        self.ray_y = ((vv - (self.res_y - 1) / 2.0) / (self.res_y / 2.0)) * tan_half_fov

    def update_map(self):
        """Reads depth camera, projects to 3D world coords, and updates 100x100 elevation grid."""
        self.sim.handleVisionSensor(self.cam_handle)
        depth_bytes, res = self.sim.getVisionSensorDepth(self.cam_handle, 1)
        depth = np.array(self.sim.unpackFloatTable(depth_bytes), dtype=np.float32)
        depth = depth.reshape((res[1], res[0]))

        # Filter out invalid readings at clipping boundaries
        valid = (depth > self.near_clip + 0.02) & (depth < self.far_clip - 0.05)
        if not np.any(valid):
            return self.elevation_map

        z_cam = depth[valid]
        x_cam = self.ray_x[valid] * z_cam
        y_cam = self.ray_y[valid] * z_cam

        # Homogeneous coordinates in camera local frame: shape (4, N)
        ones = np.ones_like(z_cam)
        pts_cam = np.vstack((x_cam, y_cam, z_cam, ones))

        # Transform to CoppeliaSim global world frame using 3x4 pose matrix
        mat = np.array(self.sim.getObjectMatrix(self.cam_handle), dtype=np.float32).reshape(3, 4)
        pts_world = mat @ pts_cam  # shape (3, N) -> rows are X_w, Y_w, Z_w

        xw, yw, zw = pts_world[0], pts_world[1], pts_world[2]

        # Convert world meters [-5, +5) to grid indices [0, 99]
        cols = np.floor((xw + self.half_arena) / self.resolution).astype(np.int32)
        rows = np.floor((yw + self.half_arena) / self.resolution).astype(np.int32)

        in_bounds = (rows >= 0) & (rows < self.grid_size) & (cols >= 0) & (cols < self.grid_size)
        rows, cols, zw = rows[in_bounds], cols[in_bounds], zw[in_bounds]

        # Update grid using exponential moving average to suppress sensor noise
        for r, c, z in zip(rows, cols, zw):
            if not self.visited_mask[r, c]:
                self.elevation_map[r, c] = z
                self.visited_mask[r, c] = True
            else:
                self.elevation_map[r, c] = 0.7 * self.elevation_map[r, c] + 0.3 * z

        return self.elevation_map


# Standalone test mode for Member 1
if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.require('sim')
    client.setStepping(True)
    sim.startSimulation()

    mapper = TerrainMapper(sim)

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 5))
    img = ax.imshow(mapper.elevation_map, origin='lower', extent=[-5, 5, -5, 5], vmin=0, vmax=1.5, cmap='terrain')
    plt.colorbar(img, label='Elevation Z (m)')
    ax.set_title('Live 2.5D Elevation Map (Member 1 Test)')
    ax.set_xlabel('World X (m)')
    ax.set_ylabel('World Y (m)')

    try:
        for step in range(300):
            sim.step()
            if step % 5 == 0:
                grid = mapper.update_map()
                img.set_data(grid)
                plt.pause(0.01)
    finally:
        sim.stopSimulation()
        plt.ioff()
        plt.show()