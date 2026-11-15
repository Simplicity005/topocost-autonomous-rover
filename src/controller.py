import os
import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from src.config import ROVER_MASS_KG, GRAVITY, FRICTION_COEFF, world_to_grid


class RoverController:
    def __init__(
        self,
        sim,
        rover_path="/PioneerP3DX",
        left_motor="/PioneerP3DX/leftMotor",
        right_motor="/PioneerP3DX/rightMotor",
    ):
        self.sim = sim
        self.rover_handle = self.sim.getObject(rover_path)
        self.left_motor = self.sim.getObject(left_motor)
        self.right_motor = self.sim.getObject(right_motor)

        # Rover physical parameters (Pioneer P3-DX)
        self.wheel_radius = 0.0975   # meters
        self.wheel_base = 0.381      # meters between left and right wheels
        self.base_speed = 0.4        # m/s cruise speed
        self.kp_yaw = 1.8            # Proportional steering gain
        self.lookahead_dist = 0.35   # meters

        # Virtual Battery & Physics Telemetry Constants
        self.mass = ROVER_MASS_KG
        self.g = GRAVITY
        self.mu = FRICTION_COEFF
        self.battery_capacity_j = 2500.0  # Total battery in Joules
        self.energy_used_j = 0.0

        # Telemetry history
        self.prev_pos = None
        self.history = {
            "time": [],
            "x": [],
            "y": [],
            "z": [],
            "pitch_deg": [],
            "roll_deg": [],
            "energy_j": [],
            "soc_pct": [],
        }
        self.drawing_handle = None

    def get_pose(self):
        """Returns (x, y, z, roll, pitch, yaw) in world coordinates."""
        pos = self.sim.getObjectPosition(self.rover_handle, self.sim.handle_world)
        orient = self.sim.getObjectOrientation(self.rover_handle, self.sim.handle_world)
        roll, pitch, yaw = orient[0], orient[1], orient[2]
        return pos[0], pos[1], pos[2], roll, pitch, yaw

    def draw_path_3d(self, waypoints, color=(0.0, 1.0, 0.2), z_offset=0.12, elevation_map=None):
        """Draws the planned waypoint path as a 3D line inside CoppeliaSim."""
        if self.drawing_handle is not None:
            try:
                self.sim.removeDrawingObject(self.drawing_handle)
            except Exception:
                pass
            self.drawing_handle = None

        if not waypoints or len(waypoints) < 2:
            return

        # Create a 3D line drawing object (width = 4 pixels)
        self.drawing_handle = self.sim.addDrawingObject(
            self.sim.drawing_lines, 4, 0.0, -1, 1000, list(color)
        )
        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i][0], waypoints[i][1]
            x2, y2 = waypoints[i + 1][0], waypoints[i + 1][1]

            if len(waypoints[i]) > 2:
                z1 = waypoints[i][2]
            elif elevation_map is not None:
                r1, c1 = world_to_grid(x1, y1)
                z1 = float(elevation_map[r1, c1]) + z_offset
            else:
                z1 = z_offset

            if len(waypoints[i + 1]) > 2:
                z2 = waypoints[i + 1][2]
            elif elevation_map is not None:
                r2, c2 = world_to_grid(x2, y2)
                z2 = float(elevation_map[r2, c2]) + z_offset
            else:
                z2 = z_offset

            self.sim.addDrawingObjectItem(self.drawing_handle, [x1, y1, z1, x2, y2, z2])

    def compute_wheel_velocities(self, waypoints, goal_tolerance=0.25):
        """Pure-Pursuit + Proportional steering toward the active lookahead waypoint."""
        if not waypoints:
            self.stop()
            return False

        x, y, z, roll, pitch, yaw = self.get_pose()

        # Check if we reached the final goal
        goal_x, goal_y = waypoints[-1][0], waypoints[-1][1]
        dist_to_goal = math.hypot(goal_x - x, goal_y - y)
        if dist_to_goal < goal_tolerance:
            self.stop()
            return True  # Goal reached

        # Find the closest waypoint to current rover position so we only look forward along the path
        closest_idx = min(
            range(len(waypoints)),
            key=lambda i: math.hypot(waypoints[i][0] - x, waypoints[i][1] - y),
        )

        # Find the first waypoint ahead of closest_idx that is at least lookahead_dist away
        target_wp = waypoints[-1]
        for wp in waypoints[closest_idx:]:
            if math.hypot(wp[0] - x, wp[1] - y) >= self.lookahead_dist:
                target_wp = wp
                break

        # Compute desired heading and wrap angle error to [-pi, pi]
        desired_yaw = math.atan2(target_wp[1] - y, target_wp[0] - x)
        yaw_error = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))

        # Reduce forward linear speed during sharp turns so rover doesn't overshoot
        v = self.base_speed * max(0.25, 1.0 - abs(yaw_error) / math.pi)
        omega = self.kp_yaw * yaw_error

        # Differential drive inverse kinematics
        v_left = v - (omega * self.wheel_base / 2.0)
        v_right = v + (omega * self.wheel_base / 2.0)

        self.sim.setJointTargetVelocity(self.left_motor, v_left / self.wheel_radius)
        self.sim.setJointTargetVelocity(self.right_motor, v_right / self.wheel_radius)
        return False

    def log_telemetry(self, sim_time):
        """Calculates instantaneous physics energy expenditure and logs rover state."""
        x, y, z, roll, pitch, yaw = self.get_pose()

        if self.prev_pos is not None:
            dx = x - self.prev_pos[0]
            dy = y - self.prev_pos[1]
            dz = z - self.prev_pos[2]
            dd = math.hypot(dx, dy)
            slope_angle = math.atan2(abs(dz), dd + 1e-6)

            # Gravitational climb work + Rolling resistance work
            e_gravity = self.mass * self.g * max(0.0, dz)
            e_friction = self.mu * self.mass * self.g * math.cos(slope_angle) * dd
            self.energy_used_j += e_gravity + e_friction

        self.prev_pos = (x, y, z)
        soc_pct = max(0.0, 100.0 * (1.0 - self.energy_used_j / self.battery_capacity_j))

        self.history["time"].append(sim_time)
        self.history["x"].append(x)
        self.history["y"].append(y)
        self.history["z"].append(z)
        self.history["pitch_deg"].append(math.degrees(pitch))
        self.history["roll_deg"].append(math.degrees(roll))
        self.history["energy_j"].append(self.energy_used_j)
        self.history["soc_pct"].append(soc_pct)

    def stop(self):
        self.sim.setJointTargetVelocity(self.left_motor, 0.0)
        self.sim.setJointTargetVelocity(self.right_motor, 0.0)

    def save_and_plot_results(
        self,
        csv_path="results/telemetry_logs.csv",
        plot_path="results/benchmark_comparison.png",
        show_plot=True,
    ):
        """Saves CSV telemetry and generates a 3-panel Matplotlib report."""
        os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
        os.makedirs(os.path.dirname(plot_path) or ".", exist_ok=True)

        with open(csv_path, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["time_s", "x_m", "y_m", "z_m", "pitch_deg", "roll_deg", "energy_j", "soc_pct"]
            )
            for i in range(len(self.history["time"])):
                writer.writerow(
                    [
                        round(self.history["time"][i], 2),
                        round(self.history["x"][i], 3),
                        round(self.history["y"][i], 3),
                        round(self.history["z"][i], 3),
                        round(self.history["pitch_deg"][i], 2),
                        round(self.history["roll_deg"][i], 2),
                        round(self.history["energy_j"][i], 2),
                        round(self.history["soc_pct"][i], 2),
                    ]
                )

        fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))

        # Panel 1: 2D Trajectory
        axes[0].plot(
            self.history["x"],
            self.history["y"],
            color="green",
            linewidth=2,
            label="Driven Path",
        )
        axes[0].set_title("Rover 2D Trajectory")
        axes[0].set_xlabel("X (m)")
        axes[0].set_ylabel("Y (m)")
        axes[0].grid(True)
        axes[0].legend()

        # Panel 2: Battery State-of-Charge (%)
        axes[1].plot(
            self.history["time"],
            self.history["soc_pct"],
            color="blue",
            linewidth=2,
        )
        axes[1].set_title("Battery State of Charge (SoC %)")
        axes[1].set_xlabel("Simulation Time (s)")
        axes[1].set_ylabel("Battery SoC (%)")
        axes[1].grid(True)

        # Panel 3: Chassis Pitch Angle (deg)
        axes[2].plot(
            self.history["time"],
            self.history["pitch_deg"],
            color="crimson",
            linewidth=1.8,
        )
        axes[2].set_title("Rover Pitch Angle (Stability)")
        axes[2].set_xlabel("Simulation Time (s)")
        axes[2].set_ylabel("Pitch (deg)")
        axes[2].grid(True)

        plt.tight_layout()
        plt.savefig(plot_path, dpi=150)
        print(f"[Telemetry] Saved CSV to {csv_path} and plot to {plot_path}")
        if show_plot:
            plt.show()
