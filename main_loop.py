import os
import argparse
from scipy.ndimage import gaussian_filter
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.controller import RoverController
from src.perception import TerrainMapper
from src.planner import plan_path


def main():
    parser = argparse.ArgumentParser(
        description="TopoCost Master Synchronous Simulation Runner"
    )
    parser.add_argument(
        "--mode",
        choices=["topocost", "baseline"],
        default="topocost",
        help="Planning mode: 'topocost' (energy-optimal) or 'baseline' (shortest 2D distance)",
    )
    parser.add_argument(
        "--goal",
        type=float,
        nargs=2,
        default=[3.5, 3.5],
        metavar=("GOAL_X", "GOAL_Y"),
        help="Goal coordinates in world meters (default: 3.5 3.5)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Force standalone Member 3 test mode using mock S-curve waypoints",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save telemetry CSV and plot without opening an interactive Matplotlib window",
    )
    args = parser.parse_args()

    os.makedirs("results", exist_ok=True)

    # 1. Connect to CoppeliaSim ZeroMQ Remote API in Synchronous Stepping Mode
    print("[Main] Connecting to CoppeliaSim via ZeroMQ...")
    client = RemoteAPIClient()
    sim = client.require("sim")
    sim.stopSimulation()
    try:
        sim.setStepping(True)
    except Exception:
        client.setStepping(True)

    # 2. Initialize Member 3's Rover Controller (auto-disables built-in demo script & fixes spawn Z)
    rover = RoverController(sim)

    # Try connecting Member 1's TerrainMapper unless --mock is requested
    mapper = None
    if not args.mock:
        try:
            mapper = TerrainMapper(sim)
            print(f"[Main] Connected DepthCamera. Running full pipeline in '{args.mode}' mode.")
        except Exception:
            print(
                "[Main] DepthCamera not found in scene — falling back to Member 3 standalone mock waypoints."
            )

    # 3. Start Simulation
    sim.startSimulation()
    print("[Main] Simulation started in synchronous mode.")

    # Default Mock Waypoints (S-curve around an imaginary center hill at (0,0))
    active_waypoints = [
        (-3.0, -3.0),
        (-2.2, -2.5),
        (-1.5, -1.0),
        (-0.8,  1.2),
        ( 0.5,  2.2),
        ( 2.0,  2.8),
        ( 3.2,  3.2),
    ]
    goal_world = (float(args.goal[0]), float(args.goal[1]))
    path_color = (0.0, 1.0, 0.2) if args.mode == "topocost" else (1.0, 0.2, 0.2)

    if mapper is None:
        rover.draw_path_3d(active_waypoints, color=path_color)

    step_count = 0
    max_steps = 1500  # 75 seconds at 50ms timestep

    try:
        while step_count < max_steps:
            sim_time = sim.getSimulationTime()

            # --- DYNAMIC REPLANNING HOOK (Every 10 steps / 0.5s) ---
            if mapper is not None and step_count % 10 == 0:
                raw_elevation_map = mapper.update_map()
                # Smooth camera-frustum boundary transition before gradient computation
                elevation_map = gaussian_filter(raw_elevation_map, sigma=2.0)
                rx, ry, _, _, _, _ = rover.get_pose()
                planned_wps, _ = plan_path(
                    (rx, ry), goal_world, elevation_map, mode=args.mode
                )
                if planned_wps:
                    active_waypoints = planned_wps
                    rover.draw_path_3d(
                        active_waypoints,
                        color=path_color,
                        elevation_map=elevation_map,
                    )

            # Drive rover toward next lookahead waypoint
            goal_reached = rover.compute_wheel_velocities(active_waypoints)
            rover.log_telemetry(sim_time)

            if goal_reached:
                print(
                    f"[Main] Goal reached in {sim_time:.2f}s! "
                    f"Total Energy: {rover.energy_used_j:.1f} J"
                )
                break

            # Advance CoppeliaSim physics by exactly 1 timestep (50ms)
            sim.step()
            step_count += 1

    finally:
        rover.stop()
        sim.stopSimulation()
        print("[Main] Simulation stopped.")
        rover.save_and_plot_results(show_plot=not args.no_show)


if __name__ == "__main__":
    main()
