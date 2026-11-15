# TopoCost: Elevation-Aware & Energy-Optimized Rover Path Planning

An autonomous off-road rover simulation built in **CoppeliaSim** and **Python** via the ZeroMQ Remote API. Instead of relying on flat 2D shortest-distance routing, **TopoCost** uses an onboard RGB-D depth camera to incrementally construct a 2.5D terrain elevation map in memory, compute local surface slope gradients, and dynamically replan paths that minimize battery expenditure and prevent rollover hazards.

---

## System Architecture

1. **Onboard 3D Perception (`src/perception.py`)**: Extracts real-time depth buffers from a rover-mounted perspective vision sensor (`64x64`), projects pixels into the global 3D world frame, and updates a persistent 100x100 elevation grid (0.1m resolution over a 10m x 10m arena).
2. **Physics-Informed Energy Costmap (`src/costmap.py` & `src/planner.py`)**: Computes spatial terrain gradients and evaluates transition costs in Joules (J) between grid cells:
   - **Energy Formula**: `E(A -> B) = m * g * max(0, dh) + mu * m * g * cos(theta) * dd`
   - **Rollover Safety**: Slopes exceeding the critical stability threshold (`theta > 25 deg`) are assigned infinite cost.
3. **Closed-Loop Control & Telemetry (`src/controller.py` & `main_loop.py`)**: Drives the rover along planned waypoints in synchronous simulation mode, renders live 3D trajectories inside CoppeliaSim, and logs virtual battery State-of-Charge (SoC %) and chassis pitch/roll angles.

---

## Repository Structure

```text
topocost-rover-sim/
├── scenes/
│   └── topocost_arena.ttt          # CoppeliaSim uneven terrain & rover scene
├── src/
│   ├── __init__.py
│   ├── config.py                   # Shared arena, grid, and physics parameters
│   ├── perception.py               # Depth camera projection & 2.5D map memory
│   ├── costmap.py                  # Slope gradient & battery energy model
│   ├── planner.py                  # Baseline A* vs. TopoCost Energy A*
│   └── controller.py               # Waypoint tracking controller & telemetry logger
├── results/                        # Saved CSV telemetry and Matplotlib plots
├── main_loop.py                    # Master synchronous execution script
├── requirements.txt                # Python package dependencies
└── README.md
```

---

## Getting Started

### 1. Prerequisites
* **CoppeliaSim Edu** (v4.7 or newer)
* **Python** 3.10+

### 2. Installation
Clone the repository and install the required Python packages:

```bash
git clone https://github.com/<your-username>/topocost-rover-sim.git
cd topocost-rover-sim
pip install -r requirements.txt
```

*(Dependencies: `coppeliasim-zmqremoteapi-client`, `numpy`, `scipy`, `matplotlib`, `opencv-python`)*

### 3. Running the Simulation
1. Open **CoppeliaSim** and load `scenes/topocost_arena.ttt`.
2. Run the master script in either **TopoCost (Energy-Optimal)** or **Baseline (Shortest-Distance)** mode:

```bash
# Run elevation-aware energy-optimized planner
python main_loop.py --mode topocost

# Run standard 2D shortest-path planner for comparison
python main_loop.py --mode baseline
```

3. View the generated energy and stability comparison plots in `results/benchmark_comparison.png`.

---

## Evaluation Metrics
* **Cumulative Energy Expended (Joules & Battery SoC %)**
* **Maximum Chassis Pitch / Roll Angle (deg)**
* **Total Trajectory Length (m) & Travel Time (s)**

---

## Team & Roles
* **Perception & Mapping**: CoppeliaSim heightfield environment, RGB-D vision sensor calibration, and 3D-to-2.5D elevation grid mapping.
* **Motion Planning & Cost Modeling**: Spatial gradient estimation, gravitational/friction energy modeling, and dynamic Repeated A* search.
* **Control & Benchmarking**: Synchronous ZeroMQ execution loop, waypoint tracking controller, 3D visualization, and telemetry plotting.
