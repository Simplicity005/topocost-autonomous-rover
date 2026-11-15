# TopoCost: Elevation-Aware & Energy-Optimized Rover Path Planning

An autonomous off-road rover simulation built in **CoppeliaSim** and **Python** via the ZeroMQ Remote API. Instead of relying on flat 2D shortest-distance routing, **TopoCost** uses an onboard RGB-D depth camera to incrementally construct a 2.5D terrain elevation map in memory, compute local surface slope gradients, and dynamically replan paths that minimize battery expenditure and prevent rollover hazards.

---

## Project Completion Status

- [x] **Module 1 — 3D Terrain & Onboard Perception (`build_arena.py`, `scenes/topocost_arena.ttt`, `src/perception.py`)**: Complete
  - Procedural `10m × 10m` uneven heightfield terrain generator and CoppeliaSim arena scene.
  - Real-time `64×64` perspective depth buffer extraction (`TerrainMapper`), 3D world-frame camera projection, and exponential moving-average updates to the persistent `100×100` 2.5D elevation grid (`0.1m` resolution).
- [x] **Module 2 — Physics-Informed Costmap & Path Planner (`src/config.py`, `src/costmap.py`, `src/planner.py`)**: Complete
  - Spatial gradient and slope angle calculation (`np.gradient`) with rollover safety cutoff (`theta > 25°` assigned `inf` cost).
  - Gravitational + rolling-resistance battery expenditure model in Joules (`E(A -> B) = m * g * max(0, dh) + mu * m * g * cos(theta) * dd`).
  - Dynamic 8-connected Repeated A* search supporting both `topocost` (energy-optimal) and `baseline` (2D shortest-distance) modes with moving-average trajectory smoothing.
- [x] **Module 3 — Closed-Loop Rover Control, Telemetry & Master Loop (`src/controller.py`, `main_loop.py`)**: Complete
  - Differential-drive Pure Pursuit + proportional yaw controller for `/PioneerP3DX` in synchronous stepping mode (`50ms` physics steps).
  - Real-time 3D trajectory rendering inside CoppeliaSim (`sim.addDrawingObject`) lifted onto the 2.5D terrain surface.
  - Closed-loop dynamic replanning every 10 simulation steps (`0.5s`) and automated telemetry export (`results/telemetry_logs.csv` and 3-panel Matplotlib dashboard `results/benchmark_comparison.png`).

---

## System Architecture

1. **Onboard 3D Perception (`src/perception.py`)**: Extracts real-time depth buffers from a rover-mounted perspective vision sensor (`64x64`), projects pixels into the global 3D world frame, and updates a persistent `100x100` elevation grid (`0.1m` resolution over a `10m x 10m` arena).
2. **Physics-Informed Energy Costmap (`src/costmap.py` & `src/planner.py`)**: Computes spatial terrain gradients and evaluates transition costs in Joules (J) between grid cells:
   - **Energy Formula**: `E(A -> B) = m * g * max(0, dh) + mu * m * g * cos(theta) * dd`
   - **Parameters**: `m = 10.0 kg`, `g = 9.81 m/s²`, `mu = 0.15`, `battery_capacity = 2500.0 J`
   - **Rollover Safety**: Slopes exceeding the critical stability threshold (`theta > 25 deg`) are assigned infinite cost.
3. **Closed-Loop Control & Telemetry (`src/controller.py` & `main_loop.py`)**: Drives the rover along planned waypoints in synchronous simulation mode, renders live 3D trajectories inside CoppeliaSim, and logs virtual battery State-of-Charge (`SoC %`) and chassis pitch/roll angles.

---

## Repository Structure

```text
topocost-autonomous-rover/
├── scenes/
│   └── topocost_arena.ttt          # CoppeliaSim 10x10m uneven terrain & Pioneer P3-DX scene
├── src/
│   ├── __init__.py
│   ├── config.py                   # Shared arena, grid, and physics parameters
│   ├── perception.py               # Member 1: Depth camera projection & 2.5D map memory
│   ├── costmap.py                  # Member 2: Slope gradient & battery energy model
│   ├── planner.py                  # Member 2: Baseline A* vs. TopoCost Energy A*
│   └── controller.py               # Member 3: Pure Pursuit controller, 3D path drawing & telemetry
├── results/                        # Auto-generated CSV telemetry and 3-panel Matplotlib plots
├── build_arena.py                  # Procedural CoppeliaSim heightfield terrain builder
├── main_loop.py                    # Master synchronous ZeroMQ simulation runner
├── requirements.txt                # Python package dependencies
└── README.md
```

---

## Getting Started

### 1. Prerequisites
* **CoppeliaSim Edu** (v4.7 or newer)
* **Python** 3.10+

### 2. Installation
Create a virtual environment and install the required Python packages from `requirements.txt`:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

*(Dependencies: `coppeliasim-zmqremoteapi-client`, `numpy`, `scipy`, `matplotlib`, `opencv-python`)*

### 3. Running the Simulation
1. Open **CoppeliaSim** and load `scenes/topocost_arena.ttt`.
2. Run the master simulation loop in either **TopoCost (Energy-Optimal)** or **Baseline (Shortest-Distance)** mode:

```bash
# Run elevation-aware energy-optimized planner (default)
.venv/bin/python main_loop.py --mode topocost

# Run standard 2D shortest-path planner for comparison
.venv/bin/python main_loop.py --mode baseline

# Run standalone controller test using mock S-curve waypoints
.venv/bin/python main_loop.py --mock
```

3. View the generated telemetry log in `results/telemetry_logs.csv` and the 3-panel comparison plot in `results/benchmark_comparison.png`.

### 4. Standalone Module Tests
Each module can also be tested independently:

```bash
# Test Member 1 live 2.5D depth-camera elevation mapping (requires CoppeliaSim open)
.venv/bin/python -m src.perception

# Test Member 2 Baseline vs. TopoCost A* on a synthetic hill (no simulator needed)
.venv/bin/python -m src.planner
```

---

## Evaluation Metrics
* **Cumulative Energy Expended (Joules & Battery SoC %)**
* **Maximum Chassis Pitch / Roll Angle (deg)**
* **Total Trajectory Length (m) & Travel Time (s)**

---

## Team & Roles
* **Member 1 — Perception & Mapping**: CoppeliaSim heightfield environment (`scenes/topocost_arena.ttt`, `build_arena.py`), RGB-D vision sensor calibration, and 3D-to-2.5D elevation grid mapping (`src/perception.py`).
* **Member 2 — Motion Planning & Cost Modeling**: Spatial gradient estimation, gravitational/friction energy modeling (`src/costmap.py`), and dynamic Repeated A* search (`src/planner.py`).
* **Member 3 — Control & Benchmarking**: Synchronous ZeroMQ execution loop (`main_loop.py`), Pure Pursuit waypoint tracking controller, 3D CoppeliaSim trajectory visualization, and telemetry plotting (`src/controller.py`).
