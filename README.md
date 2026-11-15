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
  - Differential-drive Pure Pursuit + proportional yaw controller for `/PioneerP3DX` in synchronous stepping mode (`50ms` physics steps) using 3D rotation-matrix pose extraction.
  - Automatic pre-start scene stabilization (removes `/PioneerP3DX/Script` if present, lifts `/PioneerP3DX` above the `(-4, -4)` terrain surface so wheels never interpenetrate on frame 0, and applies Gaussian frustum-boundary smoothing before gradient computation).
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

## Getting Started & Step-by-Step Testing Guide

### 1. Prerequisites
* **CoppeliaSim Edu** (v4.7 or newer)
* **Python** 3.10+

### 2. Installation
Clone the repository, create a virtual environment, and install the required Python packages from `requirements.txt`:

```bash
git clone https://github.com/Simplicity005/topocost-autonomous-rover.git
cd topocost-autonomous-rover
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

*(Dependencies: `coppeliasim-zmqremoteapi-client`, `numpy`, `scipy`, `matplotlib`, `opencv-python`)*

### 3. Opening the Scene in CoppeliaSim
1. Launch **CoppeliaSim**.
2. Click **File → Open scene...** and load `scenes/topocost_arena.ttt`.
3. **Important:** Leave the simulation **Stopped** inside CoppeliaSim (do **not** press Play manually). `main_loop.py` automatically stabilizes the rover's initial spawn height, disables the built-in demo script, and starts/steps/stops CoppeliaSim via ZeroMQ.

### 4. Running the Simulation Modes
With CoppeliaSim open and `scenes/topocost_arena.ttt` loaded, run any of the following in your terminal:

#### Option A: Full Integrated **TopoCost** (Energy-Optimal) Mode
Runs all 3 modules together—the onboard depth camera updates the 2.5D elevation map every `0.5s` (10 steps), A* replans the lowest-battery path around steep slopes, and the controller drives `/PioneerP3DX` along the **green** 3D line:
```bash
.venv/bin/python main_loop.py --mode topocost
```

#### Option B: **Baseline** (Shortest 2D Distance) Mode for Comparison
Draws a **red** 3D line and plans using Euclidean 2D distance (ignoring uphill energy costs unless slope exceeds the 25° rollover cutoff):
```bash
.venv/bin/python main_loop.py --mode baseline
```

#### Option C: Standalone Member 3 Controller Test (Mock Waypoints)
Tests purely the Pure Pursuit controller, 3D path rendering, and telemetry logger along a fixed S-curve waypoint list (without requiring `DepthCamera` or replanning):
```bash
.venv/bin/python main_loop.py --mock
```

#### Option D: Standalone Module Tests (Member 1 & Member 2)
```bash
# Test Member 1 live 2.5D depth-camera elevation mapping (requires CoppeliaSim open)
.venv/bin/python -m src.perception

# Test Member 2 Baseline vs. TopoCost A* on a synthetic hill (no simulator needed)
.venv/bin/python -m src.planner
```

### 5. Viewing Telemetry Outputs & Verified Benchmark Results
When the rover reaches the goal `((3.5, 3.5))`:
* **Verified TopoCost Run (`scenes/topocost_arena.ttt`):**
  * **Travel Time:** `34.40 s` (`688` synchronous steps)
  * **Total Energy Expended:** `202.9 J` (`91.88%` Battery SoC remaining)
  * **Max Pitch Angle:** `6.3°` (smoothly skirts around the `1.37m` central hill)
* Outputs are saved automatically to:
  * `results/telemetry_logs.csv`
  * `results/benchmark_comparison.png`

---

## Automatic Scene Stabilization Notes (`src/controller.py` & `main_loop.py`)

When running `main_loop.py` on `scenes/topocost_arena.ttt`, three scene quirks are automatically handled at startup in Python:
1. **Wheel Spawn Clearance at `(-4, -4)`:** At `(-4.0, -4.0)`, the terrain elevation is `Z ≈ 0.038m`. `RoverController.__init__` automatically lifts `/PioneerP3DX` from `Z = 0.150m` to `Z = 0.185m` prior to starting physics so the wheels do not spawn buried inside the heightfield mesh.
2. **Built-in Demo Script Removal:** `RoverController.__init__` automatically removes `/PioneerP3DX/Script` (the default ultrasonic Braitenberg demo script) so only Python controls `leftMotor` and `rightMotor`.
3. **Camera Frustum Boundary Smoothing:** `main_loop.py` applies a spatial Gaussian filter (`sigma = 2.0`) to `TerrainMapper.update_map()` prior to calling `plan_path()` so the transition between observed terrain inside the camera's 60° cone and unvisited `0.0m` cells outside the cone does not create artificial `>25°` slope walls.

---

## Team & Roles
* **Member 1 — Perception & Mapping**: CoppeliaSim heightfield environment (`scenes/topocost_arena.ttt`, `build_arena.py`), RGB-D vision sensor calibration, and 3D-to-2.5D elevation grid mapping (`src/perception.py`).
* **Member 2 — Motion Planning & Cost Modeling**: Spatial gradient estimation, gravitational/friction energy modeling (`src/costmap.py`), and dynamic Repeated A* search (`src/planner.py`).
* **Member 3 — Control & Benchmarking**: Synchronous ZeroMQ execution loop (`main_loop.py`), Pure Pursuit waypoint tracking controller, 3D CoppeliaSim trajectory visualization, and telemetry plotting (`src/controller.py`).
