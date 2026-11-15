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
3. **Important:** Leave the simulation **Stopped** inside CoppeliaSim (do **not** press Play manually). `main_loop.py` automatically starts, steps (`50ms`), and stops CoppeliaSim via ZeroMQ.

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

### 5. Viewing Telemetry Outputs
When the rover reaches the goal (or when you stop the run with `Ctrl+C`):
* CoppeliaSim stops automatically.
* A **3-panel Matplotlib window** pops up displaying:
  1. **Rover 2D Trajectory** (`X` vs `Y` in meters)
  2. **Battery State of Charge (`SoC %`)** vs. simulation time
  3. **Rover Pitch Angle (`deg`)** vs. simulation time
* Outputs are saved automatically to:
  * `results/telemetry_logs.csv`
  * `results/benchmark_comparison.png`

---

## Scene Troubleshooting: Fixing Rover Flipping on Simulation Start (`scenes/topocost_arena.ttt`)

> [!WARNING]
> **Known Scene Issue to Fix in CoppeliaSim:** When starting the simulation in `scenes/topocost_arena.ttt`, `/PioneerP3DX` may immediately launch or flip upside down on frame 1.

### Why This Happens
1. **Rover Spawn Height / Mesh Interpenetration (Primary Cause):**
   In `build_arena.py`, `sim.createHeightfieldShape(...)` centers the heightfield's vertical bounding box around `Z = 0.0` (so the flat base sits at `Z ≈ -0.675m` rather than `Z = 0.0m`, or the terrain surface at `(-4, -4)` overlaps with the rover's wheels/chassis). When `/PioneerP3DX` spawns intersecting the terrain collision mesh (or drops from too high onto an angled triangle), the physics engine applies an extreme depenetration impulse on step 1 that flips the robot upside down.
2. **Built-in `PioneerP3DX` Child Script Enabled:**
   The stock `Pioneer p3dx.ttm` model in CoppeliaSim comes with an attached Lua script that immediately commands high wheel velocities using ultrasonic sensors, fighting `src/controller.py`.
3. **Default Floor Colliding with Heightfield:**
   If the default CoppeliaSim `ResizableFloor_5_25` is still present at `Z = 0.0` while `Terrain` is also around `Z = 0.0`, the rover collides with two overlapping static meshes simultaneously.

### How to Fix `scenes/topocost_arena.ttt` in CoppeliaSim (Checklist for Scene Lead)
1. **Adjust `/Terrain` and `/PioneerP3DX` Z-Position:**
   * Select `Terrain` in the Scene Hierarchy, open **Position** (` Translation`), and set its Z position to `0.675` (half of the max peak height `1.35m`) so the flat ground base sits at `Z = 0.0m`.
   * Select `PioneerP3DX` at `X = -4.0, Y = -4.0` and set its **Z Position** so the wheels sit **2–3 cm above** the terrain surface at `(-4, -4)` (e.g., `Z = 0.16` if the terrain base is at `Z = 0.0`). Make sure the chassis and caster wheel are not buried inside the terrain mesh.
2. **Delete or Hide the Default Floor (`ResizableFloor_5_25`):**
   * If `ResizableFloor_5_25` is in the Scene Hierarchy, delete it (or uncheck **Respondable** in its Shape Dynamics properties) so it does not intersect `Terrain`.
3. **Disable the Built-In Lua Script on `PioneerP3DX`:**
   * In the Scene Hierarchy, click the script icon next to `PioneerP3DX` → **Edit → Disable script** (or delete the child script) so only Python controls `leftMotor` and `rightMotor`.
4. **Verify Physics Engine Settings:**
   * Switch the physics engine dropdown in the top toolbar to **Bullet 2.83** (or **ODE**) with timestep `dt = 50 ms`, and save the scene (**File → Save scene** to `scenes/topocost_arena.ttt`).

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
