# TopoCost: Elevation-Aware & Energy-Optimized Rover Path Planning

An autonomous off-road rover simulation built in **CoppeliaSim** and **Python** via the ZeroMQ Remote API. Instead of relying on flat 2D shortest-distance routing, **TopoCost** uses an onboard RGB-D depth camera to incrementally construct a 2.5D terrain elevation map in memory, compute local surface slope gradients, and dynamically replan paths that minimize battery expenditure and prevent rollover hazards.

---

## Project Status: Complete & Verified

| Module | Owner | Files | Status |
| :--- | :--- | :--- | :--- |
| **1. 3D Terrain & Onboard Perception** | Member 1 | `scenes/topocost_arena.ttt`, `build_arena.py`, `src/perception.py` | **Complete & Verified** |
| **2. Physics Costmap & Path Planner** | Member 2 | `src/config.py`, `src/costmap.py`, `src/planner.py` | **Complete & Verified** |
| **3. Rover Control, Master Loop & Telemetry** | Member 3 | `src/controller.py`, `main_loop.py`, `results/` | **Complete & Verified** |
| **End-to-End Integration** | Full Team | All Modules (`(-4.0, -4.0)` → `(3.5, 3.5)`) | **Complete (`34.40 s`, `202.9 J`, `91.9%` SoC)** |

### Key Completed Features
- [x] **10m × 10m Procedural Heightfield Arena (`scenes/topocost_arena.ttt`)**: Uneven terrain with a `1.37m` central hill at `(0, 0)` and secondary mound, plus `/PioneerP3DX` equipped with a `64×64` perspective depth camera (`/PioneerP3DX/DepthCamera`).
- [x] **Real-Time 2.5D Elevation Mapping (`src/perception.py`)**: Vectorized pixel-to-world 3D projection (`TerrainMapper`) updating a persistent `100×100` elevation grid (`0.1m` per cell) with exponential moving-average noise filtering.
- [x] **Physics-Informed Energy Costmap & Repeated A* (`src/costmap.py`, `src/planner.py`)**: Evaluates cell-to-cell traversal cost in Joules (`E = m*g*max(0, dh) + mu*m*g*cos(theta)*dd`), enforces a `25°` rollover safety cutoff (`cost = inf`), and supports both `topocost` (energy-optimal) and `baseline` (2D shortest-distance) planning modes.
- [x] **Closed-Loop Pure Pursuit Control & Telemetry (`src/controller.py`, `main_loop.py`)**: Synchronous `50ms` stepping via ZeroMQ, automatic startup stabilization, live 3D trajectory rendering inside CoppeliaSim, and automated CSV + 3-panel Matplotlib benchmark reporting.

---

## How to Run on Your Laptop (Step-by-Step Guide)

Follow these steps to run the simulation on **Windows, Linux, or macOS**:

### Step 1: Install Prerequisites
1. **Download & Install CoppeliaSim Edu (v4.7 or newer):**
   * Download from the official website: [https://www.coppeliarobotics.com/downloads](https://www.coppeliarobotics.com/downloads)
   * Extract/install and launch **CoppeliaSim**.
2. **Install Python (3.10 or newer) & Git:**
   * Verify in your terminal or command prompt:
     ```bash
     python3 --version
     git --version
     ```

---

### Step 2: Clone the Repository & Install Python Dependencies
Open a terminal (Linux/macOS) or Command Prompt / PowerShell (Windows) and run:

```bash
git clone https://github.com/Simplicity005/topocost-autonomous-rover.git
cd topocost-autonomous-rover
```

Create a virtual environment and install all required packages from `requirements.txt`:

**On Linux / macOS:**
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**On Windows (PowerShell / CMD):**
```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

---

### Step 3: Load the Arena Scene in CoppeliaSim
1. Open the **CoppeliaSim** application on your laptop.
2. In the top menu bar, click **File → Open scene...**
3. Browse to the cloned repository folder and select:
   ```text
   scenes/topocost_arena.ttt
   ```
4. **IMPORTANT — Do NOT click the Play button in CoppeliaSim:**
   * Leave the simulation **Stopped** inside CoppeliaSim.
   * When you run `main_loop.py` from Python in Step 4, it connects to CoppeliaSim over ZeroMQ (port `23000`), automatically stabilizes the rover's initial spawn height, disables the built-in demo script, starts the simulation, steps the physics engine synchronously, and stops the simulation when the goal is reached.

---

### Step 4: Run the Simulation from Your Terminal
Keep **CoppeliaSim** open in the background (or side-by-side with your terminal) and run any of the following commands:

#### 1. Run Full **TopoCost** Mode (Energy-Optimal Path Planner)
The rover uses its depth camera to scan the terrain every `0.5s`, draws a **green 3D path** on the terrain, and routes smoothly around the steep center hill to save battery:
```bash
python main_loop.py --mode topocost
```

#### 2. Run **Baseline** Mode (Shortest 2D Distance Planner)
Draws a **red 3D path** and plans based only on 2D geometric distance for comparison:
```bash
python main_loop.py --mode baseline
```

#### 3. Run Standalone Controller Test (Mock S-Curve Waypoints)
Tests only the Pure Pursuit controller and telemetry logger along fixed waypoints without camera replanning:
```bash
python main_loop.py --mock
```

#### 4. Run Standalone Module Tests (Optional)
```bash
# Test Member 1's live 2.5D depth-camera elevation mapping (requires CoppeliaSim open)
python -m src.perception

# Test Member 2's Baseline vs. TopoCost A* planner on a synthetic hill (no CoppeliaSim needed)
python -m src.planner
```

---

### Step 5: View the Results
As soon as the rover reaches the goal at `(3.5, 3.5)` (or if you stop early with `Ctrl+C`):
1. CoppeliaSim automatically stops the simulation.
2. A **3-panel Matplotlib window** pops up showing:
   * **Panel 1:** Rover 2D Trajectory (`X` vs `Y` in meters)
   * **Panel 2:** Virtual Battery State of Charge (`SoC %`) over time
   * **Panel 3:** Rover Pitch Angle (`deg`) over time
3. Both result files are saved automatically to the `results/` folder:
   * `results/telemetry_logs.csv` — Full timestamped log (`time_s`, `x_m`, `y_m`, `z_m`, `pitch_deg`, `roll_deg`, `energy_j`, `soc_pct`)
   * `results/benchmark_comparison.png` — High-resolution dashboard plot

---

## Troubleshooting Common Laptop Issues

* **`ConnectionRefusedError` or Script Hangs at `"Connecting to CoppeliaSim via ZeroMQ..."`:**
  * Make sure **CoppeliaSim** is open and running before executing `python main_loop.py`.
  * Make sure no previous simulation is stuck paused/running inside CoppeliaSim (click the **Stop** square button in CoppeliaSim's top toolbar, then re-run `python main_loop.py`).
* **`ModuleNotFoundError: No module named 'coppeliasim_zmqremoteapi_client'`:**
  * Make sure your virtual environment is activated (`source .venv/bin/activate` on Linux/macOS or `.venv\Scripts\activate` on Windows), or run `.venv/bin/python main_loop.py` directly.
* **Reloading the Scene After Manual Edits:**
  * You do not need to manually edit `/PioneerP3DX` in CoppeliaSim—`RoverController` in `src/controller.py` automatically removes the built-in `/PioneerP3DX/Script` and adjusts the initial spawn Z height (`Z = 0.185m`) at runtime before starting physics.

---

## System Architecture & Physics Model

1. **Onboard 3D Perception (`src/perception.py`)**: Extracts real-time depth buffers from `/PioneerP3DX/DepthCamera` (`64×64`, `60°` FOV, `0.10m–3.50m` range), projects pixels into the CoppeliaSim world frame, and updates a `100×100` elevation grid (`0.1m` resolution over a `10m × 10m` arena).
2. **Physics-Informed Energy Costmap (`src/costmap.py` & `src/planner.py`)**:
   - **Energy Formula**: `E(A -> B) = m * g * max(0, dh) + mu * m * g * cos(theta) * dd`
   - **Constants**: `m = 10.0 kg`, `g = 9.81 m/s²`, `mu = 0.15`, `battery_capacity = 2500.0 J`
   - **Rollover Safety**: Slopes exceeding `theta > 25°` (`0.436 rad`) are assigned infinite cost.
3. **Closed-Loop Control & Telemetry (`src/controller.py` & `main_loop.py`)**: Drives `/PioneerP3DX` along planned waypoints using Pure Pursuit lookahead (`0.35m`) + proportional yaw steering, renders 3D trajectories inside CoppeliaSim, and logs energy and stability metrics.

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
├── results/
│   ├── telemetry_logs.csv          # Saved timestamped telemetry from simulation run
│   └── benchmark_comparison.png    # 3-panel trajectory, battery SoC, and pitch plot
├── build_arena.py                  # Procedural CoppeliaSim heightfield terrain builder
├── main_loop.py                    # Master synchronous ZeroMQ simulation runner
├── requirements.txt                # Python package dependencies
└── README.md
```
