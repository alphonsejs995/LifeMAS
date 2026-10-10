# LifeMAS: Life-Inspired Multi-Agent System

> **A simulation-based multi-agent drone system for autonomous flight, disaster-response mapping, and cooperative decision-making using reinforcement learning.**

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![PySide6](https://img.shields.io/badge/GUI-PySide6-green.svg)](https://pypi.org/project/PySide6/)
[![PyQtGraph](https://img.shields.io/badge/3D%20Vision-PyQtGraph%20%2B%20PyOpenGL-purple.svg)](https://www.pyqtgraph.org/)
[![AirSim](https://img.shields.io/badge/Simulator-AirSim%201.8.1-orange.svg)](https://github.com/Microsoft/AirSim)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## 📋 Table of Contents
- [Project Overview](#-project-overview)
- [Key Objectives](#-key-objectives)
- [Current Implementation Status](#-current-implementation-status)
- [System Architecture](#-system-architecture)
- [Technology Stack](#-technology-stack)
- [Installation & Setup](#-installation--setup)
- [AirSim Configuration](#-airsim-configuration)
- [Usage & Demo](#-usage--demo)
- [Project Structure](#-project-structure)
- [Development Roadmap](#-development-roadmap)
- [Known Limitations](#-known-limitations)
- [Team & Acknowledgements](#-team--acknowledgements)

---

## 🚁 Project Overview

**LifeMAS** (Life-Inspired Multi-Agent System) is an engineering research framework designed for simulating autonomous multi-drone missions in disaster-response scenarios (e.g., post-earthquake search and rescue, wildfire monitoring, structural damage inspection). 

In GPS-denied or degraded disaster zones, single-agent platforms face payload, battery, and spatial coverage limits. LifeMAS leverages bio-inspired multi-agent principles and reinforcement learning to coordinate heterogeneous drone swarms for fast, distributed spatial coverage, target tracking, and emergency response.

---

## 🎯 Key Objectives

1. **Autonomous Flight Control**: Provide robust Python-driven API control over UAV kinematics, takeoff/landing sequences, and multi-axis velocity vectors.
2. **Integrated Control Dashboard**: Offer a modern operator dashboard built with PySide6 that embeds live Unreal Engine simulator viewports, displays real-time telemetry, and exposes manual override tools.
3. **Aerial Survey & Environment Mapping**: Reconstruct persistent 2D occupancy grids and 3D voxel point clouds in real time using 3D LiDAR sensors, boustrophedon lawnmower survey patterns, and PyOpenGL rendering.
4. **Cooperative Multi-Agent Coordination**: Enable swarms of autonomous drones to execute distributed search missions without central bottlenecks. *(Planned)*
5. **Multi-Agent Reinforcement Learning (MARL)**: Implement algorithms such as **MAPPO** (Multi-Agent Proximal Policy Optimization) for dynamic obstacle avoidance and cooperative task allocation. *(Planned)*

---

## ✅ Current Implementation Status

The project features an **autonomous aerial survey flight engine, multi-threaded dashboard, and persistent 3D spatial mapping system**:

- **Autonomous Aerial Survey Engine (`mapping/survey_engine.py`)**:
  - **Boustrophedon Coverage Planning**: Automated lawnmower survey flight plans bounded within configurable scan dimensions, altitudes, lane spacing, and turn rates.
  - **Sensor Frame Transformation**: Exact quaternion-based rotation and translation transforms aligning sensor-relative LiDAR returns with AirSim visual world coordinates.
  - **3D Elevation Gradient Rendering**: Dynamic multi-stage elevation colormap (Navy/Teal ground -> Emerald mid -> Gold/White elevations).
  - **Point Cloud Exporters**: High-resolution export of reconstructed survey point clouds to XYZ, PLY, and compressed NPZ formats.
- **High-Performance Persistent World-Frame Mapping Engine (`mapping/persistent_map.py`)**:
  - **Thread-Safe Architecture**: Reentrant locking (`RLock`) ensuring concurrent read/write access during heavy flight streaming.
  - **Ground vs. Obstacle Discrimination**: Height-filtered ray-tracing distinguishing navigable free-space ground planes from obstacles.
  - **Vectorized Rasterization**: Sub-millisecond hardware-accelerated QImage generation for 2D occupancy grid visualization.
  - **3D Voxel Deduplication**: World-coordinate voxel grid preventing duplicate point buildup.
  - **Disk Persistence**: Save and load compressed maps (`.npz`) preserving boundaries, voxel grids, and metadata.
- **AirSimNH Simulator Integration**: Automated launching and Win32 window embedding of the packaged `AirSimNH` Unreal Engine environment directly into a PySide6 GUI container.
- **Asynchronous Worker Architecture**: Non-blocking `QThread` pipeline (`AirSimWorker`, `SurveyFlightWorker`, `AutonomousMappingWorker`, `LandingWorker`) ensuring zero GUI stuttering.
- **Manual WASD Control UI**: Dedicated interactive GUI window enabling smooth 6-DOF manual keyboard flight testing ($W/S$ forward/backward, $A/D$ left/right, $R/F$ ascend/descend, $Q/E$ yaw turn, `Space` emergency hover).
- **Comprehensive Testing Suite**: Diagnostic scripts in `control/` for LiDAR geometry diagnosis, sensor frame verification, survey flight verification, persistent mapping stability, and UI testing.

> [!NOTE]
> *Multi-agent swarm coordination and MARL policy training (MAPPO) are active roadmap goals under ongoing development.*

---

## 🏗️ System Architecture

```
                                +-----------------------------------+
                                |     LifeMAS PySide6 Dashboard     |
                                |  (Main UI, Telemetry & Controls)  |
                                +-----+---------------+-------+-----+
                                      |               |       |
                 +--------------------+               |       +--------------------+
                 |                                    |                            |
                 v                                    v                            v
+---------------------------------+  +---------------------------------+  +---------------------------------+
|   Embedded AirSimNH Viewport    |  |     2D Occupancy Grid Map       |  |  3D LiDAR Point Cloud Viewport  |
| (Win32 HWND Host Workspace)     |  | (Vectorized QImage + Raytrace)  |  |  (Elevation Colormap + OpenGL)  |
+---------------------------------+  +----------------+----------------+  +---------------------------------+
                 |                                    |                            |
                 +--------------------+               |       +--------------------+
                                      |               |       |
                                      v               v       v
                                +-----------------------------------+
                                |    SurveyEngine & PersistentMap   |
                                |  (Boustrophedon Plan + 3D Voxels) |
                                +-----------------+-----------------+
                                                  |
                                                  v
                                +-----------------------------------+
                                |  QThread Asynchronous Worker Pool |
                                |  (SurveyWorker, Mapping, Landing) |
                                +-----------------+-----------------+
                                                  |
                                                  | Python AirSim RPC API (Port 41451)
                                                  v
                                +-----------------+-----------------+
                                |   AirSimNH Unreal Engine 4 Exec   |
                                |   (Physics Engine & Environment)  |
                                +-----------------------------------+
```

---

## 🛠️ Technology Stack

- **Primary Language**: Python 3.10+
- **Simulation Environment**: Microsoft AirSim 1.8.1 (AirSimNH Unreal Engine 4 Environment)
- **GUI Framework**: PySide6 (Qt 6 for Python)
- **3D Visualization & Graphics**: PyQtGraph (`pyqtgraph.opengl`), PyOpenGL
- **Computer Vision & Math**: OpenCV (`opencv-contrib-python`), NumPy
- **Communication Protocol**: Msgpack RPC over TCP (`msgpack-rpc-python`)
- **OS Platform**: Windows 10/11 (Win32 API embedding support)

---

## 🚀 Installation & Setup

### Prerequisites
- Operating System: Windows 10/11 (64-bit)
- Python: Version 3.10 or higher installed
- Graphics Hardware: NVIDIA GPU with DirectX 11/12 support (recommended for smooth Unreal Engine simulation)

### 1. Clone the Repository
```bash
git clone https://github.com/alphonsejs995/LifeMAS.git
cd LifeMAS
```

### 2. Set Up a Virtual Environment
```bash
# Create virtual environment
python -m venv .venv

# Activate virtual environment (PowerShell)
.\.venv\Scripts\Activate.ps1

# Activate virtual environment (CMD)
.\.venv\Scripts\activate.bat
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

---

## ⚙️ AirSim Configuration

AirSim requires a `settings.json` file in your system user documents directory: `C:\Users\<YourUsername>\Documents\AirSim\settings.json`.

1. Copy the example configuration provided in the repository:
   ```powershell
   New-Item -ItemType Directory -Path "$HOME\Documents\AirSim" -Force
   Copy-Item settings.example.json "$HOME\Documents\AirSim\settings.json"
   ```
2. Verify `settings.json` content:
   ```json
   {
     "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
     "SettingsVersion": 1.2,
     "SimMode": "Multirotor",
     "ClockSpeed": 1.0,
     "Vehicles": {
       "Drone1": {
         "VehicleType": "SimpleFlight",
         "X": 0, "Y": 0, "Z": 0,
         "Sensors": {
           "Lidar1": {
             "SensorType": 6,
             "Enabled": true,
             "NumberOfChannels": 16,
             "PointsPerSecond": 10000
           }
         }
       }
     }
   }
   ```

### 4. Download AirSimNH Simulator Executable
Due to repository size constraints, the pre-built `AirSimNH` Unreal Engine distribution (~1.5 GB) is hosted separately:
1. Download `AirSimNH.zip` from official release sources or Microsoft AirSim releases.
2. Extract the contents into the project directory at `./AirSimNH/AirSimNH/WindowsNoEditor/AirSimNH.exe`.

---

## 🎮 Usage & Demo

### Option A: Launching the Full PySide6 Dashboard
```bash
python ui/main.py
```
1. Click **1 Launch Simulator** to start the `AirSimNH` process.
2. Click **2 Embed Viewport** to dock the simulator window directly into the primary dashboard workspace.
3. Click **3 Connect API** to initiate asynchronous telemetry, mapping, and 3D LiDAR streaming.
4. Open the **Survey Mission Planner** to configure coverage dimensions and launch an automated boustrophedon aerial survey.
5. Inspect real-time 2D occupancy grids and 3D elevation-gradient point clouds, and export results in XYZ/PLY/NPZ formats.

### Option B: WASD Flight Controller
```bash
python wasd_control.py
```
- **W / S**: Move Forward / Backward
- **A / D**: Move Left / Right
- **R / F**: Ascend / Descend
- **Q / E**: Yaw Left / Yaw Right
- **Space**: Stop & Hover in place
- **Esc**: Disarm & Exit

### Option C: Running the Verification Suite
```bash
# Verify autonomous survey flight engine logic
python control/test_survey_engine.py

# Verify survey UI workflow components
python control/test_survey_workflow_ui.py

# Verify LiDAR geometry and sensor transformations
python control/diagnose_lidar_geometry.py
python control/verify_sensor_frame.py

# Run persistent mapping requirement tests
python control/test_persistent_mapping.py
```

---

## 📂 Project Structure

```
LifeMAS/
├── .gitignore                          # Git ignore rules for binaries, venv, datasets, and exports
├── LICENSE                             # MIT License
├── README.md                           # Comprehensive project documentation
├── requirements.txt                    # Python dependency manifest
├── settings.example.json               # AirSim configuration template with 3D LiDAR enabled
├── airsim_test.py                      # API connection diagnostic script
├── flight_test.py                      # Automated flight test script
├── wasd_control.py                     # Interactive keyboard flight controller UI
├── control/                            # Flight control logic & verification test suite
│   ├── check_api_control.py            # API ownership verification
│   ├── check_state.py                  # Multirotor state inspector
│   ├── diagnose_lidar_geometry.py      # Sensor frame geometry diagnostic
│   ├── diagnose_readiness.py           # System readiness diagnostic tool
│   ├── run_diagnostic.py               # Test suite runner
│   ├── run_survey_flight_verification.py # Full survey mission flight validator
│   ├── test_autonomous_mapping.py      # Mapping flight mission verification
│   ├── test_lidar.py                   # 3D LiDAR sensor streaming test
│   ├── test_map_manager.py             # PersistentMapManager unit tests
│   ├── test_persistent_mapping.py      # Multi-scan stability & persistence tests
│   ├── test_survey_engine.py           # Survey planner & reconstruction unit tests
│   ├── test_survey_workflow_ui.py      # Survey workflow UI component tests
│   ├── test_ui_comprehensive.py        # UI component integration test
│   ├── test_ui_launch.py               # Headless UI launch test
│   └── verify_sensor_frame.py          # Sensor coordinate frame validator
├── data/
│   └── maps/                           # Directory for persistent map storage
│       └── .gitkeep
├── mapping/                            # Spatial survey & mapping engine
│   ├── persistent_map.py               # PersistentMapManager (2D Occupancy Grid + 3D Voxels)
│   └── survey_engine.py                # SurveyEngine (Boustrophedon planning, 3D Elevation Color, Export)
├── ui/                                 # PySide6 graphical user interface
│   └── main.py                         # Main dashboard, Survey Planner & Dual 2D/3D visualizer
└── simulator/                          # [Planned] Multi-drone spawning & environment management
```

---

## 🗺️ Development Roadmap

- [x] **Phase 1: Dashboard & Process Foundation**
  - [x] AirSimNH process integration and Win32 viewport embedding.
  - [x] PySide6 live telemetry dashboard.
  - [x] Python API motion control and manual WASD controller.
- [x] **Phase 2: 3D LiDAR Perception & Non-Blocking Worker Architecture**
  - [x] Real-time 3D LiDAR point cloud rendering (`pyqtgraph.opengl`).
  - [x] Asynchronous `QThread` worker architecture (`AirSimWorker`, `AutonomousMappingWorker`, `LandingWorker`).
  - [x] Automated mapping mission flight routines.
- [x] **Phase 3: Spatial Occupancy Grid & Persistent Mapping**
  - [x] Persistent world-frame 2D occupancy grid with Bresenham ray-tracing.
  - [x] 3D voxel grid point deduplication.
  - [x] Map file persistence (compressed `.npz` save/load).
- [x] **Phase 4: Autonomous Aerial Survey & High-Fidelity 3D Reconstruction**
  - [x] Boustrophedon (lawnmower) survey coverage planning.
  - [x] Quaternion sensor-to-world coordinate transformations.
  - [x] Dynamic 3D elevation gradient colormap.
  - [x] Multi-format point cloud export (XYZ, PLY, NPZ).
- [ ] **Phase 5: Multi-Agent Swarm Integration**
  - [ ] Multi-vehicle configuration in AirSim (`Drone1`, `Drone2`, `Drone3`).
  - [ ] Distributed swarm communication protocol.
  - [ ] Decentralized collision avoidance algorithms.
- [ ] **Phase 6: Multi-Agent Reinforcement Learning (MARL)**
  - [ ] Gym/PettingZoo environment wrapper for AirSim swarms.
  - [ ] MAPPO (Multi-Agent PPO) policy training for search-and-rescue coverage.
  - [ ] Benchmark comparison against classical path planning (A*, RRT*).

---

## ⚠️ Known Limitations

- **Platform Specificity**: Window embedding inside the PySide6 UI relies on Win32 user32 API calls (`SetParent`, `SetWindowPos`), requiring Windows 10/11.
- **OpenGL Graphics Acceleration**: 3D LiDAR point cloud visualization requires OpenGL support on the host system.

---

## 👥 Team & Acknowledgements

- **Development Team**: B.Tech AI & Robotics Engineering Team
- **Simulator Framework**: [Microsoft AirSim](https://github.com/Microsoft/AirSim) project.
- **GUI & 3D Engine**: [Qt for Python (PySide6)](https://wiki.qt.io/Qt_for_Python), [PyQtGraph](https://www.pyqtgraph.org/).

---

*For inquiries or collaboration, please open an issue or pull request in this repository.*
