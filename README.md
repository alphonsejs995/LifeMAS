# LifeMAS: Life-Inspired Multi-Agent System

> **A simulation-based multi-agent drone system for autonomous flight, disaster-response mapping, and cooperative decision-making using reinforcement learning.**

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![PySide6](https://img.shields.io/badge/GUI-PySide6-green.svg)](https://pypi.org/project/PySide6/)
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
2. **Integrated Control Dashboard**: Offer an operator dashboard built with PySide6 that embeds live Unreal Engine simulator viewports, displays real-time telemetry, and exposes manual override tools.
3. **Disaster Environment Mapping**: Synthesize visual and depth sensor feeds to reconstruct 2D/3D spatial occupancy grids for obstacle avoidance. *(In progress)*
4. **Cooperative Multi-Agent Coordination**: Enable swarms of autonomous drones to execute distributed search missions without central bottlenecks. *(Planned)*
5. **Multi-Agent Reinforcement Learning (MARL)**: Implement algorithms such as **MAPPO** (Multi-Agent Proximal Policy Optimization) for dynamic obstacle avoidance and cooperative task allocation. *(Planned)*

---

## ✅ Current Implementation Status

The project currently features a functional **single-drone simulation control system and operator dashboard**:

- **AirSimNH Simulator Integration**: Automated launching and Win32 window embedding of the packaged `AirSimNH` Unreal Engine environment directly into a PySide6 GUI container.
- **Python API Bridge**: Asynchronous bi-directional communication with AirSim over TCP RPC (Port 41451).
- **Live Telemetry & Diagnostics**: Real-time rendering of UAV position coordinates $(x, y, z)$, linear velocities $(v_x, v_y, v_z)$, orientation angles, and arming states.
- **Manual WASD Control UI**: Dedicated interactive GUI window enabling smooth 6-DOF manual keyboard flight testing ($W/S$ forward/backward, $A/D$ left/right, $R/F$ ascend/descend, $Q/E$ yaw turn, `Space` emergency hover).
- **Flight Testing Utility Scripts**: Scripted preflight checks, API readiness diagnosis, and automated arming/takeoff/movement/landing verification routines.

> [!NOTE]
> *Multi-agent swarm coordination, depth occupancy mapping, and MARL policy training are active roadmap goals under ongoing development.*

---

## 🏗️ System Architecture

```
                                +-----------------------------------+
                                |     LifeMAS PySide6 Dashboard     |
                                |  (Telemetry, Viewport, WASD UI)   |
                                +-----------------+-----------------+
                                                  |
                                                  | Python AirSim API (RPC)
                                                  v
                                +-----------------+-----------------+
                                |      Microsoft AirSim Client      |
                                |     (MultirotorClient API)        |
                                +-----------------+-----------------+
                                                  |
                                                  v
                                +-----------------+-----------------+
                                |   AirSimNH Unreal Engine 4 Exec   |
                                |   (Physics Engine & Environment)  |
                                +-----------------------------------+
                                                  |
                                (Planned Modules Integration)
                                                  |
                 +--------------------------------+--------------------------------+
                 |                                                                 |
                 v                                                                 v
+---------------------------------+                               +---------------------------------+
|   Environment Mapping Module    |                               |    MARL Policy Engine (MAPPO)   |
| (Occupancy Grids & Vision Feed) |                               |  (Multi-Agent Reinforcement RL) |
+---------------------------------+                               +---------------------------------+
```

---

## 🛠️ Technology Stack

- **Primary Language**: Python 3.10+
- **Simulation Environment**: Microsoft AirSim 1.8.1 (AirSimNH Unreal Engine 4 Environment)
- **GUI Framework**: PySide6 (Qt 6 for Python)
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
         "X": 0, "Y": 0, "Z": 0
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
1. Click **1 Launch Simulator** to launch the `AirSimNH` process.
2. Click **2 Embed Viewport** to dock the simulator window directly into the dashboard.
3. Click **3 Connect API** to initiate telemetry polling.

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

### Option C: Diagnostic & Preflight Scripts
```bash
# Verify AirSim API communication
python airsim_test.py

# Perform arming, takeoff, 2s movement, and landing test
python flight_test.py
```

---

## 📂 Project Structure

```
LifeMAS/
├── .gitignore              # Git ignore rules for binaries, venv, and pycache
├── LICENSE                 # MIT License
├── README.md               # Project documentation
├── requirements.txt        # Python dependency manifest
├── settings.example.json   # AirSim configuration template
├── airsim_test.py          # API connection diagnostic test script
├── flight_test.py          # Automated flight test script
├── wasd_control.py         # Standalone WASD interactive keyboard flight controller
├── control/                # Diagnostic routines and flight controller logic
│   ├── check_api_control.py
│   ├── check_state.py
│   ├── diagnose_readiness.py
│   ├── preflight_check.py
│   ├── test_api_motion.py
│   └── test_telemetry.py
├── ui/                     # PySide6 graphical user interface
│   └── main.py             # Main dashboard application & Win32 viewport embedding
├── mapping/                # [In Progress] Spatial mapping & occupancy grid module
└── simulator/              # [Planned] Multi-drone spawning & environment management
```

---

## 🗺️ Development Roadmap

- [x] **Phase 1: Basic Simulation & Dashboard Foundation**
  - [x] AirSimNH process integration and viewport embedding.
  - [x] PySide6 live telemetry dashboard.
  - [x] Python API motion control and manual WASD controller.
- [ ] **Phase 2: Sensor Perception & Mapping**
  - [ ] Depth camera stream processing via OpenCV.
  - [ ] Real-time 2D/3D occupancy grid generation.
  - [ ] Obstacle boundary detection for disaster terrain.
- [ ] **Phase 3: Multi-Agent Swarm Integration**
  - [ ] Multi-vehicle configuration in AirSim (`Drone1`, `Drone2`, `Drone3`).
  - [ ] Distributed swarm communication protocol.
  - [ ] Decentralized collision avoidance algorithms.
- [ ] **Phase 4: Multi-Agent Reinforcement Learning (MARL)**
  - [ ] Gym/PettingZoo environment wrapper for AirSim swarms.
  - [ ] MAPPO (Multi-Agent PPO) policy training for search-and-rescue coverage.
  - [ ] Benchmark comparison against classical path planning (A*, RRT*).

---

## ⚠️ Known Limitations

- **Platform Specificity**: Window embedding inside the PySide6 UI relies on Win32 user32 API calls (`SetParent`, `SetWindowPos`), requiring Windows 10/11.
- **Simulation Only**: Designed and evaluated strictly in AirSim simulated environments; hardware deployment onto physical PX4/ArduPilot flight controllers is planned for future work.

---

## 👥 Team & Acknowledgements

- **Development Team**: B.Tech AI & Robotics Engineering Team
- **Simulator Framework**: [Microsoft AirSim](https://github.com/Microsoft/AirSim) project.
- **GUI Engine**: [Qt for Python (PySide6)](https://wiki.qt.io/Qt_for_Python).

---

*For inquiries or collaboration, please open an issue or pull request in this repository.*
