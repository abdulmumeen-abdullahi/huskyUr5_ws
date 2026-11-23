# Uncertainty-Triggered, Latency-Aware Digital Twin Synchronization for a Husky A200–UR5 Mobile Manipulator

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-F58025?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![MoveIt 2](https://img.shields.io/badge/Motion_Planning-MoveIt_2-009688)](https://moveit.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![External UKF](https://img.shields.io/badge/Estimator-reconfigurable__UKF-6E40C9)](https://github.com/abdulmumeen-abdullahi/reconfigurable_UKF)

A ROS 2 simulation and state-estimation workspace for a **Clearpath Husky A200** mobile base equipped with a **Universal Robots UR5** 6-DOF manipulator operating in an indoor warehouse environment. This repository houses the robot descriptions, Gazebo warehouse worlds, MoveIt 2 motion-planning configurations, external Unscented Kalman Filter (`ukf_dt_sync`) integration, and empirical latency benchmark datasets supporting an uncertainty-triggered, latency-aware digital twin synchronization framework.

---

## Table of Contents

- [Project Scope & Current Status](#project-scope--current-status)
- [Repository Architecture](#repository-architecture)
- [Prerequisites](#prerequisites)
- [Workspace Installation & Build](#workspace-installation--build)
- [Running Simulations & Pipelines](#running-simulations--pipelines)
- [Manipulator Latency Benchmarks](#manipulator-latency-benchmarks)
- [Related Repositories](#related-repositories)

---

## Project Scope & Current Status

The overarching research objective is a two-subsystem state estimation and supervisory control architecture for mobile manipulation under degraded communication links:

1. **Manipulator Subsystem (Completed):** Compensates for delayed joint-state feedback on the 6-DOF UR5 arm using a reconfigurable $N$-DOF Unscented Kalman Filter ([`reconfigurable_UKF`](https://github.com/abdulmumeen-abdullahi/reconfigurable_UKF)), validated across 17 artificial communication delay horizons (`10 ms` to `5000 ms`).
2. **Mobile Manipulator Simulation Environment (Completed):** Full URDF/Xacro integration, `ros2_control` interfaces, MoveIt 2 planning configurations, and a multi-asset warehouse Gazebo simulation for the combined Husky A200 + UR5 platform.
3. **Ground-Vehicle Base Estimator (In Progress):** Model-based state estimator for the Husky A200 mobile base designed to handle delayed or intermittent absolute pose fixes.
4. **Supervisory Synchronization Layer (Planned):** Rule-based decision module combining state covariance (uncertainty) and Normalized Innovation Squared (NIS) consistency metrics to select among three operational modes:
   - **Continue** on local model prediction,
   - **Request** a high-priority digital twin state update, or
   - **Halt** system execution when estimation consistency degrades beyond safe bounds.

---

## Repository Architecture

External dependencies are decoupled via [`vcstool`](https://github.com/dirk-thomas/vcstool) (`deps.repos`) so core estimator updates remain version-pinned and reproducible.

```text
src/
├── deps.repos                     # vcstool manifest pinning external ukf_dt_sync
├── husky_ur5_description/         # Combined Husky A200 + UR5 URDF/Xacro, meshes & RViz configs
│   ├── launch/                    # Robot state publisher & visualization launch files
│   ├── meshes/                    # Visual (.dae) & collision (.stl) meshes for Husky & UR5
│   ├── rviz/                      # Robot model visualization presets
│   └── urdf/                      # Modular Xacro definitions & ros2_control hardware tags
├── husky_ur5_gazebo/              # Gazebo simulation worlds, warehouse models & spawners
│   ├── launch/                    # warehouse.launch.py, ur5_moveit_gazebo.launch.py, ur5_on_riser.launch.py
│   ├── models/                    # Custom SDF/DAE warehouse assets (shelves, pallets, boxes, walls)
│   └── worlds/                    # warehouse.sdf, github_warehouse.world, table_gz.world, table_riser.world
├── husky_ur5_moveit_config/       # MoveIt 2 configuration for the full mobile manipulator
│   ├── config/                    # SRDF, joint limits, Pilz Cartesian limits, kinematics & controllers
│   └── launch/                    # move_group, RViz, demo & controller spawner launch files
├── ur5_moveit_config/             # Standalone MoveIt 2 configuration for isolated UR5 arm testing
│   ├── config/                    # UR5-only SRDF, kinematics, joint limits & ros2_control setup
│   └── launch/                    # Standalone MoveIt 2 launch suite
├── ukf_dt_sync/                   # [Imported via deps.repos] Reconfigurable N-DOF UKF pipeline
│   ├── config/                    # default_config.yaml (UKF process/measurement noise & delay params)
│   ├── launch/                    # ukf_pipeline.launch.py
│   ├── rviz/                      # ukf_ghosts.rviz (true vs. delayed vs. predicted state overlay)
│   └── ukf_dt_sync/               # ukf.py, ukf_predictor.py, injector.py, trajectory_executor.py, data_logger.py
└── experiment_results/            # Empirical UKF delay-sweep logs & analysis notebook
    ├── UKF_nis_results.csv        # Normalized Innovation Squared (NIS) consistency results
    ├── UKF_pred_results.csv       # Joint-state prediction error metrics across delay horizons
    ├── results_analysis.ipynb     # Jupyter notebook for statistical evaluation & plotting
    └── UKF_timeseries_plot/       # Raw time-series CSVs from 10ms to 5000ms
```

### Package Summary

| Package / Directory | Build Type | Description |
| :--- | :--- | :--- |
| **`husky_ur5_description`** | `ament_cmake` | Defines the kinematic chain (`huskyur5.urdf.xacro`) mounting the UR5 manipulator onto the Clearpath Husky A200 chassis, including collision geometries and `ros2_control` transmissions. |
| **`husky_ur5_gazebo`** | `ament_cmake` | Provides the indoor warehouse simulation (`warehouse.sdf`), tabletop testbeds (`table_riser.world`, `table_gz.world`), 11 custom warehouse SDF models, and simulation launch scripts. |
| **`husky_ur5_moveit_config`** | `ament_cmake` | Configures MoveIt 2 motion planning, inverse kinematics, `ros2_controllers.yaml`, and Pilz industrial trajectory limits for the integrated Husky–UR5 system. |
| **`ur5_moveit_config`** | `ament_cmake` | Standalone MoveIt 2 package for the 6-DOF UR5 manipulator used to execute repeatable arm trajectories during isolated joint-delay experiments. |
| **`ukf_dt_sync`** | `ament_python` | Cloned via `deps.repos` from [`reconfigurable_UKF`](https://github.com/abdulmumeen-abdullahi/reconfigurable_UKF). Implements artificial latency injection (`injector.py`), $N$-DOF UKF forward prediction (`ukf_predictor.py`), RViz ghost visualization, and automated CSV telemetry logging (`data_logger.py`). |
| **`experiment_results`** | Data / Analysis | Contains time-series trajectories, NIS consistency logs, prediction error summaries, and `results_analysis.ipynb` across 17 tested feedback delays. |

---

## Prerequisites

- **OS:** Ubuntu 24.04 LTS
- **Middleware:** ROS 2 (Jazzy) with `colcon` and `rosdep`
- **Simulator:** Gazebo (`gz sim`) & `ros_gz` bridge packages
- **Motion Planning:** MoveIt 2 (`moveit`, `pilz_industrial_motion_planner`, `ros2_control`, `ros2_controllers`)
- **Workspace Tooling & Analysis:** `python3-vcstool`, Python 3.12 (`numpy`, `scipy`, `pandas`, `matplotlib`, `jupyter`)

---

## Workspace Installation & Build

1. **Create a ROS 2 workspace and clone this repository as `src`:**

   ```bash
   mkdir -p ~/huskyUr5_ws
   cd ~/huskyUr5_ws
   git clone https://github.com/abdulmumeen-abdullahi/huskyUr5_ws.git
   ```

2. **Import external dependencies (`ukf_dt_sync`) using `vcstool`:**

   ```bash
   sudo apt update && sudo apt install -y python3-vcstool
   cd ~/huskyUr5_ws
   vcs import src < src/deps.repos
   ```

3. **Install ROS package dependencies via `rosdep`:**

   ```bash
   cd ~/huskyUr5_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the workspace and source the overlay:**

   ```bash
   cd ~/huskyUr5_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## Running Simulations & Pipelines

### 1. Spawn Husky A200 + UR5 in the Warehouse Gazebo Environment

Launches the Gazebo warehouse world (`warehouse.sdf`), spawns the combined Husky A200–UR5 mobile manipulator, and initializes `ros2_control` joint broadcasters and controllers:

```bash
ros2 launch husky_ur5_gazebo warehouse.launch.py
```

### 2. Launch UR5 + MoveIt 2 in Gazebo

For manipulator trajectory execution and MoveIt 2 integration in simulation:

```bash
# Full UR5 + MoveIt 2 + Gazebo simulation
ros2 launch husky_ur5_gazebo ur5_moveit_gazebo.launch.py

# Or standalone MoveIt 2 planning demo for the mobile manipulator
ros2 launch husky_ur5_moveit_config demo.launch.py
```

### 3. Run the Delayed Joint-State UKF Pipeline

Starts the latency injector, $N$-DOF UKF state predictor, trajectory executor, telemetry logger, and RViz ghost-state visualization (`ukf_ghosts.rviz`):

```bash
ros2 launch ukf_dt_sync ukf_pipeline.launch.py
```

---

## Manipulator Latency Benchmarks

The `experiment_results/` directory archives the evaluation of the manipulator UKF synchronization pipeline under controlled feedback latency:

- **Tested Delay Horizons (`UKF_timeseries_plot/`):** 17 discrete feedback delay intervals spanning low-latency to severe communication degradation:
  `10 ms`, `50 ms`, `100 ms`, `300 ms`, `500 ms`, `700 ms`, `900 ms`, `1100 ms`, `1200 ms`, `1300 ms`, `1500 ms`, `2000 ms`, `2500 ms`, `3000 ms`, `3500 ms`, `4500 ms`, and `5000 ms`.
- **Filter Consistency (`UKF_nis_results.csv`):** Tracks Normalized Innovation Squared (NIS) statistics to quantify filter credibility and detect model-measurement divergence as latency increases.
- **Prediction Accuracy (`UKF_pred_results.csv`):** Summarizes joint-space state estimation errors comparing delayed raw observations against UKF forward-predicted states.
- **Reproducing Figures & Metrics:**

  ```bash
  cd ~/huskyUr5_ws/src/experiment_results
  jupyter notebook results_analysis.ipynb
  ```

---

## Related Repositories

- **[`abdulmumeen-abdullahi/reconfigurable_UKF`](https://github.com/abdulmumeen-abdullahi/reconfigurable_UKF):** Core ROS 2 package (`ukf_dt_sync`) providing the generalized $N$-DOF Unscented Kalman Filter, delay injection node, and RViz digital twin ghost visualization.