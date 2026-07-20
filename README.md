# Articulated Robot Manipulation

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2 Build](https://github.com/camirian/articulated-robot-manipulation-public/actions/workflows/build.yml/badge.svg)](https://github.com/camirian/articulated-robot-manipulation-public/actions/workflows/build.yml)

A physics-based pick-and-place pipeline for a Franka Emika Panda 7-DOF arm in **NVIDIA Isaac Sim 5.0**, plus a **ROS 2 (Humble)** workspace for MoveIt 2 motion planning and a colour-based perception demo.

The repository contains two complementary tracks:

1. A **standalone Isaac Sim demo** (`scripts/sim_demo.py`) that drives a full pick-and-place cycle using NVIDIA's Lula IK `PickPlaceController` and PhysX rigid-body physics — **no ROS 2 required**.
2. A **ROS 2 workspace** (`ros2_ws/`) with two packages — `simple_manipulation` (Python) and `simple_moveit_interface` (C++) — that integrate MoveIt 2 motion planning and a perception node with Isaac Sim over the ROS 2 bridge.

## Demo video

[![Watch the demo on YouTube](https://img.shields.io/badge/▶%20Watch%20Demo-YouTube-red?style=for-the-badge&logo=youtube)](https://youtu.be/tvgWZHi6GRg)

A short screen capture showing the Franka arm using **Lula Inverse Kinematics** to pick up a rigid-body cube, transport it across the workspace, and place it — driven by `PickPlaceController` with real PhysX contact forces (gravity and friction): **https://youtu.be/tvgWZHi6GRg**

For definitions of key terms, see the central **[AI & Robotics Glossary](https://github.com/camirian/robotics-ontology/blob/main/GLOSSARY.md)**.

---

## Prerequisites

> [!IMPORTANT]
> This project requires NVIDIA Isaac Sim and/or a ROS 2 Humble installation to run. It **cannot** run in a plain environment without these. The two tracks below have different requirements.

**For the standalone Isaac Sim demo (`scripts/sim_demo.py`):**
- NVIDIA Isaac Sim **5.0** (provides its own bundled Python via `python.sh`).
- An NVIDIA GPU with a driver that meets the [Isaac Sim system requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
- Network access on first run (the Franka USD asset is loaded from NVIDIA's asset server).

**For the ROS 2 workspace (`ros2_ws/`):**
- Ubuntu 22.04.
- ROS 2 **Humble**.
- MoveIt 2 (`ros-humble-moveit`) and the Panda MoveIt resources (`ros-humble-moveit-resources-panda-moveit-config`).
- `colcon` build tools, `python3-opencv`, and `cv_bridge`.
- To drive the simulation from ROS 2, the Isaac Sim ROS 2 bridge (the Isaac Sim install above) and the bridge setup in `scripts/sim_setup.py`.

A reproducible ROS 2 build environment is also defined in `.devcontainer/` (VS Code Dev Containers) and exercised in CI by `.github/workflows/build.yml`.

---

## Repository layout

```
scripts/
  sim_demo.py            Standalone Isaac Sim pick-and-place demo (Lula IK, no ROS 2)
  sim_setup.py           Loads Franka in Isaac Sim and enables the ROS 2 bridge
  verify_project.sh      End-to-end smoke test (Isaac Sim + ROS 2 launch)
ros2_ws/src/
  simple_manipulation/       Python package: controller, perception, MoveIt launch files
  simple_moveit_interface/   C++ package: move_to_pose MoveGroup client
.devcontainer/           ROS 2 Humble + MoveIt 2 Dev Container definition
.github/workflows/       colcon build CI for the two ROS 2 packages
```

---

## How to build and run

### Track 1 — Standalone Isaac Sim demo (no ROS 2)

Run the pick-and-place demo directly with the Isaac Sim Python interpreter:

```bash
# Point this at your Isaac Sim 5.0 install
PYTHON_SH_PATH=/path/to/isaac-sim/python.sh
"${PYTHON_SH_PATH}" scripts/sim_demo.py
```

Isaac Sim opens, loads the Franka and a cube, and runs the full pick-and-place sequence automatically. Watch the terminal for `[DONE] Pick-and-place complete!`.

### Track 2 — ROS 2 workspace (MoveIt 2 + perception)

**1. Build the workspace.** The `.devcontainer/` provides a ready ROS 2 Humble + MoveIt 2 environment; open the repo in VS Code and run **"Dev Containers: Reopen in Container"**, or build on a host that already has ROS 2 Humble and MoveIt 2.

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> [!NOTE]
> The MoveIt launch files depend on `moveit_resources_panda_moveit_config`. Install it with
> `sudo apt install ros-humble-moveit-resources-panda-moveit-config` if it is not already present.

**2. Bring up MoveIt 2** (MoveGroup + RViz) for the Panda:

```bash
ros2 launch simple_manipulation bringup_moveit.launch.py
```

**3. Drive Isaac Sim from ROS 2.** In a separate terminal, start Isaac Sim with the ROS 2 bridge so it publishes `/joint_states` and accepts `/joint_command`:

```bash
"${PYTHON_SH_PATH}" scripts/sim_setup.py
```

The `manipulation_controller` node publishes `sensor_msgs/JointState` messages to the `/joint_command` topic that Isaac Sim's bridge consumes:

```bash
ros2 run simple_manipulation manipulation_controller
```

**4. Perception-driven pick (optional).** `visual_pick.launch.py` brings up MoveIt together with the camera transform and perception node. The `perform_pick` coordinator waits for a detected object pose on `/object_pose` and commands the arm to hover above the cube:

```bash
ros2 launch simple_manipulation visual_pick.launch.py
# in another terminal
ros2 run simple_manipulation perform_pick
```

### End-to-end smoke test

`scripts/verify_project.sh` launches Isaac Sim headless, waits for the ROS 2 bridge, checks for `/joint_states`, and runs the recording launch file. It expects an Isaac Sim Python at `$ISAAC_SIM_PYTHON` (or `~/isaac-sim-4.5.0/python.sh`); set the variable for your install before running.

---

## Available ROS 2 entry points

| Package | Executable | Role |
| --- | --- | --- |
| `simple_manipulation` | `manipulation_controller` | Publishes `JointState` commands to `/joint_command` |
| `simple_manipulation` | `simple_trajectory_server` | Trajectory server helper |
| `simple_manipulation` | `perception_node` | OpenCV colour + depth object detection, publishes `/object_pose` |
| `simple_manipulation` | `perform_pick` | Coordinator that reacts to detected object poses |
| `simple_moveit_interface` | `move_to_pose` | C++ MoveGroup client for Cartesian pose goals |

---

## License

This project is licensed under the [Apache 2.0 License](LICENSE).
