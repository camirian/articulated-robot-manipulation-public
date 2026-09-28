# Articulated Robot Manipulation

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2 Build](https://github.com/camirian/articulated-robot-manipulation-public/actions/workflows/build.yml/badge.svg)](https://github.com/camirian/articulated-robot-manipulation-public/actions/workflows/build.yml)

A physics-based pick-and-place pipeline for a Franka Emika Panda 7-DOF arm in **NVIDIA Isaac Sim 5.0**, plus a **ROS 2 (Humble)** workspace for MoveIt 2 motion planning and a colour-based perception demo.

The repository contains two complementary tracks:

1. A **standalone Isaac Sim demo** (`scripts/sim_demo.py`) that drives a full pick-and-place cycle using NVIDIA's Lula IK `PickPlaceController` and PhysX rigid-body physics — **no ROS 2 required**.
2. A **ROS 2 workspace** (`ros2_ws/`) with two packages — `simple_manipulation` (Python) and `simple_moveit_interface` (C++) — that integrate MoveIt 2 motion planning and a perception node with Isaac Sim over the ROS 2 bridge.

## Portfolio role and current boundary

**Status: PARKED supporting reference.** This repository demonstrates Isaac Sim, robot manipulation, ROS 2, MoveIt 2, and a bounded colour-based perception example. Work here is limited to correctness, reproducibility, CI, setup, documentation, and genuine external issues.

The measured Physical AI flagship is [`sim-to-real-control-systems-public`](https://github.com/camirian/sim-to-real-control-systems-public), which focuses on closed-loop control, seeded disturbances, deterministic metrics, and evidence packets.

This repository does not claim:

- sim-to-real transfer to physical hardware;
- robot safety, certification, or production readiness;
- learned manipulation or reinforcement learning;
- robust perception outside the bounded colour-based example;
- or customer or commercial deployment.

Future work here is limited to correctness, reproducibility, CI, setup, documentation, and genuine external issues. New scenarios and research directions belong in the flagship only when they answer a measured question.

## Architecture

The repository has two separate execution paths. The standalone demo does not use ROS. The ROS workspace contains a direct joint-command demo and a separate perception-to-MoveIt path. See [the architecture and topic map](docs/ARCHITECTURE.md).

## Demo video

[![Watch the demo on YouTube](https://img.shields.io/badge/▶%20Watch%20Demo-YouTube-red?style=for-the-badge&logo=youtube)](https://youtu.be/tvgWZHi6GRg)

The repository links a video for the standalone example. The local script uses NVIDIA's `PickPlaceController` with a dynamic cube in Isaac Sim. The video is illustrative; it is not a test report or physical-robot evidence.

For public definitions of key terms, see the central **[AI & Robotics Glossary](https://github.com/camirian/robotics-ontology-public/blob/main/GLOSSARY.md)**.

---

## Prerequisites

> [!IMPORTANT]
> The standalone track requires NVIDIA Isaac Sim. The ROS workspace requires ROS 2 Humble and MoveIt 2; running it against the simulator also requires the Isaac Sim ROS 2 bridge. The two tracks have different requirements.

**For the standalone Isaac Sim demo (`scripts/sim_demo.py`):**
- NVIDIA Isaac Sim **5.0** (provides its own bundled Python via `python.sh`).
- An NVIDIA GPU with a driver that meets the [Isaac Sim system requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
- Network access on first run (the Franka USD asset is loaded from NVIDIA's asset server).

**For the ROS 2 workspace (`ros2_ws/`):**
- Ubuntu 22.04.
- ROS 2 **Humble**.
- MoveIt 2 (`ros-humble-moveit`) and the Panda MoveIt resources (`ros-humble-moveit-resources-panda-moveit-config`).
- `colcon` build tools, `python3-opencv`, and `cv_bridge`.
- To drive the simulation from ROS 2, the Isaac Sim ROS 2 bridge and `scripts/sim_setup.py`.

**Version boundary:** `scripts/sim_demo.py` identifies its standalone demo as Isaac Sim 5.0. `scripts/sim_setup.py` still describes its ROS bridge graph as Isaac Sim 4.5.0-compatible, and `scripts/verify_project.sh` defaults to `~/isaac-sim-4.5.0/python.sh`. The repository has not established that the ROS bridge path works on Isaac Sim 5.0. Select and validate the matching Isaac installation before running that path; CI does not launch Isaac Sim.

`.devcontainer/` defines a local ROS 2 Humble + MoveIt 2 environment. GitHub CI uses the separate `osrf/ros:humble-desktop` image declared in `.github/workflows/build.yml`; it does not build or run the Dev Container.

The Dev Container is a development convenience, not a security boundary: its configuration requests GPU access, host networking, privileged mode, and a read-only bind mount of the host Isaac Sim directory.

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
ISAAC_SIM_5_PYTHON=/path/to/isaac-sim-5.0.0/python.sh
"${ISAAC_SIM_5_PYTHON}" scripts/sim_demo.py
```

Isaac Sim opens, loads the Franka and a cube, and runs the full pick-and-place sequence automatically. Watch the terminal for `[DONE] Pick-and-place complete!`.

### Track 2 — ROS 2 workspace (MoveIt 2 + perception)

**1. Build the workspace.** The `.devcontainer/` provides a ready ROS 2 Humble + MoveIt 2 environment; open the repo in VS Code and run **"Dev Containers: Reopen in Container"**, or build on a host that already has ROS 2 Humble and MoveIt 2.

```bash
cd ros2_ws
rosdep update
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
# sim_setup.py is marked 4.5.0-compatible; this path is not runtime-validated by CI.
ISAAC_SIM_PYTHON=/path/to/isaac-sim-4.5.0/python.sh
"${ISAAC_SIM_PYTHON}" scripts/sim_setup.py
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

**Current limitation:** the perception node also requires `/camera/camera_info` for camera intrinsics. The current `scripts/sim_setup.py` graph publishes RGB and depth but does not create a CameraInfo publisher. This path therefore needs an external CameraInfo source or a bridge-graph correction before it works end to end from this repository alone.

### End-to-end smoke test

`scripts/verify_project.sh` is a host-side integration smoke script, not a CI test. It launches Isaac Sim headless, waits for the ROS 2 bridge, checks for `/joint_states`, and starts the recording launch file. It expects an Isaac Sim Python at `$ISAAC_SIM_PYTHON`; if unset, it falls back to `~/isaac-sim-4.5.0/python.sh`. The integration path has not been validated by the current CI workflow.

## Verification and evidence

GitHub CI builds both ROS 2 packages in the ROS 2 Humble container and runs the deterministic perception helper tests. CI does not start Isaac Sim, MoveIt, RViz, or the ROS graph. The `test_detection.py` fixtures verify image-to-pose math on synthetic images; they do not establish runtime camera calibration, grasp success, robustness, or physical transfer.

The linked video is a visual demonstration. It is not a test report or a record of current runtime compatibility. No physical-robot, safety, reliability, or sim-to-real result is claimed.

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
