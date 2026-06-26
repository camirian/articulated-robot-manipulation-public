# Articulated Robot Manipulation


[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

> Physics-correct pick-and-place pipeline on a Franka Panda 7-DOF arm using NVIDIA Isaac Sim 5.0, Lula IK, and Python — no ROS 2 bridge required for the core demo.

## 🎬 Live Demo

[![Watch the Demo on YouTube](https://img.shields.io/badge/▶%20Watch%20Demo-YouTube-red?style=for-the-badge&logo=youtube)](https://youtu.be/_889bOzgvUY)

> 40-second live screen-capture showing the Franka arm using **Lula Inverse Kinematics** to pick up a rigid-body cube, transport it across the workspace, and place it — all driven by `PickPlaceController` with real PhysX contact forces (gravity + friction).

For definitions of key terms, please see my central **[AI & Robotics Glossary](https://github.com/camirian/robotics-ontology/blob/main/GLOSSARY.md)**.


---

## ✅ Skills Demonstrated

-   **State Machine Design:** Implementing robust state machines to manage complex robot behaviors (Home -> Grasp -> Lift -> Place).
-   **Trajectory Generation:** Programmatically generating `JointTrajectory` messages to command smooth robot motion.
-   **Sim-to-Real Control:** Designing controllers that act on simulated hardware (Isaac Sim) via standard ROS 2 interfaces, ready for deployment on physical robots.
-   **Python for Robotics:** Utilizing `rclpy` to build modular and reusable ROS 2 nodes.

---

## 🚀 Projects

### Project 3.1: Sim-to-Real Pick and Place Controller

A ROS 2 package (`simple_manipulation`) that implements a state-machine-based controller for a Franka Emika Panda robot.

-   **Node:** `manipulation_controller`
-   **Logic:** Cycles through a predefined sequence of poses to simulate a pick-and-place operation.
-   **Interface:** Publishes to `/franka_joint_trajectory_controller/joint_trajectory`.
-   **[▶ Video Demonstration](https://youtu.be/_889bOzgvUY)**

---

### Project 3.2: MoveIt 2 Integration (Dynamic Planning)

Upgrade of the control system to use **MoveIt 2**, the industry standard for motion planning, integrated seamlessly with Isaac Sim via NVIDIA OmniGraph.

-   **Dynamic Planning:** Instead of hardcoded joint angles, we define target *poses* (e.g., "Move gripper to [x, y, z]"). MoveIt calculates the collision-free path.
-   **OmniGraph Integration (The Bridge):** Utilizes an OmniGraph Action Graph as the critical translation layer. It bridges the ROS 2 software environment with the physics simulation by capturing standard `FollowJointTrajectory` actions and deterministically translating them into low-level Isaac Sim articulation joint commands.
-   **Integration:** Full MoveIt stack (MoveGroup, RViz) integrated with the simulation.
-   **[▶ Video Demonstration](https://youtu.be/_889bOzgvUY)**

---

### 🏗️ Sim-to-Real Architectural Principles
To guarantee deterministic behavior when transitioning from simulation to physical hardware, this project adheres to strict Sim-to-Real design principles:
1.  **Interface Parity:** The ROS 2 APIs exposed by the OmniGraph action graph exactly mirror the hardware drivers of a physical robot.
2.  **Clock Synchronization:** The ROS 2 network is configured to use the `/clock` topic published by Isaac Sim (`use_sim_time = true`), ensuring node execution is deterministically bound to the simulation's physics step, not the host machine's wall time.
3.  **DDS Tuning:** Implementation of explicit FastRTPS profiles to ensure low-latency, reliable message passing between perception, planning, and simulation nodes.

---

### Project 3.3: Perception Pipeline (Visual Servoing)

Implementation of a closed-loop perception system allowing the robot to detect and interact with objects in the environment.

-   **RGB-D Processing:** Uses OpenCV to detect objects (Red Cube) based on color and depth data from a wrist-mounted camera.
-   **Projection:** Converts 2D pixel coordinates + Depth into precise 3D World Coordinates for the robot.
-   **Visual Servoing:** A coordinator node (`perform_pick`) dynamically commands the MoveIt interface to move the arm to the detected object's location.
-   **[▶ Video Demonstration](https://youtu.be/_889bOzgvUY)**

---

## 🛠️ How to Build and Run

### 1. Build the Workspace (Inside DevContainer)
> [!WARNING]
> DO NOT build this workspace on your Host OS. Open the repository in Visual Studio Code and execute **"Dev Containers: Reopen in Container"**.

Once inside the sandbox, compile the MoveIt 2 C++ architecture:
```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### **Run the Standalone Pick-and-Place Demo (Isaac Sim 5.0)**
No ROS 2 required. Uses the Lula IK `PickPlaceController` directly.

```bash
PYTHON_SH_PATH=/path/to/isaac-sim/python.sh
"${PYTHON_SH_PATH}" scripts/sim_demo.py
```
*Wait for Isaac Sim to load. The simulation starts automatically and runs the full pick-and-place sequence.*
*Watch the terminal for `[DONE] Pick-and-place complete!`*

**Open Terminal 3 (Action):**
```bash
source ros2_ws/install/setup.bash
ros2 run simple_manipulation perform_pick
```
*(The robot will detect the cube and verify alignment by hovering above it)*

---

## 📜 License

This project is licensed under the Apache 2.0 License.
