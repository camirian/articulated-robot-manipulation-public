# Articulated Robot Manipulation: Quickstart Guide

## 1. Prerequisites
- ROS 2 (Humble or Iron).
- MoveIt 2.
- Gazebo or Isaac Sim.

## 2. Installation
```bash
git clone https://github.com/camirian/articulated-robot-manipulation-public.git
cd articulated-robot-manipulation
# Initialize the ROS 2 workspace
cd ros2_ws
vcs import src < manipulator.repos
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## 3. Launching the Manipulator Simulation
```bash
source install/setup.bash
ros2 launch manipulator_description simulation.launch.py
```

## 4. Planning a Motion
Use MoveIt 2 to plan and execute a simple trajectory:
```bash
ros2 launch manipulator_moveit_config demo.launch.py
```

## 5. Verifying Kinematics
Run the forward kinematics validation script:
```bash
python3 scripts/verify_kinematics.py --joint_angles "[0.0, 1.57, -1.57, 0.0, 0.0, 0.0]"
```
