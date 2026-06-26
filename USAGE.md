# Articulated Robot Manipulation: Usage & Control Protocols

## 1. Controlling the Robot via CLI
To send a specific joint target:
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_1', 'joint_2'], points: [{positions: [1.0, 1.0]}]}" -1
```

## 2. Using the MoveIt 2 Python API
Example scripts for automated manipulation can be found in `ros2_ws/src/manipulator_examples/`.
```bash
python3 src/manipulator_examples/pick_and_place.py
```

## 3. Calibration
Run the hand-eye calibration procedure:
```bash
ros2 launch manipulator_calibration calibrate.launch.py
```

## 4. Troubleshooting
### Inverse Kinematics Failure
If the solver fails to find a solution, ensure the target pose is within the robot's reachable workspace. Check for self-collisions in the MoveIt RViz plugin.

### Controller Latency
If the joints jitter or lag, verify the PID gains in `config/controllers.yaml` and ensure the ROS 2 executor is running with high priority.
```bash
chrt -f 99 ros2 launch manipulator_description simulation.launch.py
```
