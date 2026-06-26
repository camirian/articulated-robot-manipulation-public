# Contributing to Articulated Robot Manipulation

Thanks for your interest. This repo pairs a standalone NVIDIA Isaac Sim demo with a small ROS 2 (Humble) workspace, so most contributions touch one of two areas: the Isaac Sim scripts in `scripts/`, or the ROS 2 packages in `ros2_ws/src/` (`simple_manipulation`, `simple_moveit_interface`).

## Development workflow

1. Fork the repository and create a feature branch from `main`.
2. Build the ROS 2 packages and run the linters before opening a PR:
   ```bash
   cd ros2_ws
   colcon build --packages-select simple_manipulation simple_moveit_interface
   colcon test --packages-select simple_manipulation
   ```
   The `simple_manipulation` package ships `ament_flake8`, `ament_pep257`, and `ament_copyright` tests.
3. If you change Isaac Sim behaviour, verify it against the Franka demo (`scripts/sim_demo.py`) and, where applicable, the end-to-end check in `scripts/verify_project.sh`.
4. Open a Pull Request. CI (`.github/workflows/build.yml`) runs a `colcon build` of both packages on ROS 2 Humble and must pass.

## Conventions

- Keep ROS 2 nodes that consume the Isaac Sim clock honouring the `use_sim_time` parameter.
- Match existing topic and frame names (`/joint_command`, `/joint_states`, `/object_pose`) unless a change is documented in the PR description.
- Avoid committing generated artifacts (`build/`, `install/`, `log/`) — these are already in `.gitignore`.
