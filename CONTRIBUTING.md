# Contributing to Articulated Robot Manipulation

This repository implements rigorous Sim-to-Real control pipelines. Contributions must preserve deterministic behavior across both NVIDIA Isaac Sim and the physical hardware.

## 🏗️ Sim-to-Real Code Standards
1.  **Time Sync:** All ROS 2 nodes *must* support the `use_sim_time` parameter. Do not use standard wall-system clocks for any control loops; always use `rclcpp::Clock(RCL_ROS_TIME)`.
2.  **OmniGraph Integrity:** Modifications to the OmniGraph action graphs (`.usda` files) must guarantee 1:1 parity with the ROS 2 physical driver interfaces. 
3.  **DDS Profiles:** Do not modify the default FastRTPS profiles without explicit justification logged in an Architectural Decision Record (ADR).

## 🔄 Development Workflow
1.  Fork and branch from `main`.
2.  Test changes against the reference `ur10e` or `panda` digital twin in Isaac Sim.
3.  Verify the ROS 2 Action server (`FollowJointTrajectory`) successfully completes without latency spikes.
4.  Submit a Pull Request and verify the `ament_lint` CI pipeline passes.
