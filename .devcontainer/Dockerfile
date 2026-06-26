# Context: ROS 2 + MoveIt 2 + Omniverse Proxy
# Reason: Provides the C++ path-planning algorithms safely insulated from the host.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Layer 1: Core Robotics matrices including the massive MoveIt 2 stack
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    ros-humble-moveit \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Layer 2: Perception Libraries
RUN pip3 install --no-cache-dir \
    numpy==1.24.3 opencv-python==4.8.0

# Layer 3: Bind the global ROS environment 
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Layer 4: Bind the local physical workspace (if built)
RUN echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /root/.bashrc
