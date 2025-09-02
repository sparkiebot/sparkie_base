FROM arm64v8/ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-robot-state-publisher

# Clean all apt data files
RUN rm -rf /var/lib/apt/lists/*

COPY ./entrypoint.sh /entrypoint.sh
# Sourcing ros workspace.
ENTRYPOINT ["/entrypoint.sh"]
