ARG ROS_DISTRO=jazzy
ARG ROS_VARIANT=full
FROM althack/ros2:${ROS_DISTRO}-${ROS_VARIANT}

# Cyclone DDS RMW — matches the robot container (deploy/Dockerfile) so
# discovery works across hosts on multi-NIC / WiFi setups. The althack
# base image does not ship rmw_cyclonedds_cpp.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
