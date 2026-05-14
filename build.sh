#!/bin/bash
set -e

# The esp32-s3-uros-controller submodule ships an ESP-IDF firmware
# project whose top-level CMakeLists.txt confuses colcon (it tries to
# build it as a ROS package). Drop a COLCON_IGNORE marker so colcon
# skips that subtree. Safe to re-run.
touch src/esp32-s3-uros-controller/firmware/COLCON_IGNORE 2>/dev/null || true

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
