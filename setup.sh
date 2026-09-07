#!/bin/bash
set -e

envsubst < src/ros2.repos | vcs import src
./scripts/rosdep-install.sh
