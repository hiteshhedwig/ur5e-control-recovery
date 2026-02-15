#!/usr/bin/env bash
# Source this file in each project terminal.
CAPSTONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
CAPSTONE_PREFIX="$CAPSTONE_ROOT/.deps/root/opt/ros/noetic"
export ROS_PACKAGE_PATH="$CAPSTONE_PREFIX/share:${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$CAPSTONE_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$CAPSTONE_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$CAPSTONE_ROOT/src:${PYTHONPATH:-}"
export ROS_HOME="$CAPSTONE_ROOT/.runtime/ros"
export ROS_LOG_DIR="$CAPSTONE_ROOT/.runtime/log"
export GAZEBO_LOG_PATH="$CAPSTONE_ROOT/.runtime/gazebo"
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
export GAZEBO_MASTER_URI=http://127.0.0.1:11355
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH"
