#!/usr/bin/env bash

set -e

echo "[install_deps] Updating apt index..."
sudo apt-get update

echo "[install_deps] Installing ROS Noetic Turtlebot3 + navigation dependencies..."
sudo apt-get install -y \
  ros-noetic-turtlebot3-gazebo \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-turtlebot3-description \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-move-base-msgs \
  ros-noetic-dwa-local-planner \
  ros-noetic-gmapping

echo "[install_deps] Done."


