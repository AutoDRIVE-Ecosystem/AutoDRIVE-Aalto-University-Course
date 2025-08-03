#!/bin/bash
set -e

# Setup Development Environment
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash

# AutoDRIVE Devkit Workspace
cd /home/autodrive_devkit

# Launch AutoDRIVE Devkit with GUI
# ros2 launch autodrive_nigel bringup_graphics.launch.py

# Launch AutoDRIVE Devkit Headless
# ros2 launch autodrive_nigel bringup_headless.launch.py