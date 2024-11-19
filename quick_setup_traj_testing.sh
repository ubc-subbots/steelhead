#!/bin/bash

# Define directories
LOG_DIR="$HOME/Documents/code/steel_head/logs"
BUILD_DIR="$HOME/Documents/code/steel_head"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Setup environment variables
echo "Setting up environment variables..."
source /opt/ros/foxy/setup.bash
source "$BUILD_DIR/install/setup.bash"
source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH="$BUILD_DIR/src/triton_gazebo/models"
export GAZEBO_RESOURCE_PATH="$BUILD_DIR/src/triton_gazebo/worlds"

# Set ROS2 logging directory
export RCUTILS_LOGGING_DIRECTORY="$LOG_DIR"
echo "Logs will be saved to $LOG_DIR"

# Clean and build the package
# echo "Cleaning and building the triton_controls package..."
# colcon build --packages-select triton_controls --cmake-clean-cache

# if [ $? -ne 0 ]; then
#     echo "Build failed. Exiting..."
#     exit 1
# fi

# Run the node
# echo "Running the trajectory_generator node with logs..."
# ros2 launch triton_gazebo gate_navigation_test.py


# End of script
