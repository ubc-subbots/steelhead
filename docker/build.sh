#!/bin/bash

# prepare sources
rosdep update
sudo apt update

# Manual installation (debugging)
sudo apt install pip -y
sudo apt update

# Installing OpenCV
sudo apt install -y libopencv-dev python3-opencv

# fetch repo
git clone https://github.com/ubc-subbots/steelhead.git
cd steelhead

# install deps and build
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -i --from-path src --rosdistro lyrical -y
source /opt/ros/lyrical/setup.bash
colcon build
