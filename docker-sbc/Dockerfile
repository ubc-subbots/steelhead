FROM ros:foxy-ros-base-focal
MAINTAINER Fei Kuan for UBC Subbots - kuan@iatfei.com
ARG TARGETARCH

# IMPORTANT!!!! Before running rosdep or colcon commands, you must source the ros2 installation in the same RUN command. 

RUN apt-get -y update && apt-get -y upgrade && apt-get -y autoremove
RUN apt-get install -y apt-transport-https apt-utils bash-completion python3-pip nano htop less byobu ros-perception ros-foxy-robot-localization ros-foxy-angles ros-foxy-imu-tools ros-foxy-usb-cam ros-foxy-rosbridge-suite libboost-filesystem-dev libopencv-dev python3-opencv libceres-dev libeigen3-dev ros-foxy-cv-bridge ros-foxy-vision-opencv ros-foxy-eigen3-cmake-module

# Download and build ros2 packages
WORKDIR /opt/ros/foxy
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; RTI_NC_LICENSE_ACCEPTED=yes rosdep install --from-paths . --ignore-src --rosdistro foxy -yr"

# Clone Steelhead repo
WORKDIR /root/
RUN git clone https://github.com/ubc-subbots/steelhead.git 

# Realsense install. Only works on amd64! To run on Jetson we need to follow their specific guide (pain)
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    mkdir -p /etc/apt/keyrings; \
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null; \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list; \
    apt-get update; \
    apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg; \
    mkdir -p /root/ros2_ws/src/; \
    cd /root/ros2_ws/src/; \
    git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master; \
    cd /root/ros2_ws/; \
    /bin/bash -c "source /opt/ros/foxy/setup.bash; rosdep init --include-eol-distros; rosdep update --include-eol-distros; rosdep install -i --from-path src --rosdistro foxy --skip-keys=librealsense2 -y; colcon build; . install/local_setup.bash"; \
    echo "source /root/ros2_ws/install/local_setup.bash" >> /root/.bashrc; \
    echo 'echo "Sourced RealSense in /root/ros2_ws/"' >> /root/.bashrc; \
    else \
    echo "Non-amd64 architecture. Skipping Realsense Build"; \
    fi
# OpenCV build skipped
#&& \
#                git clone -b 4.5.3 https://github.com/opencv/opencv
#RUN mkdir /root/opencv/build
#WORKDIR /root/opencv/build
#RUN cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
#RUN make -j4 && make install
#RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; rosdep update"

# Setup and build Steelhead
WORKDIR /root/steelhead
RUN /bin/bash -c 'source /opt/ros/foxy/setup.bash; rosdep update --include-eol-distros; rosdep install -i --from-path src --rosdistro foxy --skip-keys "gazebo_ros gazebo_dev gazebo_ros_pkgs phidgets_spatial realsense2_camera camera_model" -y'
RUN python3 -m pip install setuptools==58.2.0 pyserial sshkeyboard
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build --packages-skip steelhead_gazebo"
RUN echo "source /root/steelhead/install/setup.bash" >> /root/.bashrc
RUN echo 'echo "Sourced Steelhead in /root/steelhead/"' >> /root/.bashrc
RUN echo "alias cbs='colcon build --packages-skip steelhead_gazebo && source install/setup.bash'" >> /root/.bashrc
RUN echo "alias clean='rm -r build install log'" >> /root/.bashrc

CMD /bin/bash
