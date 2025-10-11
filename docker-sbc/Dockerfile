FROM ros:foxy-ros-base-focal
MAINTAINER Fei Kuan for UBC Subbots - kuan@iatfei.com

# IMPORTANT!!!! Before running rosdep or colcon commands, you must source the ros2 installation in the same RUN command. 

RUN apt-get -y update && apt-get -y upgrade && apt-get -y autoremove
RUN apt-get install -y apt-utils bash-completion python3-pip nano htop less byobu ros-perception ros-foxy-robot-localization ros-foxy-angles ros-foxy-imu-tools ros-foxy-usb-cam libboost-filesystem-dev

# Download and build ros2 packages
WORKDIR /opt/ros/foxy
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; RTI_NC_LICENSE_ACCEPTED=yes rosdep install --from-paths . --ignore-src --rosdistro foxy -yr"

# Clone Steelhead repo
WORKDIR /root/
RUN git clone https://github.com/ubc-subbots/steelhead.git 

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
RUN /bin/bash -c 'source /opt/ros/foxy/setup.bash; rosdep update; rosdep install -i --from-path src --rosdistro foxy --skip-keys "gazebo_ros gazebo_dev gazebo_ros_pkgs phidgets_spatial realsense2_camera" -y'
RUN python3 -m pip install setuptools==58.2.0 pyserial sshkeyboard
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build --packages-skip steelhead_gazebo"
RUN echo "source /root/steelhead/install/setup.bash" >> /root/.bashrc
RUN echo 'echo "Sourced Steelhead in /root/steelhead/"' >> /root/.bashrc
RUN echo "alias cbs='colcon build --packages-skip steelhead_gazebo && source install/setup.bash'" >> /root/.bashrc
RUN echo "alias clean='rm -r build install log'" >> /root/.bashrc

CMD /bin/bash
