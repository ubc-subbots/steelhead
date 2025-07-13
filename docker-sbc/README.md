# Docker container for ROS2 environment for Robot SBC
This container contains everything you need to run ROS2 on an SBC, so that a brand new in box SBC can be up and running in less than an hour. 
This Dockerfile is compatible with any SBC with amd64 or arm64 architecture CPU running Linux with Docker installed. 
However, it is only tested on the NVIDIA Jetson TX2 and Radxa X4, running Ubuntu 18.04 and 24.04 respectively.

## How to use
1. Build the image by copying this Dockerfile to a new folder on the target SBC, then run `docker build -t "steelhead-foxy:<version>" . 2>&1 | tee build.log`. `<version>` convention is MAJOR.MINOR.BUGFIX-ARCH, e.g. `1.0.0-amd64`
2. Run the container with `sudo docker run -it --net=host -v /dev:/dev --privileged steelhead-foxy:<version> bash`.
3. To open a new terminal session on an existing container, run `docker exec -it <container_id> bash`. `byobu` is supported within the container to allow multiple sessions in the same terminal window
4. Update and `colcon build` the steelhead repo if necessary.
5. Profit!
