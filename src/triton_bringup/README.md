# triton_bringup
## Description

This package is for creating launch files and utilities needed for launching the entire AUV system in different environments and under different configurations. Nothing new should be defined here, and it should just be launching preexisting nodes in other packages.

## Launch Files

- `barebones_teleop_launch.py` : All-in-one launch file which launches everything necessary for keyboard operation WITHIN GAZEBO.

- `cameras_launch.py`: Starts the nodes neccessary to utilize the usb cameras onboard Steelhead.

    ### Notes
    - This node is configured with the config files `cameraX.yaml`.

- `imu_launch.py`: Starts the nodes that publish the IMU data to ROS as well as the robot_localization package which publishes the Odometry of the robot.

- `teensy_test_launch.py`: Test package to test the connection to the Teensy, mainly to isolate any issues.

- `pool_test_x.py` : Series of launch files for launching during pool time.

    ### x
    - `base`: A whole bunch of nodes that are meant for showcasing the various things that can be launched.
    - `gate`: Launches nodes necessary for a gate navigation test, which is the preliminary task for Robosub.
    - `hardcode`: Launches nodes that make the robot spin certain thrusters for a certain amount of time.

        ### Notes
        - The hardcoded instructions are defined in triton_controls/triton_controls/predetermined_thrust.py.

    - `route`: Launches nodes that make the robot follow a predetermined route.

        ### Notes
        - The hardcoded instructions are defined in triton_controls/triton_controls/predetermined_route.py.

    - `ssh_teleop`: Launches the nodes that allow for teleoperation (keyboard control) of Steelhead. 

        ### Usage
        - Because of certain limitations with SSH, the teleop can't be run in a launch file, so do `ros2 launch triton_bringup pool_test_ssh_teleop.py` alongside `ros2 run triton_teleop ssh_keyboard_teleop` in another terminal session.

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
- Dorson Tang (dorsontang123@gmail.com)