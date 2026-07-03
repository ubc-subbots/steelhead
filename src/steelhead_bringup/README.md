# steelhead_bringup
## Description

This package is for creating launch files and utilities needed for launching the entire AUV system in different environments and under different configurations. Nothing new should be defined here, and it should just be launching preexisting nodes in other packages.

## Launch Files

- `barebones_gazebo_launch.py` : All-in-one launch file which launches everything necessary for keyboard operation WITHIN GAZEBO.

    - If the simulation is running really slow, try following the optimization guide in the steelhead_gazebo README.md.

- `localization_visualization_gazebo_launch.py` : Launches a visualization WITHIN GAZEBO of where the robot "thinks" it is, visualized in RViz.

- `pool_test_x_launch.py` : Series of launch files for launching during pool time.

    - `all`: A whole bunch of nodes that would be the robot utilizuing every sensor it has.
    - `sensors`: Launches all of the nodes that publish information from the onboard sensors (currently the imu, cameras, and depth sensor.)
    - `hardcode`: Launches nodes that make the robot spin certain thrusters for a certain amount of time.

        - The hardcoded instructions are defined in `steelhead_controls/steelhead_controls/predetermined_thrust.py`.
        
    - `hardcode_pipeline`: Launches nodes that are neccessary for the hard coded hover route.

        - The hardcoded instructions are defined in `src/steelhead_pipeline/config/competition_hardcode_sequence.yaml`.

    - `ssh_teleop`: Launches the nodes that allow for teleoperation (keyboard control) of Steelhead. 

        - Because of certain limitations with SSH, the teleop can't be run in a launch file, so do `ros2 launch steelhead_bringup pool_test_ssh_teleop.py` alongside `ros2 run steelhead_teleop ssh_keyboard_teleop` in another terminal session.

    - `hover`: Launches everything necessary for the hover script, which aims to keep Steelhead at a certain depth and orientation.

        - We wait a bit before trying to hover to allow the imu to calibrate itself.

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
- Dorson Tang (dorsontang123@gmail.com)