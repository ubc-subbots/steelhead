# steelhead_bringup
## Description

This package is for creating launch files and utilities needed for launching the entire AUV system in different environments and under different configurations. Nothing new should be defined here, and it should just be launching preexisting nodes in other packages.

## Launch Files

- `barebones_gazebo_launch.py` : All-in-one launch file which launches everything necessary for keyboard operation WITHIN GAZEBO.

    - If the simulation is running really slow, try following the optimization guide in the steelhead_gazebo README.md.

- `localization_visualization_gazebo_launch.py` : Launches a visualization WITHIN GAZEBO of where the robot "thinks" it is, visualized in RViz.

- `gazebo_test_slam_launch.py` : Launches the competition Gazebo world, hover controls, keyboard teleop, and the RTAB-Map RGB-D SLAM launch from `steelhead_localization`. Also launches rviz with configurations made to visualize mapping and localization. RTAB-Map-specific parameters and camera transforms are kept in `steelhead_localization/launch/rtab_map_launch.py`.

        ros2 launch steelhead_bringup gazebo_test_slam_launch.py

- `pool_test_x_launch.py` : Series of launch files for launching during pool time.

    - `all`: A whole bunch of nodes that would be the robot utilizuing every sensor it has.
    - `sensors`: Launches all of the nodes that publish information from the onboard sensors (currently the imu, cameras, and depth sensor.)
    - `hardcode`: Launches nodes that make the robot spin certain thrusters for a certain amount of time.

        - The hardcoded instructions are defined in `steelhead_controls/steelhead_controls/predetermined_thrust.py`.

    - `hardcode_pipeline`: Launches nodes that are neccessary for the hard coded hover route (serial, imu, depth sensor, PID, thrust allocator, hover script, and the pipeline running the competition sequence).

        - The hardcoded instructions are defined in `src/steelhead_pipeline/config/competition_hardcode_sequence.yaml`.

    - `ssh_teleop`: Launches the nodes that allow for teleoperation (keyboard control) of Steelhead. 

        - Because of certain limitations with SSH, the teleop can't be run in a launch file, so do `ros2 launch steelhead_bringup pool_test_ssh_teleop.py` alongside `ros2 run steelhead_teleop ssh_keyboard_teleop` in another terminal session.

- `gazebo_test_x_launch.py` : Series of launch files within Gazebo.

    - `hardcode_pipeline`: Runs the same hard coded competition sequence as the pool test version, but WITHIN GAZEBO (gazebo with the competition world, PID, thrust allocator, hover script, and the pipeline).

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
- Dorson Tang (dorsontang123@gmail.com)
# steelhead_bringup
## Description

This package is for creating launch files and utilities needed for launching the entire AUV system in different environments and under different configurations. Nothing new should be defined here, and it should just be launching preexisting nodes in other packages.

## Launch Files

- `barebones_gazebo_launch.py` : All-in-one launch file which launches everything necessary for keyboard operation WITHIN GAZEBO.

    - If the simulation is running really slow, try following the optimization guide in the steelhead_gazebo README.md.

- `localization_visualization_gazebo_launch.py` : Launches a visualization WITHIN GAZEBO of where the robot "thinks" it is, visualized in RViz.

- `pool_test_x_launch.py` : Series of launch files for launching during pool time.

    - `all`: A whole bunch of nodes that would be the robot utilizing every sensor it has.
    - `sensors`: Launches all of the nodes that publish information from the onboard sensors (currently the imu, cameras, and depth sensor.)
    - `hardcode`: Launches nodes that make the robot spin certain thrusters for a certain amount of time.

        - The hardcoded instructions are defined in `steelhead_controls/steelhead_controls/predetermined_thrust.py`.

    - `hardcode_pipeline`: Launches nodes that are necessary for the hard coded hover route (serial, imu, depth sensor, PID, thrust allocator, hover script, and the behaviour tree).

        - The hardcoded mission tree is defined in `src/steelhead_mission/missions/competition_hardcode.xml`.

    - `ssh_teleop`: Launches the nodes that allow for teleoperation (keyboard control) of Steelhead.

        - Because of certain limitations with SSH, the teleop can't be run in a launch file, so do `ros2 launch steelhead_bringup pool_test_ssh_teleop_launch.py` alongside `ros2 run steelhead_teleop ssh_keyboard_teleop` in another terminal session.

- `gazebo_test_x_launch.py` : Series of launch files within Gazebo.

    - `mission`: Runs the BehaviorTree.CPP `mission_runner` (`steelhead_mission`) inside a Gazebo competition world with YOLO detection, controls, and actuators simulation. Requires `world:=<world_file>` (e.g. `competition_task_gate.world`) and `mission:=<mission_file>` (e.g. `gate_only.xml`).
    - `vision`: Launches the basics required for computer vision within the robot, alongside a visualization of what the robot "sees".
    - `hover`: Launches the basics required for the robot to hover in place within the pool, with support for teleoperation.
    - `hardcode_pipeline`: Runs the same hard coded competition mission (`competition_hardcode.xml`) as the pool test version, but WITHIN GAZEBO.

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
- Dorson Tang (dorsontang123@gmail.com)
