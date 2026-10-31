# steelhead_localization

## Description

This package deals with localizing the AUV within the pool (figuring out where it is.)

## Usage

### Robot Localization

To run the robot localization node, which synthesizes odometry and IMU data, run

        ros2 launch steelhead_localization state_estimator_launch.py

## Nodes

- `state_estimator` : A standalone node which subscribes to the IMU, odometry, and (eventually) sound localization position topics, and publishes an estimated state of the AUV, including position, velocity, and acceleration synthesizing those sensors and their data.

  ### Subscribed Topics

  - `/steelhead/drivers/imu/out` (`sensor_msgs/Imu`) : IMU data: linear acceleration and angular velocity
  - `/odom` (`nav_msgs/Odometry`) : Odometry: This is the absolute state of the robot, something we most likely won't have and is for simulation purposes only

  ### Published Topics

  - `controls/ukf/odometry/filtered` (`nav_msgs/Odometry`) : The final synthesized state of the AUV

  ### Notes

  - This node is configured with the config file `state_estimator_config.yaml`.





  ### Fiducial Localization

We use symbols throughout the simulator to find where the auv's position is located with an error estimate

Run it on its own:

        ros2 launch steelhead_localization fiducial_localizer_launch.py symbols:=gate

`symbols` picks the symbol file:
 - We use `gate` for the gate world
 - we use `torpedo` for the torpedo world. 
 Add `use_sim_time:=true` in the simulator.

Test it in Gazebo:

        ros2 launch steelhead_bringup gazebo_test_fiducial_launch.py

For the torpedo board, add `world:=competition_task_torpedo.world symbols:=torpedo`.

Drive the robot in a second terminal:

        ros2 run steelhead_teleop ssh_keyboard_pid_teleop

Start driving toward the gate right away, because the robot drifts if left alone. The node only gives a position once YOLO sees a symbol, which happens within about 2 m of the gate.

See the estimate, and the true position to compare:

        ros2 topic echo /steelhead/localization/fiducial/pose --field pose.pose.position
        ros2 topic echo /steelhead/state --field pose.pose.position

Run the tests:

        colcon test --packages-select steelhead_localization
        colcon test-result --verbose


- `fiducial_localizer` : Works out the robot's position from symbols YOLO detects.

  For each symbol it sees, it:
  1. works out how far away the symbol is (smaller in the image = farther away),
  2. works out which direction it is in (from where it is in the image),
  3. uses the symbol's known position to work out where the robot must be.

  If it sees more than one symbol, it averages them and trusts closer ones more.

  ### Subscribed Topics

  - `/steelhead/yolo_detector/front/detections` (`steelhead_interfaces/DetectionBoxArray`) : What YOLO sees in the front camera
  - `/steelhead/drivers/front_camera/camera_info` (`sensor_msgs/CameraInfo`) : Camera settings
  - `/steelhead/drivers/imu/out` (`sensor_msgs/Imu`) : Which way the robot is facing

  ### Published Topics

  - `/steelhead/localization/fiducial/pose` (`geometry_msgs/PoseWithCovarianceStamped`) : The robot's estimated position, measured from where it started. It also includes an error estimate, which gets bigger the farther away the symbols are.

  ### Notes

  - Symbol positions are in `config/fiducial_symbols_gate.yaml` and `config/fiducial_symbols_torpedo.yaml`. Each one has a YOLO `label`, a position (`x`, `y`, `z` in metres from the robot's start) and the symbol's real `height`.
  - Symbols that appear more than once (like the four torpedo circles) are skipped, because we can't tell which one we're seeing.
  - It ignores symbols cut off at the edge of the image, low-confidence detections, and sudden big jumps.
  - Only the front camera is supported for now.
  - The math is in `fiducial_geometry.py` and is tested in `test/`.
