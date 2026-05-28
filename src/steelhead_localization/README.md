# steelhead_localization

## Description

This package deals with localizing the AUV within the pool (figuring out where it is.)

## Usage

### Robot Localization

To run the robot localization node, which synthesizes odometry and IMU data, run

        ros2 launch steelhead_localization state_estimator_launch.py

### Visual-inertial Odometry

To run the VINS odometry model, which estimates position based on a monocular camera and an IMU, run

        ros2 launch steelhead_localization vins_odometry_launch.py

## Nodes

- `state_estimator` : A standalone node which subscribes to the IMU, visual odometry, and (eventually) sound localization position topics, and publishes an estimated state of the AUV, including position, velocity, and acceleration synthesizing those sensors and their data.

  ### Subscribed Topics

  - `/steelhead/drivers/imu/out` (`sensor_msgs/Imu`) : IMU data: linear acceleration and angular velocity
  - `/vins_estimator/odometry` (`nav_msgs/Odometry`) : Odometry: Raw position, orientation, angular, and linear velocity estimated from visual-odometry

  ### Published Topics

  - `controls/ukf/odometry/filtered` (`nav_msgs/Odometry`) : The final synthesized state of the AUV

  ### Notes

  - This node is configured with the config file `state_estimator_config.yaml`.

- `vins_estimator` : A node that estimates position based on IMU data and feature points from a monocular camera (visual-inertial odometry)

  ### Subscribed Topics

  - `/feature_tracker/feature` (`sensor_msgs/msg/PointCloud`) : The point cloud generated from the feature tracker running on the camera, which tracks rough edges based on pixel data.
  - `/steelhead/drivers/imu/out` (`sensor_msgs/msg/Imu`) : IMU data: linear acceleration and angular velocity

  ### Published Topics

  - `/vins_estimator/odometry` (`nav_msgs/Odometry`) : The estimated state of the AUV
  - `/vins_estimator/path:` (`nav_msgs/msg/Path`) : The estimated path that the AUV followed

  ### Notes

  - This node requires both the feature_tracker node as well as the camera_model node. For simplicities sake, just use the launch file and treat it as a black box, otherwise, check [here](https://github.com/dongbo19/VINS-MONO-ROS2?tab=readme-ov-file) for more information.
  - This node requires [Ceres Solver](http://ceres-solver.org/installation.html) 1.14.0, which needs to be installed from source. Instructions are in the main repo README.md. 
  - This node publishes more topics than necessary, the major ones are listed (use ros2 node info for a full list.)
  - !TODO: put launch file under proper namespaces at some point (everything follows /vins_estimator/, when it should be something like /steelhead_localization/).

- `fiducial_pose_estimator` : A node that uses the position of an object/symbol marker relative to the camera frame which has a known global pose to publish a mapping from map to odom, effectively giving the global position of the camera.

  ### Subscribed Topics

  - `/steelhead/localization/fiducial/marker/pose/relative` (`geometry_msgs/msg/Pose`) : The position of the marker relative to the camera.
  - `/steelhead/localization/fiducial/marker/pose/global` (`geometry_msgs/msg/Pose`) : The position of the marker in global space.

  ### Published Topics

  - `/steelhead/localization/fiducial/camera/pose/global` (`geometry_msgs/msg/Pose`) : The estimated position of the AUV in global space.

  ### Notes

  - This node relies on the pose relative to the camera, which will most likely be obtained using IPPE.
  - This node must already have the transformation from the camera to the base_link of the robot.
  - Since only so many symbols can be in view, this should be use alongside robot_localization to enhance a global estimation.
  - We define global as relative to the starting position of the submarine.
 
## Acknowledgements

The visual-inertial odometry node utilizes the implementation taken from [VINS-MONO-ROS2](https://github.com/dongbo19/VINS-MONO-ROS2?tab=readme-ov-file), and by extension is released under the GPLv3 license.