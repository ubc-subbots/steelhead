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