# steelhead_sensors

## Description

This package contains the nodes related to gathering sensory information for the AUV.

## Usage

### Depth Sensor

To launch the depth sensor publisher, run:

    ros2 launch steelhead_sensors depth_sensor_publisher_launch.py

### IMU Sensor

To launch the IMU publisher and serial subscriber, run:

    ros2 launch steelhead_sensors imu_publisher_launch.py

### Cameras

To launch the camera publishers, run:

    ros2 launch steelhead_sensors cameras_publisher_launch.py

## Nodes

- `bno085_imu_publisher` : A node that publishes orientation data from the BNO085 IMU.

  ### Published Topics
  - `/steelhead/drivers/imu/out` (`sensor_msgs/msg/Imu`) : Orientation of the IMU.

- `ms5837_depth_publisher` : A node that publishes depth data from the MS5837 sensor.

  ### Published Topics
  - `drivers/depth_sensor` (`steelhead_interfaces/msg/DepthSensor`) : Contains depth, pressure and temperature.

- `serial_subscriber` : A component node that reads serial communication from the Arduino.

  ### Published Topics
  - `sensors/arduino/serial` (`std_msgs/msg/String`) : Raw string data from the Arduino.

## Launch Files

- `cameras_publisher_launch.py`: Starts the nodes necessary to utilize the usb cameras onboard Steelhead.
- `depth_sensor_publisher_launch.py`: Starts the depth sensor publisher.
- `imu_publisher_launch.py`: Starts the IMU publisher.

