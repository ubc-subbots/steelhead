# steelhead_vision

## Description

This package contains vision-related nodes and utilities for the AUV, combining object recognition, gate detection, and vision processing utilities.

## Usage

### YOLO Detector

To launch the YOLO object detector, run:

    ros2 launch steelhead_vision yolo_detector_launch.py

### Gate Detector

To launch the gate detector component, run:

    ros2 launch steelhead_vision gate_detector_launch.py

## Nodes

- `yolo_detector` : A node that runs YOLO object detection on camera feeds.

  ### Subscribed Topics
  - `/steelhead/drivers/front_camera/image_raw` (`sensor_msgs/msg/Image`) : Front camera image feed.
  - `/steelhead/drivers/bottom_camera/image_raw` (`sensor_msgs/msg/Image`) : Bottom camera image feed.

  ### Published Topics
  - `/steelhead/yolo_detector/front/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Front camera object detections.
  - `/steelhead/yolo_detector/bottom/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Bottom camera object detections.

- `gate_detector` : A component node that detects the competition gate and publishes its relative pose.

  ### Subscribed Topics
  - `/steelhead/drivers/front_camera/image_raw` (`sensor_msgs/msg/Image`) : Front camera image feed.

  ### Published Topics
  - `detector/gate_pose` (`steelhead_interfaces/msg/ObjectOffset`) : Pose and classification offset of the gate.
  - `detector/gate_offset` (`std_msgs/msg/Float32MultiArray`) : Pixel offset data for the gate.
  - `detector/gate_pose_only` (`geometry_msgs/msg/PoseStamped`) : Debug-only isolated pose of the gate.

- `object_detector` : A general C++ component utility for processing image contours and detecting predefined objects.

## Launch Files

- `yolo_detector_launch.py`: Starts the YOLO object detection node.
- `gate_detector_launch.py`: Starts the gate detection pipeline.
