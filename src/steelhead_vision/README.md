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
  - `camera/image_raw` (`sensor_msgs/msg/Image`) : Raw image feed.

  ### Published Topics
  - `vision/yolo/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Array of detected objects.

- `gate_detector` : A component node that detects the competition gate and publishes its relative pose.

  ### Subscribed Topics
  - `camera/image_raw` (`sensor_msgs/msg/Image`) : Raw image feed.

  ### Published Topics
  - `vision/gate/pose` (`geometry_msgs/msg/PoseStamped`) : Pose of the gate relative to the AUV.

- `object_detector` : A general C++ component utility for processing image contours and detecting predefined objects.

## Launch Files

- `yolo_detector_launch.py`: Starts the YOLO object detection node.
- `gate_detector_launch.py`: Starts the gate detection pipeline.
