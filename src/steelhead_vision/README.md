# steelhead_vision

## Description

This package contains vision-related nodes and utilities for the AUV, combining object recognition, gate detection, and vision processing utilities.

## Usage

### YOLO Detector

To launch the YOLO object detector, run:

    ros2 launch steelhead_vision yolo_detector_launch.py

The detector runs the OpenVINO versions of the models (`config/ffc_rs_26_openvino_model/` and `config/dfc_rs_26_openvino_model/`), which run faster on the CPU than the `.pt` files. This needs `pip install openvino --break-system-packages`. If a model is retrained, re-export it from its `.pt` file:

    yolo export model=src/steelhead_vision/config/ffc_rs_26.pt format=openvino half=True imgsz=640

To use the original PyTorch model instead, set the node's `weights_path` parameter to the `.pt` file.

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

  ### Parameters
  - `debug` (`bool`, default: `false`) : Enables debug image and pose publishers.
  - `camera_frame` (`string`, default: `camera_link_optical`) : Camera optical frame for 3D detection.
  - `target_frame` (`string`, default: `base_link`) : Robot reference frame to which the gate pose is transformed via TF2.

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
