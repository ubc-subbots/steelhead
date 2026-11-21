# steelhead_vision

## Description

This package contains vision-related nodes and utilities for the AUV, combining object recognition, gate detection, path marker detection, and vision processing utilities.

## Usage

### YOLO Detector

To launch the YOLO object detector, run:

    ros2 launch steelhead_vision yolo_detector_launch.py

### Gate Detector

To launch the gate detector component, run:

    ros2 launch steelhead_vision gate_detector_launch.py

### Path Marker Detector

To launch the orange path marker detector on the bottom camera, run:

    ros2 launch steelhead_vision path_marker_detector_launch.py

To see what it detects (marker outline in green, direction to follow as a red arrow), run:

    ros2 run rqt_image_view rqt_image_view /steelhead/path_marker/debug_image

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

- `path_marker_detector` : A Python node that finds the orange path marker in bottom camera images with an HSV colour mask. It takes the largest orange region, fits a line through it, and reports the marker's direction and offset relative to the robot. Assumes the top of the image is the robot's front and the right of the image is the robot's right.

  ### Parameters
  - `input_image_topic` (`string`, default: `/steelhead/drivers/bottom_camera/image_raw`) : Camera image to search.
  - `detection_topic` (`string`, default: `/steelhead/path_marker/detection`) : Topic for detection results.
  - `debug_image_topic` (`string`, default: `/steelhead/path_marker/debug_image`) : Topic for the annotated debug image.
  - `hsv_lower` (`int[]`, default: `[5, 100, 100]`) : Lower HSV bound of the marker colour.
  - `hsv_upper` (`int[]`, default: `[25, 255, 255]`) : Upper HSV bound of the marker colour.
  - `min_area_fraction` (`double`, default: `0.01`) : Smallest orange region, as a fraction of the image, that counts as a marker.

  ### Subscribed Topics
  - `/steelhead/drivers/bottom_camera/image_raw` (`sensor_msgs/msg/Image`) : Bottom camera image feed.

  ### Published Topics
  - `/steelhead/path_marker/detection` (`steelhead_interfaces/msg/PathMarker`) : One message per image: whether a marker is visible, the turn needed to line up with it, its sideways offset, and whether it is ahead of or behind the robot.
  - `/steelhead/path_marker/debug_image` (`sensor_msgs/msg/Image`) : Camera image with the detected marker and follow direction drawn on it. Only published while something is subscribed.

## Launch Files

- `yolo_detector_launch.py`: Starts the YOLO object detection node.
- `gate_detector_launch.py`: Starts the gate detection pipeline.
- `path_marker_detector_launch.py`: Starts the path marker detector with its topics and colour thresholds.

## Contributors

- Eddie Yoshie
