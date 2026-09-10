# steelhead_object_recognition
## Description

This package contains a ROS2 node that runs a custom-trained YOLO model
(Ultralytics) on incoming camera images, annotates detections, and republishes
the annotated image for RViz visualization. The model weights ship with the
package at `config/competition.pt`.

Inference runs on CPU and is throttled (default once per second) to keep it
lightweight. If the `ultralytics` package or the weights file are unavailable,
the node falls back to republishing the raw image so the pipeline keeps running.

## Usage

    ros2 launch steelhead_object_recognition yolo_detector_launch.py

## Nodes

- `yolo_detector` : Runs YOLO inference and publishes an annotated image.

    ### Parameters
    - `weights_path` (string) : Path to the YOLO `.pt` weights. Defaults to the
      packaged `config/competition.pt` resolved from the install share dir.
    - `confidence_threshold` (double) : Minimum detection confidence.
    - `inference_interval` (double) : Seconds between inference runs.

    ### Subscribed Topics
    - `/steelhead/drivers/front_camera/image_raw` (`sensor_msgs/msg/Image`) : Input image.

    ### Published Topics
    - `/steelhead/yolo_detector/annotated_image` (`sensor_msgs/msg/Image`) : Annotated output image.
    - `/steelhead/yolo_detector/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Pixel coordinates for each detected object. 

## Contributors

- Kevin Huang (kevinh42@student.ubc.ca)
