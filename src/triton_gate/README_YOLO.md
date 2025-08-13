# YOLO Gate Detection Integration

This document describes the integration of YOLOv11 gate detection into the triton_gate package.

## Overview

The package now supports both traditional OpenCV-based gate detection and modern YOLO-based detection:

1. **OpenCV Detection** (original): Uses color segmentation and pole detection
2. **YOLO Detection** (new): Uses YOLOv11 to detect "Full Gate" objects (class 2)

## Setup Instructions

### 1. Install Dependencies

```bash
# Install ultralytics for YOLO
pip install ultralytics

# Install tf_transformations for quaternion calculations
pip install tf_transformations
```

### 2. Model Setup

Place your YOLO model file (`arvp_front.pt`) in an accessible location. The default path is relative to where you launch the node.

### 3. Build the Package

```bash
cd /path/to/your/workspace
colcon build --packages-select triton_gate
source install/setup.bash
```

## Usage

### Launch YOLO Gate Detector Only

```bash
ros2 launch triton_gate yolo_gate_detector_launch.py model_path:=/path/to/arvp_front.pt debug:=true
```

### Launch Original OpenCV Detector (if needed)

```bash
ros2 launch triton_gate gate_detector_launch.py debug:=true
```

### Parameters

- `model_path`: Path to the YOLO model file (default: `arvp_front.pt`)
- `debug`: Enable debug visualization and extra logging (default: `true`)
- `confidence_threshold`: Minimum confidence for YOLO detections (default: `0.5`)

## Testing

### Monitor Detection Topics

```bash
# Watch gate detections (Full Gate only)
ros2 topic echo /triton/gate/detector/gate_pose

# Watch offset data (Full Gate only)  
ros2 topic echo /triton/gate/detector/gate_offset

# Watch ALL detections (all object classes)
ros2 topic echo /triton/gate/detector/all_detections

# View debug visualization (if debug=true)
ros2 run rqt_image_view rqt_image_view /triton/gate/detector/debug/yolo_detection
```

## Topics Published

The YOLO detector publishes the same topics as the original detector for compatibility:

- `/triton/gate/detector/gate_pose` (triton_interfaces/msg/ObjectOffset) - Full Gate detections only
- `/triton/gate/detector/gate_offset` (std_msgs/msg/Float32MultiArray) - Full Gate offset only
- `/triton/gate/detector/gate_pose_only` (geometry_msgs/msg/PoseStamped) - debug only
- `/triton/gate/detector/debug/yolo_detection` (sensor_msgs/msg/Image) - debug only

**New topic for all detections:**
- `/triton/gate/detector/all_detections` (std_msgs/msg/Float32MultiArray) - All object detections

### All Detections Format

The `/triton/gate/detector/all_detections` topic contains data for all detected objects in this format:
```
[class_id, confidence, center_x, center_y, width, height, class_id, confidence, center_x, center_y, width, height, ...]
```

Each detection uses 6 float values:
- `class_id`: Object class (0-7, see table below)  
- `confidence`: Detection confidence (0.0-1.0)
- `center_x, center_y`: Center coordinates in pixels
- `width, height`: Bounding box dimensions in pixels

## YOLO Model Information

The system uses the "Full Gate" class (class ID 2) from the arvp_front.pt model:

```
Class ID | Obstacle
---------|----------
0        | Sawfish Gate Banner
1        | Shark Gate Banner
2        | Full Gate          <-- Used by this detector
3        | Red Slalom
4        | White Slalom
5        | Full Torpedo Banner
6        | Sawfish Torpedo Hole
7        | Shark Torpedo Hole
```

## Detection Output Format

The YOLO detector converts YOLO's xywh format (center_x, center_y, width, height) to match the existing system's expectations:

1. **Distance Calculation**: Uses the same focal length formula as the original detector
2. **Position Offset**: Calculates offset from image center to gate center
3. **3D Pose**: Converts 2D detection to 3D pose in ENU coordinates
4. **Quaternion**: Sets orientation quaternion (currently assumes head-on approach)

## Debug Visualization

When debug mode is enabled, the detector will:

- Draw bounding boxes around detected gates
- Show confidence scores
- Display distance estimates
- Draw center points and offset lines
- Publish debug images to `/triton/gate/detector/debug/yolo_detection`

## Logging

The detector logs:
- Model loading status
- Detection confidence scores
- Error messages for troubleshooting

## Troubleshooting

1. **Model not found**: Ensure the model path is correct and the file exists
2. **No detections**: Check confidence threshold and verify the camera feed
3. **Import errors**: Make sure ultralytics and tf_transformations are installed
4. **Topic conflicts**: If running both detectors, they will publish to the same topics

## Comparison with OpenCV Detector

| Feature | OpenCV Detector | YOLO Detector |
|---------|----------------|---------------|
| Method | Color segmentation + pole detection | Deep learning object detection |
| Robustness | Sensitive to lighting/color changes | More robust to environmental conditions |
| Speed | Fast | Moderate (depends on hardware) |
| Accuracy | Good in ideal conditions | Generally more accurate |
| Setup | No external models needed | Requires YOLO model file |

## Future Improvements

1. **Multi-class detection**: Detect other gate types (Sawfish, Shark banners)
2. **Orientation estimation**: Calculate gate orientation from bounding box
3. **Distance calibration**: Fine-tune distance calculations for YOLO detections
4. **Performance optimization**: Optimize inference speed for real-time performance