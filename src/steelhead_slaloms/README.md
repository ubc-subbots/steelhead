# steelhead_slaloms

## Description

This package detects the red slalom poles using color-based computer vision (no ML) and publishes the estimated pose of the gap between them. The approach mirrors `steelhead_gate` — HSV segmentation → convex hulls → pole identification → pose estimation — adapted for red poles instead of orange gate poles.

## Nodes

### `SlalomDetector` (composable)

Subscribes to the front camera and publishes the estimated position of the passable gap between the two nearest visible slalom poles.

**Subscribes to:**
- `/steelhead/drivers/front_camera/image_raw` (`sensor_msgs/Image`)

**Publishes:**
- `/steelhead/slaloms/detector/pole_pose` (`steelhead_interfaces/ObjectOffset`) — estimated 3D pose of the gap
- `/steelhead/slaloms/detector/pole_offset` (`std_msgs/Float32MultiArray`) — pixel `[offset_x, offset_y]` of gap from image centre

**Debug-only publishes** (when `debug: True`):
- `/steelhead/slaloms/detector/debug/detection` — annotated camera image
- `/steelhead/slaloms/detector/debug/segment` — binary red mask
- `/steelhead/slaloms/detector/pole_pose_only` (`geometry_msgs/PoseStamped`) — for RViz

**Parameters:**
- `debug` (bool, default `false`) — enables debug image publishing and annotations

## Detection Algorithm

1. Convert BGR frame to HSV
2. Threshold for red (two `inRange` calls — red wraps at 0°/180° in HSV — OR'd together)
3. Morphological cleanup (from `steelhead_vision_utils`)
4. Convex hulls on connected components
5. Filter hulls by aspect ratio (taller than wide = vertical pole)
6. Take two largest qualifying hulls as the two poles
7. Compute gap midpoint and estimate distance via pinhole model

## HSV Tuning

The default red ranges are:
- Lower red: H 0–10, S 60–255, V 60–255
- Upper red: H 160–180, S 60–255, V 60–255

These will likely need tuning in Gazebo. Run with `debug: True` and inspect
`/steelhead/slaloms/detector/debug/segment` to see what the mask picks up.

## Distance Calibration

The pinhole distance estimate in `detectPolesAndPublish` uses placeholder values:
```
standard_pixel_width = 20.0   # px width of pole at 1 m
standard_width       = 0.05   # real pole diameter in metres
```
Measure the actual pixel width of a pole at a known distance in simulation and update accordingly.

## Launch Files

- `launch/slalom_detector_launch.py` — detector only, `debug: True`, for development
- `steelhead_bringup/pool_test_slaloms_launch.py` — full pool test stack

## Contributors

- Krishna Garcha (garchakri@gmail.com)