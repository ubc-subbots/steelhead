# steelhead_mission

## Description

This package contains the BehaviorTree.CPP v4 mission runner for the Steelhead AUV. It loads a behavior tree XML from `missions/`, registers the task action nodes defined in `steelhead_tasks`, ticks the tree at a configurable rate, and provides built-in support for task timeouts, retries, fallbacks, Groot2 live monitoring, and RViz status visualization.

## Usage

### Running a Mission Standalone

To launch the `mission_runner` node with a specific mission XML file (defaults to `gate_only.xml`):

    ros2 launch steelhead_mission mission_runner_launch.py mission:=gate_only.xml

### Running in Gazebo

To run the full Gazebo competition gate simulation driven by the behavior tree mission runner:

    ros2 launch steelhead_bringup gazebo_test_mission_launch.py mission:=gate_only.xml

### Monitoring and Visualization

1. **Console:** `mission_runner` attaches a `BT::StdCoutLogger` that logs every node transition (`IDLE` -> `RUNNING` -> `SUCCESS` / `FAILURE`).
2. **ROS 2 Topic:** Echo the active leaf status string:

        ros2 topic echo /steelhead/mission/state

3. **RViz Marker:** Add a `Marker` display subscribed to `/steelhead/mission/status_marker` to view the active behavior tree state as 3D text in RViz (pre-configured in `gazebo_test_mission_launch.py`).
4. **Groot2:** Connect Groot2 in Monitor mode to `localhost:1667` (configurable via `groot_port`).

## Nodes

- `mission_runner` : A standalone node that constructs and ticks a BehaviorTree.CPP v4 tree using shared task nodes from `steelhead_tasks::registerTaskNodes`.

  ### Subscribed Topics

  - Task-dependent subscriptions created on the shared node (e.g., `yolo_detector/front/detections` and `drivers/front_camera/camera_info` for `Gate`).

  ### Published Topics

  - `mission/state` (`std_msgs/msg/String`) : Human-readable active behavior tree leaf and status.
  - `mission/status_marker` (`visualization_msgs/msg/Marker`) : 3D `TEXT_VIEW_FACING` marker showing active mission state in RViz.
  - `controls/hover_adjust` (`steelhead_interfaces/msg/HoverAdjustment`) : Partial wrench adjustments published by active task nodes, and zeroed automatically when tasks halt or the mission terminates.

  ### Parameters

  - `mission_file` (`string`, default `"gate_only.xml"`) : Filename in `steelhead_mission/missions/` or absolute path to a BehaviorTree.CPP v4 XML file.
  - `tick_rate_hz` (`double`, default `20.0`) : Frequency at which the behavior tree is ticked.
  - `groot_port` (`int`, default `1667`) : TCP port for `BT::Groot2Publisher` live monitoring (`0` disables Groot2).
  - `marker_frame` (`string`, default `"odom"`) : TF frame used for the RViz status marker.
