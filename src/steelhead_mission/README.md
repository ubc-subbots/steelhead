# steelhead_mission

## Description

This package contains the BehaviorTree.CPP v4 mission runner and all task action nodes (`Gate`, `Torpedo`, `Dropper`, `Octagon`, `PitchFlip`, and `DriveFor`) for the Steelhead AUV. It loads a behavior tree XML from `missions/`, ticks the tree at a configurable rate, and provides built-in support for task timeouts, retries, fallbacks, Groot2 live monitoring, and RViz status visualization.

## Usage

### Running a Mission Standalone

To launch the `mission_runner` node with a specific mission XML file (defaults to `gate_only.xml`):

    ros2 launch steelhead_mission mission_runner_launch.py mission:=gate_only.xml

### Running in Gazebo

To run the full Gazebo competition task simulations driven by `mission_runner`:

    ros2 launch steelhead_bringup gazebo_test_mission_launch.py mission:=gate_only.xml
    ros2 launch steelhead_bringup gazebo_test_competition_gate_launch.py
    ros2 launch steelhead_bringup gazebo_test_competition_torpedo_launch.py
    ros2 launch steelhead_bringup gazebo_test_competition_dropper_launch.py
    ros2 launch steelhead_bringup gazebo_test_competition_octagon_launch.py

### Monitoring

1. **Console:** `mission_runner` attaches a `BT::StdCoutLogger` that logs every node transition (`IDLE` -> `RUNNING` -> `SUCCESS` / `FAILURE`).
2. **ROS 2 Topic:** Echo the active leaf status string:

        ros2 topic echo /steelhead/mission/state

3. **RViz Marker:** Add a `Marker` display subscribed to `/steelhead/mission/status_marker` to view the active behavior tree state as 3D text in RViz (pre-configured in `gazebo_cv_test.rviz`).
4. **Groot2:** Connect Groot2 in Monitor mode to `localhost:1667` (configurable via `groot_port`; pass `groot_port:=0` to disable).

---

## Available Mission XMLs (`missions/`)

- `gate_only.xml` : Passes the competition gate (`<Gate>`) with a 120s timeout followed by a 2s hold (`<DriveFor>`).
- `torpedo_only.xml` : Aligns with the torpedo target and fires both torpedoes (`<Torpedo>`) with a 120s timeout followed by a 2s hold.
- `dropper_only.xml` : Centers over the first (`fire`) and second (`blood`) dropper bins and actuates the claw (`<Dropper>`) with a 120s timeout followed by a 2s hold.
- `octagon_only.xml` : Centers over the octagon target and surfaces (`<Octagon>`) with a 120s timeout followed by a 2s hold.
- `competition_hardcode.xml` : Hardcoded dead-reckoning competition route using `<DriveFor>` (and optional `<PitchFlip>`) legs.

---

## Available Behavior Tree Task Nodes (`mission` Namespace)

All task nodes are compiled into `libsteelhead_mission_tasks.so` and registered in `mission::registerTaskNodes`:

- `<Gate>` (`mission::GateTask`) : Front-camera YOLO alignment (`label`, `center_offset`, `pass_timeout`) and forward drive through the gate.
- `<Torpedo>` (`mission::TorpedoTask`) : Front-camera YOLO alignment (`label`, `center_offset`, `fire_alignment_threshold`) and sequential left/right torpedo firing via `/steelhead/controls/actuators_command`.
- `<Dropper>` (`mission::DropperTask`) : Bottom-camera 2D YOLO alignment (`label`, `second_label`, `center_offset_x`, `center_offset_y`, `drop_alignment_threshold`) and two-stage marker drop via `/steelhead/controls/actuators_command`.
- `<Octagon>` (`mission::OctagonTask`) : Bottom-camera 2D YOLO alignment (`label`, `center_offset_x`, `center_offset_y`, `surface_alignment_threshold`, `surface_duration`) and timed upward surfacing push.
- `<PitchFlip>` (`mission::PitchFlipTask`) : IMU quaternion-delta pitch flip (`flip_torque`, `flip_angle`) using `FULL` hover adjustment until handing back control to `hover_at_depth`.
- `<DriveFor>` (`mission::DriveFor`) : Timed open-loop wrench command (`force_x`, `force_y`, `force_z`, `torque_x`, `torque_y`, `torque_z`, `duration`, `full_override`).

---

## Creating a New Mission XML

Mission files live in `src/steelhead_mission/missions/` and use the BehaviorTree.CPP v4 XML format (`BTCPP_format="4"`).

### 1. Create the XML File

Add a new `.xml` file in `src/steelhead_mission/missions/` (e.g., `my_mission.xml`):

```xml
<root BTCPP_format="4" main_tree_to_execute="MyMission">
  <BehaviorTree ID="MyMission">
    <Sequence name="main_sequence">
      <!-- Try the gate for up to 120s; if it times out or fails, dead-reckon forward -->
      <Fallback name="gate_with_fallback">
        <Timeout msec="120000">
          <Gate name="pass_gate"
                label="sos"
                center_offset="0.1"
                pass_timeout="4.0"/>
        </Timeout>
        <DriveFor name="fallback_drive"
                  force_x="10.0"
                  duration="5.0"/>
      </Fallback>

      <!-- Transit leg after passing the gate -->
      <DriveFor name="transit_to_next_task"
                force_x="15.0"
                duration="6.0"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 2. Built-in Control and Decorator Tags

BehaviorTree.CPP provides these tags out of the box (no C++ changes needed):

- `<Sequence>`: Ticks children in order until all return `SUCCESS` (aborts immediately if any child returns `FAILURE`).
- `<Fallback>`: Ticks children in order until one returns `SUCCESS` (used for recovery/fallback branches when a primary task fails or times out).
- `<Timeout msec="N">`: Wraps a child node and halts it (calling `onHalted()` to zero thrusters) if it does not finish within `N` milliseconds, returning `FAILURE`.
- `<RetryUntilSuccessful num_attempts="N">`: Retries a child node up to `N` times if it returns `FAILURE`.
- `<Parallel success_count="M" failure_count="K">`: Ticks multiple children concurrently on every tick.

### 3. Build and Run

Install the new XML into `share/steelhead_mission/missions/` and launch it:

    colcon build --packages-select steelhead_mission
    source install/setup.bash
    ros2 launch steelhead_mission mission_runner_launch.py mission:=my_mission.xml

---

## Creating a New Mission Task Node (C++)

Adding a new task node requires **3 steps** in `steelhead_mission` (no `.msg`/`.srv`/`.action` or YAML files required):

### Step 1: Implement a `BT::StatefulActionNode` in `steelhead_mission`

Create `include/steelhead_mission/tasks/my_task.hpp` and `src/tasks/my_task.cpp` (see `gate_task.hpp` / `drive_for.hpp` for full examples). Every task node receives the shared `rclcpp::Node::SharedPtr` in its constructor:

```cpp
// include/steelhead_mission/tasks/my_task.hpp
#ifndef STEELHEAD_MISSION__TASKS__MY_TASK
#define STEELHEAD_MISSION__TASKS__MY_TASK

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"

namespace mission
{

    class MyTask : public BT::StatefulActionNode
    {
        public:

            MyTask(
                const std::string & name,
                const BT::NodeConfig & config,
                rclcpp::Node::SharedPtr node);

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<std::string>("label", "target", "Target YOLO class label"),
                    BT::InputPort<double>("threshold", 0.1, "Alignment error threshold")
                };
            }

            // Called once when the node transitions from IDLE -> RUNNING.
            // Reset per-attempt member state and read input ports here.
            BT::NodeStatus onStart() override;

            // Called on every tree tick (20 Hz) while RUNNING.
            // Return RUNNING while working, SUCCESS when complete, or FAILURE to trigger a Fallback.
            BT::NodeStatus onRunning() override;

            // Called automatically when preempted by a Timeout or Fallback.
            // MUST publish a zero PARTIAL HoverAdjustment wrench so the sub stops cleanly.
            void onHalted() override;

        private:

            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;
    };

} // namespace mission

#endif  //STEELHEAD_MISSION__TASKS__MY_TASK
```

> **Important:** Do **not** create `rclcpp::TimerBase` timers inside a BT node, and do **not** publish commands inside subscription callbacks. Subscription callbacks should only cache the latest message; `onRunning()` is ticked by `mission_runner` at `tick_rate_hz` (20 Hz) so only the currently active task node commands the thrusters. Always zero `controls/hover_adjust` in `onHalted()` and before returning `SUCCESS`/`FAILURE`.

### Step 2: Register the Node in `register_nodes.cpp` and `CMakeLists.txt`

1. Add `src/tasks/my_task.cpp` to `add_library(steelhead_mission_tasks SHARED ...)` in `src/steelhead_mission/CMakeLists.txt`.
2. Add one line in `src/steelhead_mission/src/tasks/register_nodes.cpp`:

```cpp
#include "steelhead_mission/tasks/my_task.hpp"

void registerTaskNodes(
    BT::BehaviorTreeFactory & factory,
    const rclcpp::Node::SharedPtr & node)
{
    factory.registerNodeType<GateTask>("Gate", node);
    factory.registerNodeType<TorpedoTask>("Torpedo", node);
    factory.registerNodeType<DropperTask>("Dropper", node);
    factory.registerNodeType<OctagonTask>("Octagon", node);
    factory.registerNodeType<PitchFlipTask>("PitchFlip", node);
    factory.registerNodeType<DriveFor>("DriveFor", node);
    factory.registerNodeType<MyTask>("MyTask", node);
}
```

### Step 3: Use the Tag in Any Mission XML

Rebuild `steelhead_mission` and reference `<MyTask>` directly in your mission XML:

```xml
<Timeout msec="60000">
  <MyTask name="align_and_act" label="circle" threshold="0.08"/>
</Timeout>
```

---

## Nodes

- `mission_runner` : A standalone C++ node that constructs and ticks a BehaviorTree.CPP v4 tree using the task nodes in `steelhead_mission_tasks` (`mission::registerTaskNodes`).

  ### Subscribed Topics

  - `yolo_detector/front/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Front camera YOLO detections (`Gate`, `Torpedo`).
  - `drivers/front_camera/camera_info` (`sensor_msgs/msg/CameraInfo`) : Front camera dimensions (`Gate`, `Torpedo`).
  - `yolo_detector/bottom/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Bottom camera YOLO detections (`Dropper`, `Octagon`).
  - `drivers/bottom_camera/camera_info` (`sensor_msgs/msg/CameraInfo`) : Bottom camera dimensions (`Dropper`, `Octagon`).
  - `drivers/imu/out` (`sensor_msgs/msg/Imu`) : IMU orientation (`PitchFlip`).

  ### Service Clients

  - `/steelhead/controls/actuators_command` (`steelhead_interfaces/srv/ActuatorsCommand`) : Torpedo and claw actuator commands (`Torpedo`, `Dropper`).

  ### Published Topics

  - `controls/hover_adjust` (`steelhead_interfaces/msg/HoverAdjustment`) : Partial/full wrench adjustments published by active task nodes, and zeroed automatically when tasks halt or the mission terminates.
  - `mission/state` (`std_msgs/msg/String`) : Human-readable active behavior tree leaf and status.
  - `mission/status_marker` (`visualization_msgs/msg/Marker`) : 3D `TEXT_VIEW_FACING` marker showing active mission state in RViz.

  ### Parameters

  - `mission_file` (`string`, default `"gate_only.xml"`) : Filename in `steelhead_mission/missions/` or absolute path to a BehaviorTree.CPP v4 XML file.
  - `tick_rate_hz` (`double`, default `20.0`) : Frequency at which the behavior tree is ticked.
  - `groot_port` (`int`, default `1667`) : TCP port for `BT::Groot2Publisher` live monitoring (`0` disables Groot2).
  - `marker_frame` (`string`, default `"odom"`) : TF frame used for the RViz status marker.

## Contributors

- Dorson Tang (dtang08@student.ubc.ca)
