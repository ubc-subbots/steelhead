# steelhead_mission

## Description

This package contains the BehaviorTree.CPP v4 mission runner and all the task action nodes. It loads a behavior tree XML from `missions/`, ticks the tree at a configurable rate, and provides built-in support for task timeouts, retries, fallbacks, and Groot2 live monitoring.

## Usage

### Running a Mission Standalone

To launch the `mission_runner` node with a specific mission XML file (defaults to `gate_only.xml`):

    ros2 launch steelhead_mission mission_runner_launch.py mission:=gate_only.xml

### Running in Gazebo

To run the Gazebo competition task simulations driven by `mission_runner`:

    ros2 launch steelhead_bringup gazebo_test_mission_launch.py world:=competition.world mission:=gate_only.xml

### Monitoring

The node automatically uploads its status to the local network when ran. Download and install [Groot2](https://www.behaviortree.dev/groot/) and connect to `localhost:1667` to visualize the behaviour tree.

## Behavior Tree Task Nodes

All task nodes are compiled into `libsteelhead_mission_tasks.so` and registered in `mission::registerTaskNodes`:

- `<Gate>` (`mission::GateTask`) : Front-camera YOLO alignment (`label`, `center_offset`, `pass_timeout`) and forward drive through the gate.
- `<DriveFor>` (`mission::DriveFor`) : Timed open-loop wrench command (`force_x`, `force_y`, `force_z`, `torque_x`, `torque_y`, `torque_z`, `duration`, `full_override`).

## Creating a New Mission XML

Mission files live in `src/steelhead_mission/missions/` and use the BehaviorTree.CPP v4 XML format. You could also visually make a mission using Groot2, but I haven't tried it yet.

### 1. Create the XML File

Add a new `.xml` file in `src/steelhead_mission/missions/`.

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

BehaviorTree.CPP provides these tags out of the box:

- `<Sequence>`: Ticks children in order until all return `SUCCESS` (aborts immediately if any child returns `FAILURE`).
- `<Fallback>`: Ticks children in order until one returns `SUCCESS` (used for recovery/fallback branches when a primary task fails or times out).
- `<Timeout msec="N">`: Wraps a child node and halts it (calling `onHalted()` to zero thrusters) if it does not finish within `N` milliseconds, returning `FAILURE`.
- `<RetryUntilSuccessful num_attempts="N">`: Retries a child node up to `N` times if it returns `FAILURE`.
- `<Parallel success_count="M" failure_count="K">`: Ticks multiple children concurrently on every tick.

## Creating a New Mission Task Node 

> **Tip:** You can ask an AI agent to scaffold this automatically using the `create-task-node` skill!

### Step 1: Implement a `BT::StatefulActionNode` in `steelhead_mission`

Create `include/steelhead_mission/tasks/my_task.hpp` and `src/tasks/my_task.cpp`. 

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

### Step 2: Register the Node in `register_nodes.cpp` and `CMakeLists.txt`

1. Add `src/tasks/my_task.cpp` to `add_library(steelhead_mission_tasks SHARED ...)` in `src/steelhead_mission/CMakeLists.txt`.
2. Add one line in `src/steelhead_mission/src/register_nodes.cpp`:

```cpp
#include "steelhead_mission/tasks/my_task.hpp"

void registerTaskNodes(
    BT::BehaviorTreeFactory & factory,
    const rclcpp::Node::SharedPtr & node)
{
    factory.registerNodeType<GateTask>("Gate", node);
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

## Nodes

- `mission_runner` : A standalone node that constructs and ticks a BehaviorTree.CPP v4 tree using the task nodes.

  ### Parameters

  - `mission_file` (`string`, default `"gate_only.xml"`) : Filename in `steelhead_mission/missions/` or absolute path to a BehaviorTree.CPP v4 XML file.
  - `tick_rate_hz` (`double`, default `20.0`) : Frequency at which the behavior tree is ticked.
  - `groot_port` (`int`, default `1667`) : TCP port for `BT::Groot2Publisher` live monitoring (`0` disables Groot2).

## Contributors

- Dorson Tang (dtang08@student.ubc.ca)
