---
name: create-task-node
description: >-
  Use this skill when the user wants to create, scaffold, or generate boilerplate for
  a new BehaviorTree.CPP task action node in steelhead_mission. Guides gathering
  requirements and generates all necessary C++ code, registrations, and build
  configuration updates.
---

# Create Task Node Boilerplate

This skill provides step-by-step instructions to scaffold a new BehaviorTree.CPP task action node in `steelhead_mission`.

---

## 1. Gather Requirements

First, ask the user:
1. **Desired Task Name:**
   - PascalCase class name (e.g., `GateTask` or tag name `Gate`) and snake_case file name (e.g., `gate_task`).
2. **Task Functionality & Logic:**
   - What does the task do?
   - What input/output ports are required (e.g., target YOLO class `label`, alignment `threshold`, coordinates, durations)?
   - Which ROS topics does it subscribe to (e.g., camera detections, IMU) or publish to (`controls/hover_adjust`)?
   - Does it call any ROS service clients (e.g., `/steelhead/controls/actuators_command`)?

---

## 2. Step-by-Step Implementation

Follow the 3-step convention in [`src/steelhead_mission/README.md`](../../src/steelhead_mission/README.md):

### Step 1: Implement Header File
Create `src/steelhead_mission/include/steelhead_mission/tasks/<task_name>.hpp`:
- Header guard: `STEELHEAD_MISSION__TASKS__<TASK_NAME_UPPERCASE>`
- Inherit from `BT::StatefulActionNode`
- Constructor signature:
  ```cpp
  <TaskName>(
      const std::string & name,
      const BT::NodeConfig & config,
      rclcpp::Node::SharedPtr node);
  ```
- Declare `static BT::PortsList providedPorts()`
- Declare lifecycle methods:
  - `BT::NodeStatus onStart() override;`
  - `BT::NodeStatus onRunning() override;`
  - `void onHalted() override;`
- Member variables:
  - `rclcpp::Node::SharedPtr node_;`
  - Subscriptions, publishers, and service clients.
- **Rule:** Do NOT create `rclcpp::TimerBase` inside BT nodes. Never publish commands inside subscription callbacks; callbacks only cache the latest message.
- **Rule:** Always publish zero wrench on `controls/hover_adjust` in `onHalted()` and when returning final statuses.

### Step 2: Implement Source File
Create `src/steelhead_mission/src/tasks/<task_name>.cpp`:
- Implement constructor, `providedPorts()`, `onStart()`, `onRunning()`, and `onHalted()`.
- Read ports with `getInput("<port_name>", var)` in `onStart()`.

### Step 3: Register Node and Update Build Configuration
1. **Register in Factory:**
   In [`src/steelhead_mission/src/register_nodes.cpp`](../../src/steelhead_mission/src/register_nodes.cpp):
   - Include header: `#include "steelhead_mission/tasks/<task_name>.hpp"`
   - Register in `mission::registerTaskNodes`:
     ```cpp
     factory.registerNodeType<<TaskClass>>("<XmlTagName>", node);
     ```

2. **Update CMakeLists.txt:**
   In [`src/steelhead_mission/CMakeLists.txt`](../../src/steelhead_mission/CMakeLists.txt):
   - Add `src/tasks/<task_name>.cpp` to `add_library(steelhead_mission_tasks SHARED ...)`
