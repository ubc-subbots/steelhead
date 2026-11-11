---
name: create-task-node
description: >-
  Use this skill when the user wants to generate boilerplate for
  a new BehaviorTree.CPP task action node in steelhead_mission. Guides gathering
  interface requirements and generates all necessary C++ boilerplate (with placeholder
  lifecycle methods), registrations, and build configuration updates without implementing
  task functionality.
---

# Create Task Node Boilerplate

This skill provides step-by-step instructions to scaffold a new BehaviorTree.CPP task action node in `steelhead_mission`. The agent generates only the boilerplate code and registrations; task-specific functionality is left for the user to implement.

---

## 1. Gather Requirements

Ask the user only for minimal boilerplate interface requirements:
1. **Desired Task Name:**
   - PascalCase class name (e.g., `GateTask`), XML tag name (e.g., `Gate`), and snake_case file name (e.g., `gate_task`).
2. **Interface Requirements (Ports / Topics / Services):**
   - What input/output ports are required (names, types, defaults, descriptions)?
   - Which ROS topics does it subscribe to (e.g., camera detections, IMU) or publish to (e.g., `controls/hover_adjust`)?
   - Does it call any ROS service clients (e.g., `/steelhead/controls/actuators_command`)?

Do NOT ask the user for task execution logic or algorithms. The agent will generate placeholder lifecycle methods.

---

## 2. Step-by-Step Implementation

Follow the 3-step convention in [`src/steelhead_mission/README.md`](../../src/steelhead_mission/README.md):

### Step 1: Implement Header File
Create `src/steelhead_mission/include/steelhead_mission/tasks/<task_name>.hpp`:
- Header guard: `STEELHEAD_MISSION__TASKS__<TASK_NAME_UPPERCASE>`
- Follow C++ conventions in `CONVENTIONS.md` (namespace `mission`, brackets on new lines, member variables `under_scored_suffix_`, Doxygen comments).
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
  - Subscriptions, publishers, and service clients if specified.
- **Rule:** Do NOT create `rclcpp::TimerBase` inside BT nodes. Never publish commands inside subscription callbacks; callbacks only cache the latest message.
- **Rule:** Always publish zero wrench on `controls/hover_adjust` in `onHalted()` and when returning final statuses (if publishing to hover_adjust).

### Step 2: Implement Source File with Placeholders
Create `src/steelhead_mission/src/tasks/<task_name>.cpp`:
- Implement constructor and `providedPorts()`.
- **Do not generate task functionality.** Implement `onStart()`, `onRunning()`, and `onHalted()` automatically as compilable placeholders:
  - **`onStart()`**:
    - Read input ports using `getInput("<port_name>", ...)`.
    - Log start: `RCLCPP_INFO(node_->get_logger(), "<TaskName> started");`
    - Return `BT::NodeStatus::RUNNING;`
  - **`onRunning()`**:
    - Placeholder return: `return BT::NodeStatus::SUCCESS;` (do NOT implement task execution logic, loops, or state machines).
  - **`onHalted()`**:
    - Log halt: `RCLCPP_WARN(node_->get_logger(), "<TaskName> halted");`
    - If `controls/hover_adjust` publisher exists, publish zero wrench.
- Follow formatting conventions (two lines between functions, brackets on new lines).

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
   - Add `src/tasks/<task_name>.cpp` to `add_library(steelhead_mission_tasks SHARED ...)`.
