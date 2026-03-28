# steelhead_mission_planner

A configurable, decision-tree mission planner for the Steelhead AUV. It sequences through competition tasks (Gate, Buoy, Torpedo, Octagon, …) by reading completion strings published by other nodes, and broadcasts the current mission state on the `/State` topic.

This node is **completely independent** of `PipelineSequenceManager` — it has no knowledge of pipelines, ROS2 actions, or component loading.

---

## Table of Contents

- [Overview](#overview)
- [Topics](#topics)
- [Decision Tree Logic](#decision-tree-logic)
- [Configuration](#configuration)
- [Adding or Modifying States](#adding-or-modifying-states)
- [Build and Run](#build-and-run)
- [Relationship to PipelineSequenceManager](#relationship-to-pipelinesequencemanager)

---

## Overview

At startup the planner loads a decision tree from `config/mission_tree.yaml`. It enters the configured `initial_state`, publishes it on `/State`, and then waits for a completion message on that state's designated topic. Based on the string received (`"complete"` or `"abort"`), it follows the matching transition to the next state. This repeats until the special terminal state `"DONE"` is reached.

```
Start → Gate
         ├─ complete → Buoy
         │              ├─ complete → Torpedo
         │              │              ├─ complete → Octagon → DONE
         │              │              └─ abort   → Octagon → DONE
         │              └─ abort   → Octagon → DONE
         └─ abort   → Octagon → DONE
```

---

## Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Publishes | `/State` | `std_msgs/String` | Current mission state (e.g. `"Gate"`, `"Buoy"`, `"DONE"`). Re-published at 1 Hz. |
| Subscribes | `/Gate_Complete` | `std_msgs/String` | Completion signal from the Gate task node. |
| Subscribes | `/Buoy_Complete` | `std_msgs/String` | Completion signal from the Buoy task node. |
| Subscribes | `/Torpedo_Complete` | `std_msgs/String` | Completion signal from the Torpedo task node. |
| Subscribes | `/Octagon_Complete` | `std_msgs/String` | Completion signal from the Octagon task node. |

> Completion topics are fully configurable via `mission_tree.yaml`. The table above reflects the default config.

### Completion string protocol

Each task node is expected to publish one of three string values to its completion topic:

| Value | Meaning |
|-------|---------|
| `"undecided"` | Task is still running — planner ignores this. |
| `"complete"` | Task succeeded — planner follows the `on_complete` transition. |
| `"abort"` | Task failed — planner follows the `on_abort` transition. |

---

## Decision Tree Logic

1. Planner enters `initial_state` and publishes it on `/State`.
2. It subscribes to **all** completion topics at startup, but only processes messages for the **currently active** state's topic.
3. `"undecided"` messages are silently ignored — the task is still running.
4. `"complete"` triggers the `on_complete` transition for that state.
5. `"abort"` triggers the `on_abort` transition for that state.
6. On every transition `/State` is immediately republished and a log message is emitted.
7. Entering `"DONE"` ends the mission — no further transitions occur.

---

## Configuration

The decision tree is defined in `config/mission_tree.yaml`. It is loaded as ROS2 parameters at launch.

```yaml
/mission_planner:
  ros__parameters:

    initial_state: "Gate"

    states:
      - "Gate"
      - "Buoy"
      - "Torpedo"
      - "Octagon"

    Gate.listen_topic: "/Gate_Complete"
    Gate.on_complete:  "Buoy"
    Gate.on_abort:     "Octagon"

    Buoy.listen_topic: "/Buoy_Complete"
    Buoy.on_complete:  "Torpedo"
    Buoy.on_abort:     "Octagon"

    Torpedo.listen_topic: "/Torpedo_Complete"
    Torpedo.on_complete:  "Octagon"
    Torpedo.on_abort:     "Octagon"

    Octagon.listen_topic: "/Octagon_Complete"
    Octagon.on_complete:  "DONE"
    Octagon.on_abort:     "DONE"
```

### Parameter reference

| Parameter | Type | Description |
|-----------|------|-------------|
| `initial_state` | `string` | Name of the first state entered on startup. |
| `states` | `string[]` | List of all state names to load from parameters. |
| `<State>.listen_topic` | `string` | Topic to subscribe to for this state's completion signal. |
| `<State>.on_complete` | `string` | Next state when `"complete"` is received. |
| `<State>.on_abort` | `string` | Next state when `"abort"` is received. |

---

## Adding or Modifying States

1. **Add the state name** to the `states` list in `mission_tree.yaml`.
2. **Add three parameters** for the new state:
   ```yaml
   MyTask.listen_topic: "/MyTask_Complete"
   MyTask.on_complete:  "NextState"
   MyTask.on_abort:     "FallbackState"
   ```
3. **Wire upstream states** — update the `on_complete` or `on_abort` of whichever state should transition into the new one.
4. **Implement the task node** — it must publish `"undecided"`, `"complete"`, or `"abort"` to the configured topic.

No C++ changes are required. The planner automatically subscribes to any topic listed in the config.

---

## Build and Run

```bash
# From the workspace root
colcon build --packages-select steelhead_mission_planner
source install/setup.bash

# Launch the planner
ros2 launch steelhead_mission_planner mission_planner.launch.py
```

To monitor the current mission state:

```bash
ros2 topic echo /State
```

To manually simulate a task completion (useful for testing):

```bash
# Simulate Gate completing successfully
ros2 topic pub --once /Gate_Complete std_msgs/msg/String "data: 'complete'"

# Simulate Buoy aborting
ros2 topic pub --once /Buoy_Complete std_msgs/msg/String "data: 'abort'"
```

---

## Relationship to PipelineSequenceManager

`PipelineSequenceManager` (in `steelhead_pipeline`) is a **linear sequencer** — it reads a flat list of pipeline names from a YAML file and runs them one after another unconditionally. If any step aborts, it stops and returns an error. It is tightly coupled to the ROS2 action/service pipeline-loading system and has **no branching capability**.

`MissionPlanner` is a **decision-tree orchestrator** — it branches based on real-time feedback from task nodes, can skip or reroute tasks on failure, and is completely decoupled from the pipeline system. The two can coexist: pipeline-based task nodes can publish their result to a completion topic, which the `MissionPlanner` then acts on.
