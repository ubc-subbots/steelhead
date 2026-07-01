# steelhead_tasks
## Description

This package hosts component nodes that subscribe to sensors (or sensor-derived
detections) and perform automated tasks for the Robosub competition. Task nodes are written as C++ composable
components so they can be loaded into the `steelhead_pipeline` component container
and sequenced as part of a mission. Each task notifies the pipeline manager via a
`steelhead_interfaces/msg/PipelineFeedback` message once its action is complete so
the pipeline sequence can advance.

`automated_competition_gate` is the reference example — copy it as a starting
point for new task nodes.

## Nodes

- `automated_competition_gate` : A component node
  (`steelhead_tasks::AutomatedCompetitionGate`) which drives the AUV through the
  competition gate using the the bounding box of image attached to the competition gate.

    ### Subscribed Topics
    - `gate/detector/gate_pose` (`steelhead_interfaces/msg/ObjectOffset`) : Gate
      pose in the robot frame from the gate detector.

    ### Published Topics
    - `/steelhead/pipeline_feedback` (`steelhead_interfaces/msg/PipelineFeedback`) :
      Pipeline feedback, published once the gate has been passed.

- `timed_wrench` : A component node (`steelhead_tasks::TimedWrench`) which
  publishes a constant, parameter-configured wrench to the hover_at_depth
  adjustment topic for a parameter-configured duration, then publishes a zero
  wrench and reports success. Intended for open-loop maneuvers such as gate
  style spins (`torque_z` only) or blind forward legs (`force_x` only). The
  wrench (`force_x/y/z`, `torque_x/y/z`), `duration`, `success_message`, and
  `control_period` are node parameters; all but `control_period` are re-read
  every control tick so they can be set from a pipeline param file (e.g.
  `config/style_spin.yaml`) or live with `ros2 param set`.

    ### Subscribed Topics
    - None.

    ### Published Topics
    - `controls/hover_adjust` (`geometry_msgs/msg/Wrench`) : The configured
      wrench while the duration is running, then a zero wrench.
    - `/steelhead/pipeline_feedback` (`steelhead_interfaces/msg/PipelineFeedback`) :
      Pipeline feedback, published once the duration has elapsed.

- `yaw_spin` : A component node (`steelhead_tasks::YawSpin`) which spins the
  AUV a configurable number of full yaw turns using the IMU, ending on the
  same heading it started at (for gate style points). It latches the starting
  heading on the first IMU message, commands a yaw adjustment while
  accumulating the unwrapped yaw travel, then trims back onto the start
  heading before reporting success. Parameters `turns` (sign sets direction),
  `yaw_tolerance`, and `success_message` are re-read every control tick so
  they can be set from a pipeline param file (e.g. `config/style_spin.yaml`)
  or live with `ros2 param set`. Note yaw adjustments override
  hover_at_depth's `hold_yaw` parameter.

    ### Subscribed Topics
    - `drivers/imu/out` (`sensor_msgs/msg/Imu`) : AUV orientation, used to
      latch the start heading and count the rotation.

    ### Published Topics
    - `controls/hover_adjust` (`geometry_msgs/msg/Wrench`) : Yaw-only wrench
      while spinning and trimming, then a zero wrench.
    - `/steelhead/pipeline_feedback` (`steelhead_interfaces/msg/PipelineFeedback`) :
      Pipeline feedback, published once the spin is complete and the heading
      is back on the start.

## Contributors

- Dorson Tang (apersonnameddorson@gmail.com)
