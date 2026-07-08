# steelhead_tasks
## Description

This package hosts component nodes that subscribe to sensors (or sensor-derived
detections) and perform automated tasks for the Robosub competition. Task nodes are written as C++ composable
components so they can be loaded into the `steelhead_pipeline` component container
and sequenced as part of a mission. Each task notifies the pipeline manager via a
`steelhead_interfaces/msg/PipelineFeedback` message once its action is complete so
the pipeline sequence can advance.

## Nodes

- `automated_competition_gate` : A node that uses cv and a designated label to center the bounding box and drive through. 

- `hardcode_inputs` : A component node which
  publishes a constant, parameter-configured HoverAdjustment to the hover_at_depth
  adjustment topic with a partial type for a parameter-configured duration, then publishes a zero
  wrench and reports success. Intended for blind forward legs (`force_x` only). 

    ### Subscribed Topics
    - None.

    ### Published Topics
    - `controls/hover_adjust` (`steelhead_interfaces/msg/HoverAdjustment`) : The configured
      wrench while the duration is running. This adjustment is partial, which means the robot will try to balance itself.
    - `/steelhead/pipeline_feedback` (`steelhead_interfaces/msg/PipelineFeedback`) :
      Pipeline feedback, published once the duration has elapsed.

    ### Parameters
    - `force_x`, `force_y`, `force_z`, `torque_x`, `torque_y`, `torque_z` (double, default 0.0) :
      The wrench to publish.
    - `duration` (double, default 5.0) : How long to publish the wrench for, in seconds.

- `pitch_flip` : A component node which makes the robot flip once using IMU, ending on the
  same orientation it started at (for gate style points). It latches the starting
  heading on the first IMU message, commands a flip adjustment until flip is 3pi/2 beforereporting success. 

    ### Subscribed Topics
    - `drivers/imu/out` (`sensor_msgs/msg/Imu`) : AUV orientation, used to
      latch the start heading and count the rotation.

    ### Published Topics
    - `controls/hover_adjust` (`steelhead_interfaces/msg/HoverAdjustment`) : Wrench while spinning. Uses the full adjustment type until the very end, where it publishes a zero adjustment with partial typing.
    - `/steelhead/pipeline_feedback` (`steelhead_interfaces/msg/PipelineFeedback`) :
      Pipeline feedback, published once spin is almost completed to allow the hover node to take over.

    ### Parameters
    - `flip_torque` (double, default 1.0) : The pitch torque to command while flipping.
    - `flip_angle` (double, default 3pi/2) : Accumulated pitch rotation at which to stop
      commanding the flip and hand off to the hover node.

## Contributors

- Dorson Tang (dtang08@student.ubc.ca)