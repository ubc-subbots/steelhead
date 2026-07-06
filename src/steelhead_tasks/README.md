# steelhead_tasks
## Description

This package hosts component nodes that subscribe to sensors (or sensor-derived
detections) and perform automated tasks for the Robosub competition. Task nodes are written as C++ composable
components so they can be loaded into the `steelhead_pipeline` component container
and sequenced as part of a mission. Each task notifies the pipeline manager via a
`steelhead_interfaces/msg/PipelineFeedback` message once its action is complete so
the pipeline sequence can advance.

## Nodes

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

## Contributors

- Dorson Tang (dtang08@student.ubc.ca)