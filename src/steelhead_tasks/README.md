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

- `automated_competition_torpedo` : A node that uses the front camera to center a
  detection horizontally, then fires both torpedoes via the actuators service once
  aligned.

    ### Subscribed Topics
    - `yolo_detector/front/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Front camera detections.
    - `drivers/front_camera/camera_info` (`sensor_msgs/msg/CameraInfo`) : Read once for the image width.

    ### Published Topics
    - `controls/hover_adjust` (`steelhead_interfaces/msg/HoverAdjustment`) : Partial wrench to center on the target.
    - `/steelhead/pipeline_feedback` (`steelhead_interfaces/msg/PipelineFeedback`) : Reports success after firing.

    ### Service Clients
    - `/steelhead/controls/actuators_command` (`steelhead_interfaces/srv/ActuatorsCommand`) : Fires the torpedoes.

    ### Parameters
    - `target_detection_label` (string, default `"circle"`) : Detection label to home in on.
    - `center_offset` (double, default 0.0) : Desired horizontal box position as a fraction of half the image width (0 = image centre).
    - `fire_alignment_threshold` (double, default 0.1) : Fire once `|offset error|` drops below this.

- `automated_competition_dropper` : A node that uses the bottom camera to center a
  detection in both axes, then drops markers by opening the claw via the actuators
  service once centered.

    ### Subscribed Topics
    - `yolo_detector/bottom/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Bottom camera detections.
    - `drivers/bottom_camera/camera_info` (`sensor_msgs/msg/CameraInfo`) : Read once for the image width and height.

    ### Published Topics
    - `controls/hover_adjust` (`steelhead_interfaces/msg/HoverAdjustment`) : Partial wrench to center over the target.
    - `/steelhead/pipeline_feedback` (`steelhead_interfaces/msg/PipelineFeedback`) : Reports success after dropping.

    ### Service Clients
    - `/steelhead/controls/actuators_command` (`steelhead_interfaces/srv/ActuatorsCommand`) : Opens/closes the claw.

    ### Parameters
    - `target_detection_label` (string, default `"fire"`) : Detection label to home in on.
    - `center_offset_x`, `center_offset_y` (double, default 0.0 / 0.1) : Desired box position as a fraction of half the image size (0 = image centre).
    - `drop_alignment_threshold` (double, default 0.1) : Drop once both axis errors drop below this.

- `automated_competition_octagon` : A node that uses the bottom camera to center a
  detection in both axes, then applies a timed upward force to surface inside the
  octagon (no actuators).

    ### Subscribed Topics
    - `yolo_detector/bottom/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Bottom camera detections.
    - `drivers/bottom_camera/camera_info` (`sensor_msgs/msg/CameraInfo`) : Read once for the image width and height.

    ### Published Topics
    - `controls/hover_adjust` (`steelhead_interfaces/msg/HoverAdjustment`) : Partial wrench to center over the target, then the upward force while surfacing.
    - `/steelhead/pipeline_feedback` (`steelhead_interfaces/msg/PipelineFeedback`) : Reports success after the surfacing push completes.

    ### Parameters
    - `target_detection_label` (string, default `"helmet"`) : Detection label to home in on.
    - `center_offset_x`, `center_offset_y` (double, default 0.0) : Desired box position as a fraction of half the image size (0 = image centre).
    - `surface_alignment_threshold` (double, default 0.3) : Start surfacing once both axis errors drop below this.
    - `surface_duration` (double, default 10.0) : How long to apply the upward force before reporting success, in seconds.

- `automated_competition_slalom` : A node that navigates the three-set slalom. Each
  set is a WHITE / RED / WHITE triplet of vertical pipes, but only the red pipe is
  detected, so the node navigates off the red pipe alone: it holds the red pipe at a
  fixed offset to one side of the image while driving forward, which carries the AUV
  through the gap on the other side of it. The offset keeps the same sign for every
  set, so the AUV passes on the same side of the red pipe each time and earns the
  bonus for it. The sets being offset laterally from each other needs no special
  handling, since the node only servos on the pipe it can see and the hover node
  holds yaw so the AUV strafes across rather than turning.

  Several sets are in frame at once, so the node tracks the tallest red box rather
  than the most confident one: the pipes are all the same height, so the tallest box
  is the nearest pipe. It commits to a pipe once its box grows past
  `commit_box_height` of the frame, and counts the set once that committed pipe
  leaves the view and it has coasted forward for `pass_timeout`. The coast ignores
  detections, so the next set coming into view cannot restart the timer. It then
  hunts for the next set, reporting success after `num_sets` sets.

  Depth is not commanded, so the hover node holds whatever depth the pipeline set
  beforehand. Set that depth within the 0.9m span of the pipes to earn the depth bonus.

    ### Subscribed Topics
    - `yolo_detector/front/detections` (`steelhead_interfaces/msg/DetectionBoxArray`) : Front camera detections.
    - `drivers/front_camera/camera_info` (`sensor_msgs/msg/CameraInfo`) : Read once for the image width.

    ### Published Topics
    - `controls/hover_adjust` (`steelhead_interfaces/msg/HoverAdjustment`) : Partial wrench to hold the pipe at the offset and drive past it.
    - `/steelhead/pipeline_feedback` (`steelhead_interfaces/msg/PipelineFeedback`) : Reports success after the last set is passed.

    ### Parameters
    - `target_detection_label` (string, default `"slalom"`) : Detection label for the red slalom pipe.
    - `pass_side` (string, default `"left"`) : Which side of the red pipe to pass on, `"left"` or `"right"` as seen in the image. Kept constant across all sets.
    - `pass_offset` (double, default 0.4) : How far to hold the pipe off the image centre, as a fraction of half the image width. Larger gives the pipe a wider berth.
    - `num_sets` (int, default 3) : How many pipe sets to pass before reporting success.
    - `commit_box_height` (double, default 0.4) : Box height, as a fraction of the image height, at which the AUV is close enough to commit to a pipe. Only a committed pipe leaving the view counts as a passed set, so a dropout at range is not mistaken for a pass.
    - `pass_timeout` (double, default 10.0) : Seconds to keep driving after losing the committed pipe before counting the set as passed.
    - `approach_force` (double, default 5.0) : Forward force while searching for and approaching a set.
    - `coast_force` (double, default 15.0) : Forward force while coasting past a pipe.
    - `sway_gain` (double, default 15.0) : Sway force per unit of normalized horizontal error.

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