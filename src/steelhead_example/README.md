# steelhead_example
## Description

This package serves as an example package displaying how to write composable ROS 2 component nodes and also to show the style conventions that are expected to be followed.

## Usage

Launch the example component container:

    ros2 launch steelhead_example example_launch.py

You can then use the following command to send data to the example components:

    ros2 topic pub /steelhead/example/component_one/in std_msgs/String '{data: "Hello World"}'

You can see the final output using the command:

    ros2 topic echo /steelhead/example/component_two/out

You should see the string `"Hello World from ComponentOne and ComponentTwo"` being output from the above command.

## Nodes

- `component_one` : A component node (`steelhead_example::ComponentOne`) which adds `"from ComponentOne"` to any string it receives.

    ### Subscribed Topics
    - `example/component_one/in` (`std_msgs/msg/String.msg`) : Input string.
    
    ### Published Topics
    - `example/component_one/out` (`std_msgs/msg/String.msg`) : Output string.
    
- `component_two` : A component node (`steelhead_example::ComponentTwo`) which adds `"and ComponentTwo"` to any string it receives and logs once it receives 25 string messages.

    ### Subscribed Topics
    - `example/component_two/in` (`std_msgs/msg/String.msg`) : Input string.
    
    ### Published Topics
    - `example/component_two/out` (`std_msgs/msg/String.msg`) : Output string.

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
