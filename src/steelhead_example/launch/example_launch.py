from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    component_one = ComposableNode(
        name='component_one',
        namespace='/steelhead',
        package='steelhead_example',
        plugin='steelhead_example::ComponentOne'
    )

    component_two = ComposableNode(
        name='component_two',
        namespace='/steelhead',
        package='steelhead_example',
        plugin='steelhead_example::ComponentTwo'
    )

    example_container = ComposableNodeContainer(
        name='example_container',
        namespace='/steelhead',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            component_one,
            component_two
        ],
        output='screen'
    )

    ld.add_action(example_container)

    return ld 