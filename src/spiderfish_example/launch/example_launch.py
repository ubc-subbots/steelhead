from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    component_one = ComposableNode(
        name='component_one',
        namespace='/spiderfish',
        package='spiderfish_example',
        plugin='spiderfish_example::ComponentOne'
    )

    component_two = ComposableNode(
        name='component_two',
        namespace='/spiderfish',
        package='spiderfish_example',
        plugin='spiderfish_example::ComponentTwo'
    )

    example_container = ComposableNodeContainer(
        name='example_container',
        namespace='/spiderfish',
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