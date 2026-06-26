from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    slalom_detector = ComposableNode(
        name='detector',
        namespace='/steelhead/slaloms',
        package='steelhead_slaloms',
        plugin='steelhead_slaloms::SlalomDetector',
        parameters=[
            {'debug': True}
        ]
    )

    slalom_container = ComposableNodeContainer(
        name='slalom_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            slalom_detector
        ],
        output='screen'
    )

    ld.add_action(slalom_container)
    return ld