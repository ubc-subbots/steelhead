from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node

def generate_launch_description():

    ld = LaunchDescription()

    pipeline_manager = Node(
        name='pipeline_manager',
        namespace='/steelhead',
        package='steelhead_pipeline',
        executable='pipeline_manager',
        output='screen'
    )

    pipeline_container = ComposableNodeContainer(
        name='pipeline',
        namespace='/steelhead',
        package='rclcpp_components',
        executable='component_container'
    )

    ld.add_action(pipeline_manager)
    ld.add_action(pipeline_container)

    return ld