from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    timed_wrench = ComposableNode(
        name='timed_wrench',
        namespace='/steelhead',
        package='steelhead_tasks',
        plugin='steelhead_tasks::TimedWrench'
    )

    tasks_container = ComposableNodeContainer(
        name='tasks_container',
        namespace='/steelhead',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            timed_wrench
        ],
        output='screen'
    )

    ld.add_action(tasks_container)

    return ld
