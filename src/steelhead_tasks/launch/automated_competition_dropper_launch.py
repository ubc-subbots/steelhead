from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    automated_competition_dropper = ComposableNode(
        name="automated_competition_dropper",
        namespace="/steelhead",
        package="steelhead_tasks",
        plugin="steelhead_tasks::AutomatedCompetitionDropper",
    )

    tasks_container = ComposableNodeContainer(
        name="tasks_container",
        namespace="/steelhead",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[automated_competition_dropper],
        output="screen",
    )

    ld.add_action(tasks_container)

    return ld
