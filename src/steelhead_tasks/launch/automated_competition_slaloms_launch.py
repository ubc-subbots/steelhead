from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    nav_node = Node(
        package="steelhead_tasks",
        executable="automated_competition_slaloms.py",
        name="automated_competition_slaloms",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    ld.add_action(nav_node)

    return ld
