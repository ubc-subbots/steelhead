
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    waypoint_marker = Node(
        name="waypoint_marker",
        namespace="/steelhead/controls",
        package="steelhead_controls",
        executable="waypoint_marker",
        output="screen",
    )

    ld.add_action(waypoint_marker)

    return ld
