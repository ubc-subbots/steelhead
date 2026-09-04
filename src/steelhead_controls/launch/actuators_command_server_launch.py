
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    actuators_command = Node(
        name="actuators_command",
        namespace="/steelhead/controls",
        package="steelhead_controls",
        executable="actuators_command",
        output="screen",
    )

    ld.add_action(actuators_command)

    return ld
