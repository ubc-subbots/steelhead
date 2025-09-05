from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    keyboard_teleop = Node(
        name='keyboard_teleop',
        namespace='/steelhead/teleop',
        package='steelhead_teleop',
        executable='keyboard_teleop',
        output='screen',
    )

    ld.add_action(keyboard_teleop)

    return ld