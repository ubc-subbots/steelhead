from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    controller_teleop = Node(
        name='controller_teleop',
        namespace='/steelhead/teleop',
        package='steelhead_teleop',
        executable='controller_teleop',
        output='screen',
    )

    ld.add_action(controller_teleop)

    return ld