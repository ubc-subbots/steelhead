from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ssh_teleop = Node(
        name='ssh_teleop',
        namespace='/triton/teleop',
        package='triton_teleop',
        executable='ssh_keyboard_teleop',
        output='screen',
    )

    ld.add_action(ssh_teleop)

    return ld