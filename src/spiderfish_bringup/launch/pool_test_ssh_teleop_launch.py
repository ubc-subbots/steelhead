from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    serial = ComposableNodeContainer(
            name='serial_subscriber_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='spiderfish_controls',
                    plugin='spiderfish_controls::SerialSubscriber',
                    name='serial_subscriber'),
            ],
            output='both',
    ) 

    ta_config = os.path.join(
        get_package_share_directory('spiderfish_controls'),
        'config',
        'thruster_config.yaml'
    )

    thrust_allocator = Node(
        name='thrust_allocator',
        namespace='/spiderfish/controls',
        package='spiderfish_controls',
        executable='thrust_allocator',
        output='screen',
        parameters=[ta_config],
        remappings=[
            ('/spiderfish/controls/signals', '/motor_control')
        ]
    )

    # ssh_teleop = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('spiderfish_teleop'), 'launch', 'ssh_teleop_launch.py')
    #     )
    # )

    # RUN ros2 run spiderfish_teleop ssh_keyboard_teleop INSTEAD

    ld.add_action(serial)
    ld.add_action(thrust_allocator)
    # ld.add_action(ssh_teleop)
    
    return ld
