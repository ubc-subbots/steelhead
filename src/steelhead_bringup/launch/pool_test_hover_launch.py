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
                    package='steelhead_controls',
                    plugin='steelhead_controls::SerialSubscriber',
                    name='serial_subscriber'),
            ],
            output='both',
    ) 


    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_bringup'), 'launch', 'imu_launch.py')
        )
    )

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_pid_controller'), 'launch', 'steelhead_pid_controller_launch.py')
        )
    )

    ta_config = os.path.join(
        get_package_share_directory('steelhead_controls'),
        'config',
        'thruster_config_steelhead.yaml'
    )

    thrust_allocator = Node(
        name='thrust_allocator',
        namespace='/steelhead/controls',
        package='steelhead_controls',
        executable='thrust_allocator',
        output='screen',
        parameters=[ta_config],
        remappings=[
            ('/steelhead/controls/signals', '/motor_control')
        ]
    )

    hover = Node(
        package='steelhead_controls', 
        executable='hover_at_depth',
        parameters=[{'depth': 0.5, 'adjust_yaw': False}],
        namespace="steelhead"
    )

    delay_hover = TimerAction(
        period=10.0,
        actions=[hover]
    )

    pressure_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'depth_sensor_publisher_launch.py')
        )
    )

    pid_teleop = Node(
        name='keyboard_teleop',
        namespace='/steelhead/teleop',
        package='steelhead_teleop',
        executable='keyboard_teleop',
        output='screen',
        parameters=[{'publish_topic': "/steelhead/controls/hover_adjust"}],
    )

    ld.add_action(serial)
    ld.add_action(imu)
    ld.add_action(cameras)
    ld.add_action(thrust_allocator)
    ld.add_action(delay_hover) # delay starting the hoverscript to let the imu calibrate. During this time, should move around the robot in figure 8s
    ld.add_action(pressure_sensor)
    ld.add_action(pid_teleop)
    
    return ld
