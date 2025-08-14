from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    # Serial communication with AUV hardware
    serial = ComposableNodeContainer(
            name='serial_subscriber_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='triton_controls',
                    plugin='triton_controls::SerialSubscriber',
                    name='serial_subscriber'),
            ],
            output='both',
    ) 

    # IMU launch for compass and orientation data
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_bringup'), 'launch', 'imu_launch.py')
        )
    )

    # PID controller for waypoint following
    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_pid_controller'), 'launch', 'triton_pid_controller_launch.py')
        )
    )

    # Waypoint marker for waypoint management
    waypoint_marker = Node(
        package='triton_controls', 
        executable='waypoint_marker',
        output='screen',
    )

    # Thrust allocator configuration
    ta_config = os.path.join(
        get_package_share_directory('triton_controls'),
        'config',
        'thruster_config_triton_mini.yaml'
    )

    # Thrust allocator for motor control
    thrust_allocator = Node(
        name='thrust_allocator',
        namespace='/triton/controls',
        package='triton_controls',
        executable='thrust_allocator',
        output='screen',
        parameters=[ta_config],
        remappings=[
            ('/triton/controls/signals', '/motor_control')
        ]
    )

    # Straight line navigator - the main component
    straight_line_navigator = ComposableNodeContainer(
        name='straight_line_navigator_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='triton_controls',
                plugin='triton_controls::StraightLineNavigator',
                name='straight_line_navigator'),
        ],
        output='screen',
    )

    # Add all components to launch description
    ld.add_action(serial)
    ld.add_action(imu)
    ld.add_action(pid_controller)
    ld.add_action(waypoint_marker)
    ld.add_action(thrust_allocator)
    ld.add_action(straight_line_navigator)

    return ld