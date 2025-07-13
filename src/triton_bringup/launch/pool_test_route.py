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
                    package='triton_controls',
                    plugin='triton_controls::SerialSubscriber',
                    name='serial_subscriber'),
            ],
            output='both',
    ) 

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_bringup'), 'launch', 'imu_launch.py')
        )
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_bringup'), 'launch', 'cameras_launch.py')
        )
    )

    gate_detector = ComposableNode(
        name='detector',
        namespace='/triton/gate',
        package='triton_gate',
        plugin='triton_gate::GateDetector',
        parameters=[
            {'debug': False}
        ]
    )

    gate_container = ComposableNodeContainer(
        name='gate_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            gate_detector
        ],
        output='screen'
    )


    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_pid_controller'), 'launch', 'triton_pid_controller_launch.py')
        )
    )

    waypoint_marker = Node(
        package='triton_controls', 
        executable='waypoint_marker',
        output='screen', 
        # parameters=[{'use_sim_time': True}]
    )

    ta_config = os.path.join(
        get_package_share_directory('triton_controls'),
        'config',
        'thruster_config_triton_mini.yaml'
    )

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

    predetermined_route = Node(
        package='triton_controls',
        executable='predetermined_route.py',
        name='predetermined_route',
        output='screen', 
    )

    ld.add_action(serial)
    ld.add_action(imu)
    ld.add_action(cameras)
    ld.add_action(pid_controller)
    ld.add_action(waypoint_marker)
    ld.add_action(thrust_allocator)
    ld.add_action(predetermined_route)
    
    # we don't really need these, but we could use them for data collection
    # uncomment to launch them too
    # ld.add_action(gate_container)
    # ld.add_action(camera1)
    # ld.add_action(camera2)


    return ld
