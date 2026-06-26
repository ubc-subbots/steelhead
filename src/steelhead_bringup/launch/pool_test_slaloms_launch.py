from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # --- Hardware drivers (identical to pool_test_gate_launch.py) ---

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
            os.path.join(get_package_share_directory('steelhead_bringup'), 'imu_launch.py')
        )
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_bringup'), 'cameras_launch.py')
        )
    )

    # --- Slalom detector (replaces gate_container from pool_test_gate_launch.py) ---

    slalom_detector = ComposableNode(
        name='detector',
        namespace='/steelhead/slaloms',
        package='steelhead_slaloms',
        plugin='steelhead_slaloms::SlalomDetector',
        parameters=[
            {'debug': False}
        ]
    )

    slalom_container = ComposableNodeContainer(
        name='slalom_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            slalom_detector
        ],
        output='screen'
    )

    # --- Controls stack (identical to pool_test_gate_launch.py) ---

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('steelhead_pid_controller'),
                'launch',
                'steelhead_pid_controller_launch.py'
            )
        )
    )

    waypoint_marker = Node(
        package='steelhead_controls',
        executable='waypoint_marker',
        output='screen',
    )

    ta_config = os.path.join(
        get_package_share_directory('steelhead_controls'),
        'config',
        'thruster_config.yaml'
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

    trajectory_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_controls') + '/launch/trajectory_generator_launch.py'
        )
    )

    # --- Compose ---

    ld.add_action(serial)
    ld.add_action(imu)
    ld.add_action(cameras)
    ld.add_action(pid_controller)
    ld.add_action(waypoint_marker)
    ld.add_action(thrust_allocator)
    ld.add_action(slalom_container)
    ld.add_action(trajectory_generator)

    return ld