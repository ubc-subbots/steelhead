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
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'imu_publisher_launch.py')
        )
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'cameras_publisher_launch.py')
        )
    )

    gate_detector = ComposableNode(
        name='detector',
        namespace='/steelhead/gate',
        package='steelhead_gate',
        plugin='steelhead_gate::GateDetector',
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
            os.path.join(get_package_share_directory('steelhead_pid_controller'), 'launch', 'steelhead_pid_controller_launch.py')
        )
    )

    waypoint_marker = Node(
        package='steelhead_controls', 
        executable='waypoint_marker',
        output='screen', 
        # parameters=[{'use_sim_time': True}]
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

    trajectory_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_controls') + '/launch/trajectory_generator_launch.py'
        )
    )

    actuators_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_controls') + '/launch/actuators_command_server_launch.py'
        )
    )

    depth_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_controls') + '/launch/depth_sensor_publisher_launch.py'
        )
    )

    hover_at_depth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_controls') + '/launch/hover_at_depth_launch.py'
        )
    )

    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_pipeline'), 'launch', 'pipeline_launch.py')
        ),
        launch_arguments={'sequence': 'competition_hardcode_sequence.yaml'}.items()
    )

    delayed_pipeline = TimerAction(
        period=5.0,
        actions=[pipeline]
    )

    ld.add_action(serial)
    # ld.add_action(micro_ros_agent)
    # ld.add_action(state_estimator)
    # ld.add_action(imu_tf)
    # ld.add_action(transform_publisher)
    ld.add_action(pid_controller)
    # ld.add_action(waypoint_marker)
    ld.add_action(thrust_allocator)
    # ld.add_action(gate_container)
    # ld.add_action(trajectory_generator) 
    # ld.add_action(keyboard_teleop)
    ld.add_action(actuators_server)
    ld.add_action(depth_sensor)
    ld.add_action(hover_at_depth)
    # ld.add_action(delayed_pipeline)
    
    return ld
