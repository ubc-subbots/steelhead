import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    log_level = LaunchConfiguration('log_level')

    ld = LaunchDescription([log_level_arg])
    

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_pid_controller'), 'launch', 'steelhead_pid_controller_launch.py')
        )
    )

    waypoint_marker = Node(
        package='steelhead_controls', 
        executable='waypoint_marker',
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    # waypoint_marker_tester = Node(
    #     package='steelhead_controls',
    #     executable='waypoint_marker_tester.py',
    #     name='waypoint_marker_tester',
    #     output='screen', 
    #     parameters=[{'use_sim_time': True}]
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': 'competition.world'}.items()
    )

    config = os.path.join(
        get_package_share_directory('steelhead_localization'),
        'config',
        'state_estimator_config.yaml'
    )

    state_estimator = Node(
        name='state_estimator',
        namespace='/steelhead/controls/ukf',
        package='robot_localization',
        executable='ukf_node',
        output='screen',
        parameters=[config, {'use_sim_time': True}]
    )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'state_publisher_launch.py')
        )
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    gate_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_gate'), 'launch', 'gate_detector_launch.py')
        )
    )

    underwater_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_gazebo') + '/launch/underwater_camera_launch.py'
        )
    )

    trajectory_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_controls') + '/launch/trajectory_generator_launch.py'
        )
    )

    ld.add_action(gazebo)
    # ld.add_action(rviz)
    ld.add_action(thrust_allocator)
    ld.add_action(gate_detector)
    ld.add_action(state_publisher)
    ld.add_action(underwater_camera)
    ld.add_action(state_estimator)
    ld.add_action(waypoint_marker)
    # ld.add_action(waypoint_marker_tester)
    ld.add_action(pid_controller)
    ld.add_action(trajectory_generator)

    return ld