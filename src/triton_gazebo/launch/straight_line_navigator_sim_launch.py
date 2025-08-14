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

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description="Set to 'true' to run gazebo headless"
    )

    log_level = LaunchConfiguration('log_level')

    ld = LaunchDescription([log_level_arg, headless_arg])
    
    # Gazebo simulation with competition world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={
            'world': 'competition.world',
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    # State estimator configuration for simulation (using ground truth)
    config = os.path.join(
        get_package_share_directory('triton_controls'),
        'config',
        'state_estimator_config_GT_only.yaml'
    )

    # UKF state estimator for odometry
    state_estimator = Node(
        name='state_estimator',
        namespace='/triton/controls/ukf',
        package='robot_localization',
        executable='ukf_node',
        output='screen',
        parameters=[config, {'use_sim_time': True}]
    )

    # State publisher for robot transforms
    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'state_publisher_launch.py')
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
        parameters=[{'use_sim_time': True}]
    )

    # Thrust allocator for motor control in simulation
    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    # Straight line navigator - the main component being tested
    straight_line_navigator = Node(
        package='triton_controls',
        executable='straight_line_navigator',
        name='straight_line_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Add underwater camera for visual feedback (optional)
    underwater_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'underwater_camera_launch.py')
        )
    )

    # # RViz for visualization (optional)
    # rviz_config = os.path.join(
    #     get_package_share_directory('triton_gazebo'),
    #     'config',
    #     'rviz_config.rviz'
    # )
    
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    # Launch sequence with proper timing
    ld.add_action(gazebo)
    ld.add_action(TimerAction(period=3.0, actions=[state_publisher]))
    ld.add_action(TimerAction(period=5.0, actions=[state_estimator]))
    ld.add_action(TimerAction(period=7.0, actions=[thrust_allocator]))
    ld.add_action(TimerAction(period=9.0, actions=[waypoint_marker]))
    ld.add_action(TimerAction(period=11.0, actions=[pid_controller]))
    ld.add_action(TimerAction(period=13.0, actions=[straight_line_navigator]))
    ld.add_action(TimerAction(period=15.0, actions=[underwater_camera]))
    # ld.add_action(TimerAction(period=17.0, actions=[rviz]))

    return ld