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
            os.path.join(get_package_share_directory('triton_pid_controller'), 'launch', 'triton_pid_controller_launch.py')
        )
    )

    waypoint_marker = Node(
        package='triton_controls', 
        executable='waypoint_marker',
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    # waypoint_marker_tester = Node(
    #     package='triton_controls',
    #     executable='waypoint_marker_tester.py',
    #     name='waypoint_marker_tester',
    #     output='screen', 
    #     parameters=[{'use_sim_time': True}]
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': 'competition.world'}.items()
    )

    config = os.path.join(
        get_package_share_directory('triton_controls'),
        'config',
        'state_estimator_config_IMU_only.yaml'
    )

    state_estimator = Node(
        name='state_estimator',
        namespace='/triton/controls/ukf',
        package='robot_localization',
        executable='ukf_node',
        output='screen',
        parameters=[config, {'use_sim_time': True}]
    )

    pkg_share = get_package_share_directory('triton_controls')
    sdf_file =  os.path.join(pkg_share, 'models', 'triton_auv', 'model.urdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        output='screen', 
        parameters=[rsp_params, {'use_sim_time': True}]
    )

    transform_publisher = Node(
        package='triton_controls',
        executable='auv_transform_publisher.py',
        name='auv_transform_publisher',
        output='screen', 
        parameters=[rsp_params, {'use_sim_time': True}]
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    gate_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gate'), 'launch', 'gate_detector_launch.py')
        )
    )

    underwater_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_gazebo') + '/launch/underwater_camera_launch.py'
        )
    )

    qualifying_task = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_controls') + '/launch/qualifying_task_launch.py'
        )
    )

    # TODO For imu only state estimation, we're getting an error 
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'frame', 'base_link'],
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
    ld.add_action(qualifying_task)
    ld.add_action(static_tf)

    return ld