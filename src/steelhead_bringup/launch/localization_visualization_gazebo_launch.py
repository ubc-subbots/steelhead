import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': 'competition.world'}.items()
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('steelhead_bringup'), 'config', 'localization_visualization.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}], 
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

    # vins_odometry = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('steelhead_localization'), 'launch', 'vins_odometry_launch.py')
    #     )
    # )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'state_publisher_launch.py')
        )
    )

    keyboard_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_teleop'), 'launch', 'keyboard_teleop_launch.py')
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

    static_tf_vins = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    # Arguments: x y z yaw pitch roll frame_id child_frame_id
    # This aligns the VINS world/odom frame with the ROS AUV body convention
    arguments=['0', '0', '0', '1.571', '0', '1.571', 'odom', 'vins_world_frame'],
    parameters=[{'use_sim_time': True}]
    )
    ld.add_action(static_tf_vins)
    
    ld.add_action(gazebo)
    ld.add_action(rviz)
    ld.add_action(thrust_allocator)
    ld.add_action(keyboard_teleop)
    ld.add_action(gate_detector)
    ld.add_action(state_publisher)
    # ld.add_action(underwater_camera) # the underwater camera simulator isn't that good and is very taxing on performance, so i'm disabling it for now
    ld.add_action(state_estimator)
    # ld.add_action(vins_odometry)

    return ld