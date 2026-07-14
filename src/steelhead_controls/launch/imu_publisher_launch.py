import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    # The D455 has no fusion processor, so orientation is not sensed. The chain
    # below rebuilds it: the camera emits raw accel/gyro on optical axes,
    # d455_imu_publisher puts those on body axes, and imu_filter_madgwick fuses
    # them into the /steelhead/drivers/imu/out contract the BNO085 used to hold.
    d455 = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        parameters=[{
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,  # 0-none, 1-copy, 2-linear_interpolation
            'gyro_fps': 200,
            'accel_fps': 250,
            # Nothing here consumes the image streams; the two mission cameras
            # come up separately in cameras_publisher_launch.py
            'enable_color': False,
            'enable_depth': False,
            'enable_infra1': False,
            'enable_infra2': False,
        }],
    )

    d455_imu_publisher = Node(
        name='d455_imu_publisher',
        namespace='/steelhead/controls',
        package='steelhead_controls',
        executable='d455_imu_publisher.py',
        output='screen',
        parameters=[{
            'input_topic': '/camera/imu',
            'frame_id': 'imu_link',
            # Gyro bias (rad/s, body axes). Yaw is integrated open-loop with no
            # magnetometer to reference it, so bias on z becomes yaw drift that
            # nothing downstream corrects. Measure with:
            #   ros2 run steelhead_controls measure_gyro_bias.py
            'gyro_bias': [0.0, 0.0, 0.0],
        }],
    )

    imu_filter_config = os.path.join(
        get_package_share_directory('steelhead_controls'),
        'config',
        'imu_filter.yaml',
    )

    imu_filter = Node(
        name='imu_filter',
        namespace='/steelhead/controls',
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output='screen',
        parameters=[imu_filter_config],
        remappings=[
            ('imu/data_raw', '/steelhead/drivers/imu/data_raw'),
            ('imu/data', '/steelhead/drivers/imu/out'),
        ],
    )

    ld.add_action(d455)
    ld.add_action(d455_imu_publisher)
    ld.add_action(imu_filter)

    return ld
